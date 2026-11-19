function syncTime() {
    const n = new Date();
    const p = `ts=${Math.floor(n.getTime()/1000)}&tzOffset=${n.getTimezoneOffset()}&tzName=${Intl.DateTimeFormat().resolvedOptions().timeZone}`;
    fetch('/set-time?' + p).then(r => r.text()).then(t => alert(t));
}

function formatTime(utc_ts, tz_offset, tzn) {
    if (!utc_ts) return '...';
    const d = new Date((utc_ts + (tz_offset * 3600)) * 1000);
    let h = d.getUTCHours();
    let m = d.getUTCMinutes();
    const ampm = h >= 12 ? 'PM' : 'AM';
    h = h % 12 || 12;
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')} ${ampm} ${tzn || 'IST'}`;
}

function fetchStatus() {
    fetch('/status?_=' + Date.now())
        .then(res => res.json())
        .catch(() => ({ error: true }))
        .then(data => {
            if (!data || data.error) return;

            const current = (typeof data.current_tilt === 'number') ? data.current_tilt : 0;
            const target = (typeof data.target_tilt === 'number') ? data.target_tilt : 0;

            document.getElementById('val-current-tilt').innerText = current.toFixed(2) + '°';
            document.getElementById('val-target-tilt').innerText = target.toFixed(2) + '°';

            let diff = Math.abs(current - target);
            let pct = Math.min(100, Math.max(0, 100 - (diff * 2)));
            document.getElementById('progress-hint').innerText = 'Sync Precision: ' + pct.toFixed(1) + '%';

            if (data.time && document.getElementById('val-current-time')) {
                const localT = new Date((data.time + (data.tz_offset * 3600)) * 1000);
                let h = localT.getUTCHours();
                const ampm = h >= 12 ? 'PM' : 'AM';
                h = h % 12 || 12;
                document.getElementById('val-current-time').innerText =
                    h.toString().padStart(2, '0') + ':' +
                    localT.getUTCMinutes().toString().padStart(2, '0') + ' ' +
                    ampm + ' ' + (data.tz_name || 'IST');
            }

            const tag = document.getElementById('mode-tag');
            if (tag) {
                tag.innerText = data.mode || '...';
                tag.className = 'tag is-rounded has-text-weight-bold ' +
                    (data.mode == 'MANUAL' ? 'is-warning' : 'is-success');
            }

            const trTag = document.getElementById('tracking-tag');
            if (trTag && data.tracking_mode) {
                let cls = 'tag is-rounded ml-1 has-text-weight-bold ';
                if (data.tracking_mode === 'Standard') {
                    cls += 'is-success';
                    trTag.style.display = 'none';
                } else if (data.tracking_mode === 'Flat Backtracking') {
                    cls += 'is-info';
                    trTag.style.display = 'inline-flex';
                } else if (data.tracking_mode === 'Slope Backtracking') {
                    cls += 'is-warning';
                    trTag.style.display = 'inline-flex';
                }
                trTag.className = cls;
                if (data.correction_active) {
                    trTag.innerText = data.tracking_mode + ' ⚡';
                } else {
                    trTag.innerText = data.tracking_mode;
                }
            }

            const mUI = document.getElementById('ctrl-manual');
            const aUI = document.getElementById('ctrl-auto');
            if (data.mode === 'MANUAL') {
                mUI.style.display = 'block';
                aUI.style.display = 'none';
            } else {
                mUI.style.display = 'none';
                aUI.style.display = 'block';
                // Prefill lat/long if they exist and are not already focused
                if (data.lat !== undefined && document.activeElement.id !== 'in-lat') document.getElementById('in-lat').value = data.lat;
                if (data.lon !== undefined && document.activeElement.id !== 'in-lon') document.getElementById('in-lon').value = data.lon;
            }

            const elT30 = document.getElementById('val-target-30');
            const elNext = document.getElementById('next-move-hint');
            if (elT30 && elNext) {
                if (data.mode === 'MANUAL') {
                    elT30.innerText = '---';
                    elNext.innerText = 'Auto tracking disabled';
                } else if (data.target_30 !== undefined) {
                    elT30.innerText = data.target_30.toFixed(2) + '°';
                    const nT = new Date((data.target_30_time + (data.tz_offset * 3600)) * 1000);
                    let nh = nT.getUTCHours();
                    const nampm = nh >= 12 ? 'PM' : 'AM';
                    nh = nh % 12 || 12;
                    elNext.innerText = 'Next: ' +
                        nh.toString().padStart(2, '0') + ':' +
                        nT.getUTCMinutes().toString().padStart(2, '0') + ' ' +
                        nampm;
                }
            }

    const windTag = document.getElementById('wind-tag');
    if (windTag) {
        if (data.wind_ok) {
            windTag.innerText = 'WIND ' + Number(data.wind_speed).toFixed(1) + ' m/s' + (data.wind_stow ? ' - STOW' : '');
            windTag.className = 'tag is-rounded ml-2 has-text-weight-bold ' + (data.wind_stow ? 'is-danger' : 'is-link');
            windTag.title = data.wind_stow ? 'Wind stow active' : 'Anemometer OK';
        } else {
            windTag.innerText = 'WIND --- (offline)';
            windTag.className = 'tag is-rounded ml-2 has-text-weight-bold is-dark';
            windTag.title = 'Anemometer offline';
        }
    }

    const inPitch = document.getElementById('in-pitch');
    const inWidth = document.getElementById('in-width');
    if (inPitch && inWidth && data.pitch_distance !== undefined && data.panel_width !== undefined) {
        // do not overwrite while the user is typing / has unsaved edits
        if (!inPitch.dataset.dirty && document.activeElement !== inPitch) inPitch.value = data.pitch_distance;
        if (!inWidth.dataset.dirty && document.activeElement !== inWidth) inWidth.value = data.panel_width;
    }

    if(document.getElementById('prod-data')) {
      const elRise = document.getElementById('sunrise'); if(elRise) elRise.innerText = formatTime(data.sunrise, data.tz_offset, data.tz_name);
      const elSet = document.getElementById('sunset'); if(elSet) elSet.innerText = formatTime(data.sunset, data.tz_offset, data.tz_name);
      let tH = '<table class="table is-fullwidth is-narrow is-transparent is-size-7">';
      const interval = data.interval_min || 30;
      if(data.table) data.table.forEach((v, i) => { let h = Math.floor(i*interval/60); let m = (i*interval)%60; const ampm = h >= 12 ? 'PM' : 'AM'; let h12 = h % 12 || 12; tH += `<tr><td class="has-text-grey">${h12.toString().padStart(2,'0')}:${m.toString().padStart(2,'0')} ${ampm}</td><td class="has-text-right">${v.toFixed(1)}°</td></tr>`; });
      const elTable = document.getElementById('tilt-table'); if(elTable) elTable.innerHTML = tH + '</table>';
    }
 else {
      const elX = document.getElementById('val-x'); if(elX) elX.innerText = (data.inc ? data.inc.x : (data.sensor_x || 0)).toFixed(2) + '°';
      const elY = document.getElementById('val-y'); if(elY) elY.innerText = (data.inc ? data.inc.y : (data.sensor_y || 0)).toFixed(2) + '°';
      const elZ = document.getElementById('val-z'); if(elZ) elZ.innerText = (data.inc ? data.inc.z : (data.sensor_z || 0)).toFixed(2) + '°';
    }
    
    if (data.clean_angle !== undefined) {
        window.cleanAngle = data.clean_angle;
        window.maintAngle = data.maint_angle;
    }
    
    if (data.max_tilt && (!window.svgMaxTilt || window.svgMaxTilt !== data.max_tilt)) {
        initSVG(data.max_tilt);
    }
    
    if(data.time) { 
        const local = new Date((data.time + (data.tz_offset * 3600)) * 1000); 
        const utc = new Date(data.time * 1000);
        const clockEl = document.getElementById('clock');
        if(clockEl) {
            clockEl.innerText = 
                local.getUTCFullYear() + '-' + (local.getUTCMonth()+1).toString().padStart(2,'0') + '-' + local.getUTCDate().toString().padStart(2,'0') + ' ' + 
                local.getUTCHours().toString().padStart(2,'0') + ':' + local.getUTCMinutes().toString().padStart(2,'0') + ':' + local.getUTCSeconds().toString().padStart(2,'0') + ' ' + (data.tz_name || 'IST'); 
        }
        
        const diagEl = document.getElementById('diag');
        if(diagEl) {
            diagEl.innerText = 'UTC: ' + utc.getUTCFullYear() + '-' + (utc.getUTCMonth()+1).toString().padStart(2,'0') + '-' + utc.getUTCDate().toString().padStart(2,'0') + ' ' + 
                utc.getUTCHours().toString().padStart(2,'0') + ':' + utc.getUTCMinutes().toString().padStart(2,'0') + ':' + utc.getUTCSeconds().toString().padStart(2,'0') + ' | ' + (data.diagnostics || 'OK');
        }
    } else {
        const diagEl = document.getElementById('diag');
        if(diagEl) diagEl.innerText = data.diagnostics || 'Waiting for data...';
    }
    
    document.getElementById('header-progress').style.width = pct + '%';
    const sunAng = Math.atan2(-data.sx, data.sz) * 180 / Math.PI;
    const sEl = document.getElementById('svg-sun'); if(sEl) { sEl.setAttribute('transform', 'rotate(' + (-sunAng) + ')'); sEl.style.opacity = (data.sun_elevation > 0) ? 1 : 0; }
    const cT = current;
    const tT = target;
    document.getElementById('svg-target').setAttribute('transform', 'rotate(' + (-tT) + ')');
    document.getElementById('svg-current').setAttribute('transform', 'rotate(' + (-cT) + ')');
        });
}

function setMode(m) {
    fetch('/set-mode?mode=' + m);
}

function setAngle() {
    fetch('/set-angle?val=' + document.getElementById('target-input').value);
}

function setTargetAngle(v) {
    fetch('/set-angle?val=' + v);
}

function setLoc() {
    const la = document.getElementById('in-lat').value;
    const lo = document.getElementById('in-lon').value;
    fetch(`/set-location?lat=${la}&lon=${lo}`).then(r => r.text()).then(t => alert(t));
}

function setGeometry() {
    const w = document.getElementById('in-width');
    const p = document.getElementById('in-pitch');
    fetch(`/set-geometry?width=${encodeURIComponent(w.value)}&pitch=${encodeURIComponent(p.value)}`)
        .then(r => r.text().then(t => {
            if (r.ok) { w.dataset.dirty = ''; p.dataset.dirty = ''; }
            alert(t);
        }));
}

function manualPress(d, s, e) {
    if (e && e.cancelable) e.preventDefault();
    fetch(`/manual-press?dir=${d}&state=${s}&_=${Date.now()}`);
}


function scanWiFi() {
    document.getElementById('wifi-modal').classList.add('is-active');
    fetch('/wifi-scan').then(r => r.json()).then(data => {
        const list = document.getElementById('ssid-list');
        list.innerHTML = '';
        data.forEach(net => {
            const div = document.createElement('div');
            div.className = 'ssid-item';
            div.innerText = net.ssid;
            div.onclick = () => {
                document.getElementById('wifi-ssid').value = net.ssid;
            };
            list.appendChild(div);
        });
    });
}

function connectWiFi() {
    const s = document.getElementById('wifi-ssid').value;
    const p = document.getElementById('wifi-pass').value;
    fetch(`/wifi-connect?ssid=${s}&pass=${p}`).then(r => r.text()).then(t => alert(t));
}

function forgetWiFi() {
    if (confirm('Forget WiFi?')) fetch('/wifi-logout').then(r => r.text()).then(t => alert(t));
}

function factoryReset() {
    if (confirm('Factory Reset?')) fetch('/factory-reset').then(r => r.text()).then(t => alert(t));
}

function initSVG(maxTilt) {
    const sEl = document.getElementById('svg-scale');
    if (!sEl) return;
    window.svgMaxTilt = maxTilt;
    let sc = '';
    let m = Math.abs(maxTilt);
    for (let a = -m; a <= m; a++) {
        if (a % 15 !== 0 && Math.abs(a) < m - 0.01) continue;
        let isEnd = (Math.abs(a) >= m - 0.01);
        let highlighted = (a === 0 || isEnd);
        let col = highlighted ? '#f8fafc' : '#1e293b';
        sc += `<line x1='84' y1='0' x2='90' y2='0' stroke='${col}' stroke-width='${highlighted ? 2 : 1.5}' transform='rotate(${-a - 90})' />`;
        if (a % 45 === 0 || a === 0 || isEnd) {
            let r = (-a - 90) * Math.PI / 180;
            let tx = Math.round(100 * Math.cos(r));
            let ty = Math.round(100 * Math.sin(r));
            sc += `<text x='${tx}' y='${ty}' fill='${col}' font-size='7' font-weight='${highlighted ? 700 : 400}' text-anchor='middle' alignment-baseline='middle'>${Math.abs(a)}</text>`;
        }
    }
    sEl.innerHTML = sc;
}

window.onload = () => {
    initSVG(60);
    setInterval(fetchStatus, 1000);
};
