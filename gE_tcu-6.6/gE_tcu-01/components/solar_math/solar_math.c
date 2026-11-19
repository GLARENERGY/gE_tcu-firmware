#include "solar_math.h"
#include <math.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "SOLAR_MATH";
static solar_config_t g_config = {
    .latitude = 17.385044,
    .longitude = 78.486671,
    .timezone_offset = 5.5,
    .timezone_name = "IST",
    .location_loaded = false
};

#define NVS_NAMESPACE "solar_cfg"

// Array geometry: config.yaml values are the defaults, NVS (set from the web UI) overrides them.
static float g_panel_width = (float)SOLAR_PANEL_WIDTH;
static float g_pitch_distance = (float)SOLAR_PITCH_DISTANCE;

void solar_math_init(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        nvs_get_u8(nvs_handle, "loc_ok", (uint8_t*)&g_config.location_loaded);
        nvs_get_i64(nvs_handle, "lat", (int64_t*)&g_config.latitude); // NVS stores doubles as int64 bits
        nvs_get_i64(nvs_handle, "lon", (int64_t*)&g_config.longitude);
        nvs_get_i32(nvs_handle, "tz_off", (int32_t*)&g_config.timezone_offset);
        size_t size = sizeof(g_config.timezone_name);
        nvs_get_str(nvs_handle, "tz_name", g_config.timezone_name, &size);
        {   /* array geometry saved from the web UI (stored x1000 as integers) */
            int32_t w_milli = 0, p_milli = 0;
            if (nvs_get_i32(nvs_handle, "wid_x1000", &w_milli) == ESP_OK &&
                nvs_get_i32(nvs_handle, "pit_x1000", &p_milli) == ESP_OK &&
                w_milli > 0 && p_milli >= w_milli) {
                g_panel_width = (float)w_milli / 1000.0f;
                g_pitch_distance = (float)p_milli / 1000.0f;
            }
        }
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Config loaded from NVS: Lat:%.4f Lon:%.4f LocLoaded:%d", 
                 g_config.latitude, g_config.longitude, g_config.location_loaded);
    } else {
        ESP_LOGI(TAG, "No config in NVS, using defaults.");
    }
    ESP_LOGI(TAG, "Array geometry: panel width %.3f m, pitch %.3f m (GCR %.3f)",
             (double)g_panel_width, (double)g_pitch_distance, (double)solar_math_get_gcr());
}

float solar_math_get_panel_width(void) { return g_panel_width; }
float solar_math_get_pitch_distance(void) { return g_pitch_distance; }
float solar_math_get_gcr(void) { return g_panel_width / g_pitch_distance; }

bool solar_math_set_geometry(float panel_width, float pitch_distance) {
    if (!(panel_width >= 0.1f && panel_width <= 20.0f)) return false;
    if (!(pitch_distance >= 0.1f && pitch_distance <= 50.0f)) return false;
    if (pitch_distance < panel_width) return false;      /* GCR > 1 is physically impossible */
    g_panel_width = panel_width;
    g_pitch_distance = pitch_distance;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "wid_x1000", (int32_t)lroundf(panel_width * 1000.0f));
        nvs_set_i32(h, "pit_x1000", (int32_t)lroundf(pitch_distance * 1000.0f));
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Array geometry set: panel width %.3f m, pitch %.3f m (GCR %.3f)",
             (double)g_panel_width, (double)g_pitch_distance, (double)solar_math_get_gcr());
    return true;
}

void solar_math_get_config(solar_config_t *config) {
    if (config) *config = g_config;
}

static void save_config(void) {
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_set_u8(nvs_handle, "loc_ok", g_config.location_loaded ? 1 : 0);
        nvs_set_i64(nvs_handle, "lat", *(int64_t*)&g_config.latitude);
        nvs_set_i64(nvs_handle, "lon", *(int64_t*)&g_config.longitude);
        nvs_set_i32(nvs_handle, "tz_off", *(int32_t*)&g_config.timezone_offset);
        nvs_set_str(nvs_handle, "tz_name", g_config.timezone_name);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Config saved to NVS (loc_ok=%d)", g_config.location_loaded);
    }
}

void solar_math_set_location(double lat, double lon) {
    g_config.latitude = lat;
    g_config.longitude = lon;
    g_config.location_loaded = true;
    save_config();
}

void solar_math_set_timezone(float offset, const char *name) {
    g_config.timezone_offset = offset;
    strncpy(g_config.timezone_name, name, sizeof(g_config.timezone_name) - 1);
    save_config();
}

bool solar_math_is_location_set(void) {
    return g_config.location_loaded;
}

sun_position_t solar_math_calculate_sun_position(time_t t) {
    struct tm dt;
    gmtime_r(&t, &dt);

    struct Time stTime;
    stTime.year = dt.tm_year + 1900;
    stTime.month = dt.tm_mon + 1;
    stTime.day = dt.tm_mday;
    stTime.hour = dt.tm_hour;
    stTime.minute = dt.tm_min;
    stTime.second = (double)dt.tm_sec;

    struct Location stLoc;
    stLoc.longitude = g_config.longitude;
    stLoc.latitude = g_config.latitude;
    stLoc.pressure = 101.325;
    stLoc.temperature = 298.15; // 25C

    struct Position stPos;
    SolTrack(stTime, stLoc, &stPos, 1, 1, 0, 0);

    return (sun_position_t){
        .elevation = (float)stPos.altitudeRefract,
        .azimuth = (float)stPos.azimuthRefract
    };
}

rise_set_data_t solar_math_calculate_rise_set(time_t t) {
    // Local time calculation using standard C libs
    time_t local_t = t + (time_t)(g_config.timezone_offset * 3600);
    struct tm dt;
    gmtime_r(&local_t, &dt);

    struct Time stTime = {
        .year = dt.tm_year + 1900,
        .month = dt.tm_mon + 1,
        .day = dt.tm_mday,
        .hour = 12, .minute = 0, .second = 0.0
    };

    struct Location stLoc = {
        .longitude = g_config.longitude,
        .latitude = g_config.latitude,
        .pressure = 101.325,
        .temperature = 298.15
    };

    struct Position stPos;
    struct RiseSet stRS;
    SolTrack_RiseSet(stTime, stLoc, &stPos, &stRS, 0.0, 1, 1);

    if (stRS.riseTime < 0 || stRS.setTime < 0) return (rise_set_data_t){0, 0, true};

    float riseLocal = (float)stRS.riseTime + g_config.timezone_offset;
    float setLocal = (float)stRS.setTime + g_config.timezone_offset;

    if (riseLocal >= 24.0f) riseLocal -= 24.0f;
    if (riseLocal < 0.0f) riseLocal += 24.0f;
    if (setLocal >= 24.0f) setLocal -= 24.0f;
    if (setLocal < 0.0f) setLocal += 24.0f;

    return (rise_set_data_t){riseLocal, setLocal, false};
}

float solar_math_calculate_tracking_tilt(float elevation, float azimuth) {
    return solar_math_calculate_tracking_tilt_advanced(elevation, azimuth).tracking_tilt;
}

tracking_tilt_result_t solar_math_calculate_tracking_tilt_advanced(float elevation, float azimuth) {
    const double PI_D = 3.14159265358979323846;
    double alpha = (double)elevation * PI_D / 180.0;
    double phi = (double)azimuth * PI_D / 180.0;

    double sx = cos(alpha) * sin(phi);
    double sz = sin(alpha);

    double spx = -sx;
    double spz = sz;

    double theta_T = atan2(spx, spz) * 180.0 / PI_D;

    tracking_tilt_result_t result;
    result.true_tracking_tilt = (float)theta_T;
    result.tracking_tilt = (float)theta_T;
    result.mode = TRACKING_MODE_STANDARD;
    result.correction_active = false;

    if (elevation < 0.0f) {
        result.tracking_tilt = 0.0f;
        result.true_tracking_tilt = 0.0f;
        return result;
    }

    if (elevation < 5.0f) {
        result.tracking_tilt = 0.0f;
        return result;
    }

#if SOLAR_BACKTRACKING_ENABLED
    {
        double theta_T_rad = theta_T * PI_D / 180.0;
        double cos_arg;

        if (fabsf(SOLAR_GROUND_SLOPE) < 0.01f) {
            cos_arg = fabs(cos(theta_T_rad)) / (double)SOLAR_GCR;
        } else {
            double slope_rad = (double)SOLAR_GROUND_SLOPE * PI_D / 180.0;
            double cos_beta = cos(slope_rad);
            if (cos_beta > 0.01) {
                cos_arg = fabs(cos(theta_T_rad - slope_rad)) / ((double)SOLAR_GCR * cos_beta);
            } else {
                cos_arg = 1.0;
            }
        }

        if (cos_arg < 1.0) {
            double sign = (theta_T >= 0.0) ? 1.0 : -1.0;
            double theta_c = -sign * acos(fmin(cos_arg, 1.0)) * 180.0 / PI_D;
            result.tracking_tilt = (float)(theta_T + theta_c);
            result.mode = (fabsf(SOLAR_GROUND_SLOPE) < 0.01f)
                ? TRACKING_MODE_FLAT_BACKTRACKING
                : TRACKING_MODE_SLOPE_BACKTRACKING;
            result.correction_active = true;
        }
    }
#endif

    return result;
}

const char* solar_math_tracking_mode_to_str(tracking_mode_t mode) {
    switch (mode) {
        case TRACKING_MODE_STANDARD:           return "Standard";
        case TRACKING_MODE_FLAT_BACKTRACKING:  return "Flat Backtracking";
        case TRACKING_MODE_SLOPE_BACKTRACKING: return "Slope Backtracking";
        default:                               return "Unknown";
    }
}
