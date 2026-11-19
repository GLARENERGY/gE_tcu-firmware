#pragma once
/*
 * Minimal Modbus TCP slave (server) for SCADA polling.
 *
 * Hand-rolled over a raw TCP socket (not the esp-modbus managed component) so it has no
 * extra dependency - it implements just enough of the protocol (MBAP header, function
 * codes 0x03 Read Holding Registers and 0x06 Write Single Register) for a SCADA master
 * to poll the register map below. One client at a time; good enough to prove the channel
 * works. Swap in a full Modbus stack (e.g. espressif/esp-modbus) if you need more (FC16,
 * multiple simultaneous masters, coils, etc).
 *
 * Register map (all 16-bit, Modbus holding registers, big-endian on the wire):
 *   HR0  mode           0 = auto, 1 = manual   (read/write - FC6 calls the mode callback)
 *   HR1  tracker state  tracker_state_t value (0=OFF,1=TRACKING,2=MANUAL,3=NIGHT_RESET,4=STOW,5=STOP)
 *   HR2  current tilt   degrees x10, signed (e.g. -453 = -45.3 deg)
 *   HR3  target tilt    degrees x10, signed
 *   HR4  status bits    bit0 motor_fault | bit1 inclinometer invalid | bit2 RTC not trusted | bit3 eth link up
 *                       bit4 anemometer offline
 *   HR5  register map version (currently 2) - bump if you change this table
 *   HR6  wind speed     m/s x10, unsigned (0 when the anemometer is offline, see HR4 bit5)
 */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MODBUS_REG_COUNT 7

typedef void (*modbus_tcp_mode_write_cb_t)(bool manual);

/* Starts the listening task (port 502). Call once, after the network (Wi-Fi or Ethernet)
 * is up. `cb` is called when a SCADA master writes HR0 (FC6) - may be NULL to make it
 * read-only. NOTE: like the existing web handlers, this has no authentication; anyone who
 * can reach port 502 can write the mode register. Fine for a bench/lab SCADA link, not
 * for an untrusted network. */
esp_err_t modbus_tcp_start(modbus_tcp_mode_write_cb_t cb);

/* Call periodically (e.g. every couple of seconds from tracker_task) to refresh the
 * register snapshot that the server hands out. Thread-safe. */
void modbus_tcp_update_regs(bool manual_mode, float current_tilt_deg, float target_tilt_deg,
                            int tracker_state, bool motor_fault, bool inclinometer_invalid,
                            bool rtc_untrusted, float wind_speed_ms, bool anemometer_offline);
