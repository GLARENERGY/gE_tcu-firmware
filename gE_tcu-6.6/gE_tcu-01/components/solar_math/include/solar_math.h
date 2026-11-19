#ifndef SOLAR_MATH_H
#define SOLAR_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "SolTrack.h"
#include "AppConfig.h"

// --- Constants (Site Specific) ---
// Defaults for the array geometry come from config.yaml (SOLAR_PANEL_WIDTH / SOLAR_PITCH_DISTANCE).
// The values actually used at run time can be changed from the web UI (stored in NVS) - see solar_math_set_geometry().
#ifndef SOLAR_PANEL_WIDTH
    #define SOLAR_PANEL_WIDTH 3.0f
#endif
#ifndef SOLAR_PITCH_DISTANCE
    #define SOLAR_PITCH_DISTANCE 4.0f
#endif
#define SOLAR_GCR (solar_math_get_gcr())   /* ground coverage ratio = width / pitch (run-time value) */

#if defined(MAX_TILT_LIMIT) && (MAX_TILT_LIMIT <= 60)
    #define SOLAR_MAX_TILT ((float)MAX_TILT_LIMIT)
#else
    #define SOLAR_MAX_TILT 45.0f
#endif

#ifndef CLEAN_ANGLE
    #define SOLAR_CLEAN_ANGLE 50.0f
#else
    #define SOLAR_CLEAN_ANGLE ((float)CLEAN_ANGLE)
#endif

#define SOLAR_STOW_ANGLE 0.0f
#define SOLAR_PARK_ANGLE 0.0f
#define SOLAR_PARK_ELEVATION 5.0f
#define SOLAR_STOW_WIND_THRESHOLD 160.0f

#ifdef GROUND_SLOPE
    #define SOLAR_GROUND_SLOPE ((float)GROUND_SLOPE)
#else
    #define SOLAR_GROUND_SLOPE 0.0f
#endif

#ifdef BACKTRACKING_ENABLED
    #define SOLAR_BACKTRACKING_ENABLED 1
#else
    #define SOLAR_BACKTRACKING_ENABLED 0
#endif

typedef enum {
    TRACKING_MODE_STANDARD,
    TRACKING_MODE_FLAT_BACKTRACKING,
    TRACKING_MODE_SLOPE_BACKTRACKING
} tracking_mode_t;

typedef struct {
    float tracking_tilt;       // final (possibly backtracked) tilt in degrees
    float true_tracking_tilt;  // pure geometric tilt (theta_T) in degrees
    tracking_mode_t mode;      // which mode produced this result
    bool correction_active;    // true when backtracking correction was applied
} tracking_tilt_result_t;

typedef struct {
    float elevation;
    float azimuth;
} sun_position_t;

typedef struct {
    float sunrise; // Decimal hours local time
    float sunset;
    bool error;
} rise_set_data_t;

typedef struct {
    double latitude;
    double longitude;
    float timezone_offset;
    char timezone_name[16];
    bool location_loaded;
} solar_config_t;

// --- API ---
void solar_math_init(void);
void solar_math_get_config(solar_config_t *config);
void solar_math_set_location(double lat, double lon);
void solar_math_set_timezone(float offset, const char *name);
bool solar_math_is_location_set(void);

// Array geometry (metres; only the ratio width/pitch affects the tracking angle)
float solar_math_get_panel_width(void);
float solar_math_get_pitch_distance(void);
float solar_math_get_gcr(void);
// Validates (0.1..20 m width, 0.1..50 m pitch, pitch >= width), applies and saves to NVS. Returns false if rejected.
bool  solar_math_set_geometry(float panel_width, float pitch_distance);

sun_position_t solar_math_calculate_sun_position(time_t t);
rise_set_data_t solar_math_calculate_rise_set(time_t t);
float solar_math_calculate_tracking_tilt(float elevation, float azimuth);
tracking_tilt_result_t solar_math_calculate_tracking_tilt_advanced(float elevation, float azimuth);
const char* solar_math_tracking_mode_to_str(tracking_mode_t mode);

#endif // SOLAR_MATH_H
