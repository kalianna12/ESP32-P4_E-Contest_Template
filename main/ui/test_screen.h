#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_POINTS 512
#define MODEL_POINTS_MAX 128

typedef enum {
    FREQRESP_STATE_IDLE = 0,
    FREQRESP_STATE_SCANNING = 1,
    FREQRESP_STATE_DONE = 2,
    FREQRESP_STATE_ERROR = 3,
} freqresp_state_t;

typedef enum {
    MODE_SWEEP = 0,
    MODE_SINGLE = 1,
} freqresp_mode_t;

typedef enum {
    FILTER_TYPE_UNKNOWN = 0,
    FILTER_TYPE_LOW_PASS = 1,
    FILTER_TYPE_HIGH_PASS = 2,
    FILTER_TYPE_BAND_PASS = 3,
    FILTER_TYPE_BAND_STOP = 4,
} freqresp_filter_type_t;

typedef enum {
    CMD_START = 1,
    CMD_STOP = 2,
    CMD_SET_MODE = 3,
    CMD_SET_START_FREQ = 4,
    CMD_SET_STOP_FREQ = 5,
    CMD_SET_STEP_FREQ = 6,
    CMD_SET_SINGLE_FREQ = 7,
    CMD_CLEAR_TABLE = 8,
    CMD_ADV_CAPTURE = 20,
    CMD_ADV_RECONSTRUCT = 21,
    CMD_ADV_SEND_TO_DDS = 22,
} freqresp_command_t;

typedef struct {
    uint32_t freq_hz;
    int32_t vin_mv;
    int32_t vout_mv;
    int32_t gain_x1000;
    int32_t theory_gain_x1000;
    int32_t error_x10;
    int32_t phase_deg_x10;
} freq_point_t;

typedef struct {
    uint32_t freq_hz;
    int32_t gain_x1000;
    int32_t phase_deg_x10;
} model_point_t;

typedef struct {
    bool valid;
    uint8_t filter_type;
    uint32_t cutoff_freq_hz;
    uint32_t point_count;
    model_point_t points[MODEL_POINTS_MAX];

    uint32_t start_freq_hz;
    uint32_t stop_freq_hz;
    uint32_t step_freq_hz;

    uint32_t saved_time_ms;
} circuit_model_t;

typedef struct {
    uint32_t packets;
    uint32_t frame_errors;
    uint32_t timeouts;

    uint8_t link_ok;
    uint8_t state;
    uint8_t mode;
    uint8_t filter_type;

    uint32_t progress_permille;

    uint32_t start_freq_hz;
    uint32_t stop_freq_hz;
    uint32_t step_freq_hz;
    uint32_t single_freq_hz;

    uint32_t current_freq_hz;
    uint32_t point_index;
    uint32_t total_points;

    int32_t vin_mv;
    int32_t vout_mv;
    int32_t gain_x1000;
    int32_t theory_gain_x1000;
    int32_t error_x10;
    int32_t phase_deg_x10;

    uint32_t cutoff_freq_hz;
} freqresp_ui_status_t;

void test_screen_create(void);
void test_screen_update_measurement(const freqresp_ui_status_t *status);
void test_screen_update_spi_text_test(const char *rx_text, uint8_t link_state);

#ifdef __cplusplus
}
#endif
