#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_POINTS 1200
#define MODEL_POINTS_MAX 128
#define ADC_TEST_SAMPLE_MAX 256
#define ADV_HARMONIC_MAX 50

#define FREQ_POINT_FLAG_UNSTABLE 0x00000800U
#define FREQ_POINT_FLAG_MISSING  0x80000000U

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
    MODEL_TYPE_UNKNOWN = 0,
    MODEL_TYPE_LP1 = 1,
    MODEL_TYPE_HP1 = 2,
    MODEL_TYPE_LP2 = 3,
    MODEL_TYPE_HP2 = 4,
    MODEL_TYPE_BP2 = 5,
    MODEL_TYPE_BS2 = 6,
} circuit_model_kind_t;

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
    CMD_ADC_TEST_START = 30,
    CMD_ADC_TEST_STOP = 31,
} freqresp_command_t;

typedef struct {
    uint32_t point_index;
    uint32_t total_points;
    uint32_t freq_hz;
    int32_t vin_mv;
    int32_t vout_mv;
    int32_t gain_x1000;
    int32_t theory_gain_x1000;
    int32_t error_x10;
    int32_t phase_deg_x10;
    uint32_t flags;
    bool phase_valid;
} freq_point_t;

typedef struct {
    uint32_t freq_hz;
    int32_t gain_x1000;
    int32_t phase_deg_x10;
} model_point_t;

typedef struct {
    bool valid;
    uint8_t model_type;

    uint32_t fc_hz;
    uint32_t f0_hz;
    uint32_t fl_hz;
    uint32_t fh_hz;

    int32_t q_x1000;
    int32_t k_x1000;

    int32_t rms_error_x10;
    int32_t max_error_x10;
    int32_t confidence_x1000;
    uint32_t valid_point_count;
} filter_fit_result_t;

typedef struct {
    bool valid;
    uint8_t filter_type;
    uint32_t cutoff_freq_hz;
    uint32_t point_count;
    model_point_t points[MODEL_POINTS_MAX];
    filter_fit_result_t fit;

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
    uint32_t flags;

    uint32_t cutoff_freq_hz;
    bool has_measurement;
    bool phase_valid;
} freqresp_ui_status_t;

typedef struct {
    uint32_t seq;
    uint32_t wave_type;
    uint32_t chunk_index;
    uint32_t chunk_count;
    uint32_t sample_rate_hz;
    uint32_t dds_freq_hz;
    uint32_t total_sample_count;
    uint32_t start_sample_index;
    int32_t min_mv;
    int32_t max_mv;
    int32_t mean_mv;
    int32_t vpp_mv;
    uint32_t flags;
    int16_t samples[30];
} adc_waveform_chunk_t;

typedef struct {
    uint32_t seq;
    uint32_t sample_rate_hz;
    uint32_t dds_freq_hz;
    uint32_t capture_sample_count;
    uint32_t display_sample_count;
    uint32_t measured_freq_hz;
    int32_t raw_min;
    int32_t raw_max;
    int32_t raw_mean;
    int32_t raw_vpp;
    int32_t raw_rms;
    int32_t amp_peak_raw;
    int32_t amp_rms_raw;
    int32_t phase_deg_x10;
    uint32_t flags;
    uint32_t display_decimation;
    uint32_t zero_cross_count;
} adc_analysis_result_t;

typedef struct {
    uint32_t seq;
    uint32_t adv_state;
    uint32_t error_code;
    uint32_t flags;
    uint32_t sample_rate_hz;
    uint32_t total_sample_count;
    uint32_t capture_done_count;
    uint32_t recon_done_count;
    uint32_t fft_overflow_count;
    uint32_t ifft_overflow_count;
    uint32_t tlast_missing_count;
    uint32_t tlast_unexpected_count;
    uint32_t recon_count_base;
} adv_status_t;

typedef struct {
    uint32_t seq;
    uint32_t index;
    uint32_t freq_hz;
    int32_t amp_mv;
    int32_t phase_deg_x10;
    uint32_t flags;
} adv_harmonic_t;

void test_screen_create(void);
void test_screen_update_measurement(const freqresp_ui_status_t *status);
void test_screen_update_adc_analysis_result(const adc_analysis_result_t *result);
void test_screen_update_adc_waveform_chunk(const adc_waveform_chunk_t *chunk);
void test_screen_update_adv_status(const adv_status_t *status);
void test_screen_update_adv_harmonic(const adv_harmonic_t *harmonic);
void test_screen_update_spi_text_test(const char *rx_text, uint8_t link_state);

#ifdef __cplusplus
}
#endif
