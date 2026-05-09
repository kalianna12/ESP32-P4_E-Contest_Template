#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t packets;
    uint32_t frame_errors;
    uint32_t timeouts;
    uint8_t state;
    uint8_t mode;
    uint8_t source;
    uint8_t monitor_ok;
    uint32_t progress_permille;
    uint32_t elapsed_ms;

    uint32_t sample_index;
    uint32_t total_samples;

    uint32_t dut_adc_code;
    uint32_t dut_adc_bits;
    uint32_t dut_adc_avg_x1000;
    uint32_t dut_conversion_time_ns;

    uint32_t input_mv;
    uint32_t stm32_adc_raw12;
    uint32_t stm32_adc_mv;

    int32_t offset_error_lsb_x1000;
    int32_t gain_error_lsb_x1000;
    int32_t gain_error_ppm;
    int32_t dnl_min_x1000;
    int32_t dnl_max_x1000;
    int32_t inl_min_x1000;
    int32_t inl_max_x1000;
    uint32_t missing_codes;

    int32_t snr_db_x100;
    int32_t sinad_db_x100;
    int32_t enob_x100;
    int32_t sfdr_db_x100;
    int32_t thd_db_x100;
} adc_ui_status_t;

void test_screen_create(void);
void test_screen_update_measurement(const adc_ui_status_t *status);

#ifdef __cplusplus
}
#endif
