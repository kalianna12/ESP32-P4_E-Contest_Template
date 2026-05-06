#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_index;
    uint32_t total_samples;
    uint32_t input_mv;
    uint32_t adc_code;
    uint32_t adc_bits;
    uint32_t progress_permille;

    int32_t offset_error_uv;
    int32_t gain_error_ppm;
    int32_t inl_lsb_x1000;
    int32_t dnl_lsb_x1000;

    uint32_t missing_codes;
    uint32_t conversion_time_ns;

    uint32_t packets;
    uint32_t frame_errors;
    uint32_t timeouts;
} adc_ui_status_t;

void test_screen_create(void);
void test_screen_update_measurement(const adc_ui_status_t *status);

#ifdef __cplusplus
}
#endif