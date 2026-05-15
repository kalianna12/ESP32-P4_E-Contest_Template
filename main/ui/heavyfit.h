#pragma once

#include "test_screen.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HEAVYFIT_RESULT_NONE = 0,
    HEAVYFIT_RESULT_LIGHT,
    HEAVYFIT_RESULT_HEAVY_TIMEOUT,
    HEAVYFIT_RESULT_HEAVY_LOW_CONF,
    HEAVYFIT_RESULT_HEAVY_OK,
} heavyfit_result_quality_t;

typedef struct {
    freqresp_ui_status_t status;
    uint32_t point_count;
    freq_point_t points[MAX_POINTS];
} heavyfit_input_t;

typedef struct {
    bool valid;
    bool canceled;
    uint8_t quality;
    freqresp_ui_status_t status;
    filter_fit_result_t fit;
    uint32_t point_count;
    int32_t theory_gain_x1000[MAX_POINTS];
    int32_t error_x10[MAX_POINTS];
} heavyfit_output_t;

bool HeavyFit_StartAsync(const heavyfit_input_t *input);
bool HeavyFit_PollResult(heavyfit_output_t *out);
bool HeavyFit_IsBusy(void);
void HeavyFit_Cancel(void);

#ifdef __cplusplus
}
#endif
