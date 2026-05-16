#pragma once

#include "ui/test_screen.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ESP_RECON_SAMPLE_COUNT
#define ESP_RECON_SAMPLE_COUNT 1024U
#endif

#ifndef ESP_RECON_OUTPUT_SAMPLE_COUNT
#define ESP_RECON_OUTPUT_SAMPLE_COUNT 4096U
#endif

// 0: time-domain de-mean + normalize only.
// 1: DFT/IFFT, magnitude compensation only; keep captured phase. Recommended first.
// 2: DFT/IFFT, magnitude + phase compensation.
#ifndef ESP_RECON_STAGE
#define ESP_RECON_STAGE 1
#endif

#ifndef ESP_RECON_TARGET_PEAK
#define ESP_RECON_TARGET_PEAK 8000
#endif

#ifndef ESP_RECON_HARMONIC_MAX
#define ESP_RECON_HARMONIC_MAX 20U
#endif

typedef struct {
    uint32_t index;
    uint32_t bin;
    uint32_t freq_hz;
    int32_t amp_mv;
    int32_t phase_deg_x10;
    uint32_t flags;
} esp_recon_harmonic_t;

typedef struct {
    int16_t samples[ESP_RECON_OUTPUT_SAMPLE_COUNT];
    uint32_t sample_count;
    uint32_t sample_rate_hz;
    int32_t cap_min;
    int32_t cap_max;
    int32_t cap_mean;
    int32_t cap_vpp;
    int32_t out_min;
    int32_t out_max;
    int32_t out_vpp;
    uint32_t dominant_bin;
    uint32_t dominant_freq_hz;
    uint32_t harmonic_count;
    esp_recon_harmonic_t harmonics[ESP_RECON_HARMONIC_MAX];
} esp_recon_result_t;

bool EspRecon_BuildFromCapture(const int16_t *capture,
                               uint32_t sample_count,
                               uint32_t sample_rate_hz,
                               const circuit_model_t *model,
                               esp_recon_result_t *out);

bool EspRecon_SendFromCapture(const int16_t *capture,
                              uint32_t sample_count,
                              uint32_t sample_rate_hz,
                              const circuit_model_t *model);

uint32_t EspRecon_GetLastHarmonics(esp_recon_harmonic_t *out, uint32_t max_count);

#ifdef __cplusplus
}
#endif
