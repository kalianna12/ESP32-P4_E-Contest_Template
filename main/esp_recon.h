#pragma once

#include "ui/test_screen.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ESP_RECON_SAMPLE_COUNT
#define ESP_RECON_SAMPLE_COUNT ADV_CAPTURE_SAMPLE_MAX
#endif

#ifndef ESP_RECON_OUTPUT_SAMPLE_COUNT
#define ESP_RECON_OUTPUT_SAMPLE_COUNT 4096U
#endif

// 0: time-domain de-mean + normalize only.
// 1: DFT/IFFT, magnitude compensation only; keep captured phase. Recommended first.
// 2: DFT/IFFT, magnitude + phase compensation.
#ifndef ESP_RECON_STAGE
#define ESP_RECON_STAGE 2
#endif

// 0: legacy full-spectrum IFFT output.
// 1: synthesize output from detected harmonics only. Recommended for contest demo.
#ifndef ESP_RECON_OUTPUT_USE_HARMONIC_SYNTH
#define ESP_RECON_OUTPUT_USE_HARMONIC_SYNTH 1
#endif

// When harmonic synth is enabled, keep only odd harmonics. Formal unknown
// periodic waves should allow all integer harmonics by default.
#ifndef ESP_RECON_SYNTH_ODD_HARMONICS_ONLY
#define ESP_RECON_SYNTH_ODD_HARMONICS_ONLY 0
#endif

// Optional circular 3-tap output smoothing after harmonic synthesis / IFFT fallback.
#ifndef ESP_RECON_OUTPUT_SMOOTH_ENABLE
#define ESP_RECON_OUTPUT_SMOOTH_ENABLE 0
#endif

// ============================================================
// Phase / waveform-shape debug knobs
// ============================================================
// Use these three macros to locate phase reference or sin/cos convention errors
// when reconstructed peaks/valleys do not line up with the original waveform.
//
// Suggested quick tests:
//   1) ESP_RECON_PHASE_COMP_SIGN = -1
//   2) ESP_RECON_PHASE_REFERENCE_DEG_X10 = 900 or -900
//   3) ESP_RECON_SYNTH_USE_SIN_BASIS = 1
//
// Reconstructed harmonic phase direction:
//   +1 keeps the current DFT phase convention
//   -1 mirrors the captured harmonic phase
//    0 forces zero phase for all harmonics
// When ESP_RECON_STAGE >= 2, measured circuit phase compensation uses the same
// sign so captured phase and model phase flip together.
#ifndef ESP_RECON_PHASE_COMP_SIGN
#define ESP_RECON_PHASE_COMP_SIGN 1
#endif

// Fundamental-period reference shift used only by harmonic synthesis.
// Unit: 0.1 degree of the fundamental. Example: 900 = +90.0 deg.
#ifndef ESP_RECON_PHASE_REFERENCE_DEG_X10
#define ESP_RECON_PHASE_REFERENCE_DEG_X10 0
#endif

// Harmonic synthesis basis: 0 = cos(theta + phase), 1 = sin(theta + phase).
#ifndef ESP_RECON_SYNTH_USE_SIN_BASIS
#define ESP_RECON_SYNTH_USE_SIN_BASIS 0
#endif

#ifndef ESP_RECON_OUTPUT_INVERT
#define ESP_RECON_OUTPUT_INVERT 1
#endif

#if ESP_RECON_PHASE_COMP_SIGN < -1 || ESP_RECON_PHASE_COMP_SIGN > 1
#error "ESP_RECON_PHASE_COMP_SIGN must be -1, 0, or 1"
#endif

#if ESP_RECON_SYNTH_USE_SIN_BASIS != 0 && ESP_RECON_SYNTH_USE_SIN_BASIS != 1
#error "ESP_RECON_SYNTH_USE_SIN_BASIS must be 0 or 1"
#endif

#ifndef ESP_RECON_TARGET_PEAK
#define ESP_RECON_TARGET_PEAK 8000
#endif

#ifndef ESP_RECON_HARMONIC_MAX
#define ESP_RECON_HARMONIC_MAX 100U
#endif

typedef enum {
    ESP_RECON_MODE_AUTO = 0,
    ESP_RECON_MODE_SQUARE = 1,
    ESP_RECON_MODE_ARB = 2,
    ESP_RECON_MODE_TRI_SINE = 3,
} esp_recon_mode_t;

typedef enum {
    ESP_RECON_SHAPE_UNKNOWN = 0,
    ESP_RECON_SHAPE_SINE = 1,
    ESP_RECON_SHAPE_TRIANGLE = 2,
    ESP_RECON_SHAPE_SQUARE = 3,
    ESP_RECON_SHAPE_ARB = 4,
} esp_recon_detected_shape_t;

enum {
    ESP_RECON_QUALITY_WARN_SPIKES = 0x00000001U,
    ESP_RECON_QUALITY_WARN_AMDF = 0x00000002U,
    ESP_RECON_QUALITY_UNSTABLE = 0x00010000U,
};

typedef struct {
    uint32_t spike_count;
    uint32_t spike_replaced_count;
    uint32_t max_consecutive_spikes;
    uint32_t amdf_confidence_x1000;
    float amdf_score;
    uint32_t flags;
} esp_recon_quality_t;

typedef struct {
    uint32_t index;
    uint32_t bin;
    uint32_t freq_hz;
    int32_t amp_mv;
    int32_t phase_deg_x10;
    uint32_t flags;
} esp_recon_harmonic_t;

typedef struct {
    int32_t phase_sign;
    int32_t reference_deg_x10;
    int32_t sin_basis;
} esp_recon_phase_debug_t;

typedef struct {
    int16_t samples[ESP_RECON_OUTPUT_SAMPLE_COUNT];
    uint32_t sample_count;
    uint32_t sample_rate_hz;
    uint32_t capture_sample_rate_hz;
    uint32_t playback_sample_rate_hz;
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
    esp_recon_mode_t mode;
    esp_recon_detected_shape_t detected_shape;
    esp_recon_quality_t quality;
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
void EspRecon_SetMode(esp_recon_mode_t mode);
esp_recon_mode_t EspRecon_GetMode(void);
bool EspRecon_SetPlaybackRateHz(uint32_t sample_rate_hz);
uint32_t EspRecon_GetPlaybackRateHz(void);
void EspRecon_SetPhaseDebug(const esp_recon_phase_debug_t *config);
void EspRecon_GetPhaseDebug(esp_recon_phase_debug_t *config);
void EspRecon_CyclePhaseDebug(void);

#ifdef __cplusplus
}
#endif
