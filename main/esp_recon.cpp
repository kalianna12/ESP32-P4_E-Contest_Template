#include "esp_recon.h"

#include "dds_direct_link.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

namespace {

constexpr char TAG[] = "EspRecon";
constexpr uint32_t kN = ESP_RECON_SAMPLE_COUNT;
constexpr uint32_t kOutN = ESP_RECON_OUTPUT_SAMPLE_COUNT;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kMinGain = 0.05f;
constexpr float kMaxInvGain = 5.0f;
constexpr float kMinHarmonicGain = 0.15f;
constexpr float kHarmonicRelativeFloor = 0.05f;
constexpr float kHarmonicAbsoluteFloor = 20.0f;
constexpr uint32_t kSquareMaxKeptHarmonics = 80U;
constexpr uint32_t kSquareOddMaxHarmonic = 99U;
constexpr uint32_t kSquareMaxHarmonicFreqHz = 25000U;
constexpr float kSquareRelativeFloor = 0.010f;
constexpr float kSquareAbsoluteFloorMv = 5.0f;
constexpr float kSquareMinHarmonicGain = 0.08f;
constexpr uint32_t kF0MinHz = 100U;
constexpr uint32_t kF0MaxHz = 20000U;
constexpr uint32_t kAmdfLagMinFloor = 8U;
constexpr uint32_t kAmdfLagMaxCeil = 2000U;
constexpr uint32_t kAmdfClosePercent = 115U;
constexpr uint32_t kMinLockedCycles = 3U;
constexpr uint32_t kMaxLockedHarmonics = 100U;
constexpr uint32_t kYieldEveryBins = 16U;
constexpr uint32_t kReconOutputGainDefaultX1000 = 1000U;

#ifndef ESP_RECON_HARMONIC_SEARCH_SPAN
#define ESP_RECON_HARMONIC_SEARCH_SPAN 3U
#endif

#ifndef ESP_RECON_PERIODIC_SYNTH_ENABLE
#define ESP_RECON_PERIODIC_SYNTH_ENABLE 1
#endif

static esp_recon_harmonic_t g_last_harmonics[ESP_RECON_HARMONIC_MAX] = {};
static uint32_t g_last_harmonic_count = 0;
static esp_recon_mode_t g_recon_mode = ESP_RECON_MODE_AUTO;
static uint32_t g_recon_dds_playback_rate_hz = DDS_DIRECT_DEFAULT_PLAYBACK_RATE_HZ;
static uint32_t g_recon_output_gain_x1000 = kReconOutputGainDefaultX1000;
static esp_recon_phase_debug_t g_phase_debug = {
    ESP_RECON_PHASE_COMP_SIGN,
    ESP_RECON_PHASE_REFERENCE_DEG_X10,
    ESP_RECON_SYNTH_USE_SIN_BASIS,
};

constexpr esp_recon_phase_debug_t kPhaseDebugPresets[] = {
    { 1, 0, 0 },
    { -1, 0, 0 },
    { 0, 0, 0 },
    { 1, 900, 0 },
    { 1, -900, 0 },
    { 1, 0, 1 },
};
static uint32_t g_phase_debug_preset = 0;

struct Work {
    float xr[kN];
    float xi[kN];
    float tr[kN];
    float synth[kOutN];
    int16_t wave[kOutN];
    int16_t clean_capture[kN];
};

typedef struct {
    uint32_t spike_count;
    uint32_t spike_replaced_count;
    uint32_t max_consecutive_spikes;
    int32_t rms;
} clean_capture_stats_t;

static Work *AllocWork()
{
    Work *w = static_cast<Work *>(heap_caps_calloc(1, sizeof(Work), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (w == nullptr) {
        w = static_cast<Work *>(heap_caps_calloc(1, sizeof(Work), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    return w;
}

static int16_t Sat16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}

static float WrapPhaseRad(float phase)
{
    while (phase > kPi) {
        phase -= kTwoPi;
    }
    while (phase < -kPi) {
        phase += kTwoPi;
    }
    return phase;
}

static int32_t ClampPhaseSign(int32_t sign)
{
    if (sign > 0) {
        return 1;
    }
    if (sign < 0) {
        return -1;
    }
    return 0;
}

static int32_t ClampSinBasis(int32_t sin_basis)
{
    return sin_basis != 0 ? 1 : 0;
}

static float PhaseSignFloat(void)
{
    return static_cast<float>(ClampPhaseSign(g_phase_debug.phase_sign));
}

static esp_recon_mode_t EffectiveReconMode(esp_recon_mode_t mode,
                                           esp_recon_detected_shape_t detected_shape)
{
    if (mode != ESP_RECON_MODE_AUTO) {
        return mode;
    }

    if (detected_shape == ESP_RECON_SHAPE_SQUARE ||
        detected_shape == ESP_RECON_SHAPE_TRIANGLE) {
        return ESP_RECON_MODE_SQUARE;
    }
    if (detected_shape == ESP_RECON_SHAPE_ARB) {
        return ESP_RECON_MODE_ARB;
    }
    return ESP_RECON_MODE_TRI_SINE;
}

static bool IsSquareReconMode(esp_recon_mode_t mode,
                              esp_recon_detected_shape_t detected_shape)
{
    return EffectiveReconMode(mode, detected_shape) == ESP_RECON_MODE_SQUARE;
}

static uint32_t SquareOddHarmonicLimit(uint32_t period_samples)
{
    if (period_samples >= 128U) {
        return kSquareOddMaxHarmonic;
    }
    if (period_samples >= 64U) {
        return 41U;
    }
    if (period_samples >= 32U) {
        return 21U;
    }
    if (period_samples >= 12U) {
        return 9U;
    }
    return 5U;
}

static float SynthReferencePhaseRad(void)
{
    return static_cast<float>(g_phase_debug.reference_deg_x10) * (kPi / 1800.0f);
}

static uint32_t ClampU32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void CaptureStats(const int16_t *capture, uint32_t n, esp_recon_result_t *out)
{
    int32_t min_v = capture[0];
    int32_t max_v = capture[0];
    int64_t sum = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const int32_t v = capture[i];
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
        sum += v;
    }
    out->cap_min = min_v;
    out->cap_max = max_v;
    out->cap_mean = static_cast<int32_t>(sum / static_cast<int64_t>(n));
    out->cap_vpp = max_v - min_v;
}

static void NormalizeToInt16(const float *src, uint32_t n, int32_t target_peak, int16_t *dst,
                             int32_t *out_min, int32_t *out_max)
{
    float mean = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        mean += src[i];
    }
    mean /= static_cast<float>(n);

    float peak = 1.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const float a = fabsf(src[i] - mean);
        if (a > peak) {
            peak = a;
        }
    }
    const float scale = static_cast<float>(target_peak) / peak;

    int32_t mn = 32767;
    int32_t mx = -32768;
    for (uint32_t i = 0; i < n; ++i) {
        const int32_t q = static_cast<int32_t>(lrintf((src[i] - mean) * scale));
        dst[i] = Sat16(q);
        if (dst[i] < mn) {
            mn = dst[i];
        }
        if (dst[i] > mx) {
            mx = dst[i];
        }
    }
    *out_min = mn;
    *out_max = mx;
}

static int32_t MeanOfWindow(const int16_t *capture, uint32_t n)
{
    int64_t sum = 0;
    for (uint32_t i = 0; i < n; ++i) {
        sum += capture[i];
    }
    return static_cast<int32_t>(sum / static_cast<int64_t>(n));
}

static int16_t Median3(int16_t a, int16_t b, int16_t c)
{
    if (a > b) {
        const int16_t t = a;
        a = b;
        b = t;
    }
    if (b > c) {
        const int16_t t = b;
        b = c;
        c = t;
    }
    if (a > b) {
        const int16_t t = a;
        a = b;
        b = t;
    }
    return b;
}

static int32_t EstimateRms(const int16_t *capture, uint32_t n, int32_t mean)
{
    if (capture == nullptr || n == 0U) {
        return 0;
    }

    uint64_t sum_sq = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const int32_t d = static_cast<int32_t>(capture[i]) - mean;
        sum_sq += static_cast<uint64_t>(d * d);
    }
    return static_cast<int32_t>(sqrtf(static_cast<float>(sum_sq) / static_cast<float>(n)));
}

static void BuildCleanCaptureAndStats(const int16_t *capture,
                                      uint32_t n,
                                      int32_t mean,
                                      int16_t *clean,
                                      clean_capture_stats_t *stats)
{
    if (stats != nullptr) {
        *stats = {};
    }
    if (capture == nullptr || clean == nullptr || n == 0U) {
        return;
    }

    memcpy(clean, capture, n * sizeof(clean[0]));
    if (n < 3U) {
        return;
    }

    const int32_t rms = EstimateRms(capture, n, mean);
    const int32_t spike_threshold = rms * 4;
    const int32_t hi = mean + spike_threshold;
    const int32_t lo = mean - spike_threshold;
    uint32_t consecutive_spikes = 0U;

    if (stats != nullptr) {
        stats->rms = rms;
    }

    for (uint32_t i = 1; i + 1U < n; ++i) {
        const int16_t median = Median3(clean[i - 1U], clean[i], clean[i + 1U]);
        const int32_t raw = static_cast<int32_t>(capture[i]);
        const int32_t diff = raw - static_cast<int32_t>(median);
        const int32_t abs_diff = (diff < 0) ? -diff : diff;
        const bool isolated_spike = (abs_diff > spike_threshold) || raw > hi || raw < lo;
        if (isolated_spike) {
            clean[i] = static_cast<int16_t>(
                (static_cast<int32_t>(clean[i - 1U]) + static_cast<int32_t>(clean[i + 1U])) / 2);
            ++consecutive_spikes;
            if (stats != nullptr) {
                ++stats->spike_count;
                ++stats->spike_replaced_count;
                if (consecutive_spikes > stats->max_consecutive_spikes) {
                    stats->max_consecutive_spikes = consecutive_spikes;
                }
            }
        } else {
            consecutive_spikes = 0U;
        }
    }
}

static uint32_t EstimateAmdfConfidenceX1000(float score, int32_t rms, uint32_t period_samples)
{
    if (score <= 0.0f || rms <= 0 || period_samples == 0U) {
        return 0U;
    }

    const float norm = score / static_cast<float>(rms);
    float confidence = 1400.0f - norm * 180.0f;
    if (period_samples < 16U) {
        confidence -= 200.0f;
    }
    if (confidence < 0.0f) {
        confidence = 0.0f;
    }
    if (confidence > 1000.0f) {
        confidence = 1000.0f;
    }
    return static_cast<uint32_t>(lrintf(confidence));
}

static void EvaluateCaptureQuality(const clean_capture_stats_t *stats,
                                   float amdf_score,
                                   uint32_t amdf_confidence_x1000,
                                   esp_recon_quality_t *quality)
{
    if (quality == nullptr) {
        return;
    }

    quality->flags = 0U;
    quality->amdf_score = amdf_score;
    quality->amdf_confidence_x1000 = amdf_confidence_x1000;
    if (stats != nullptr) {
        quality->spike_count = stats->spike_count;
        quality->spike_replaced_count = stats->spike_replaced_count;
        quality->max_consecutive_spikes = stats->max_consecutive_spikes;
    }

    if (quality->spike_replaced_count > 0U) {
        quality->flags |= ESP_RECON_QUALITY_WARN_SPIKES;
    }
    if (quality->amdf_confidence_x1000 < 500U) {
        quality->flags |= ESP_RECON_QUALITY_WARN_AMDF;
    }
    if (quality->spike_replaced_count > (kN / 100U) ||
        quality->max_consecutive_spikes >= 4U ||
        quality->amdf_confidence_x1000 < 250U) {
        quality->flags |= ESP_RECON_QUALITY_UNSTABLE;
    }
}

static esp_recon_detected_shape_t DetectReconShape(const float *raw_amp,
                                                   uint32_t max_harmonic,
                                                   uint32_t f0_hz)
{
    (void)f0_hz;
    if (raw_amp == nullptr || max_harmonic == 0U || raw_amp[1] <= 1.0f) {
        return ESP_RECON_SHAPE_UNKNOWN;
    }

    const float h1 = raw_amp[1];
    const float h2 = (max_harmonic >= 2U) ? raw_amp[2] : 0.0f;
    const float h3 = (max_harmonic >= 3U) ? raw_amp[3] : 0.0f;
    const float h5 = (max_harmonic >= 5U) ? raw_amp[5] : 0.0f;
    const float h7 = (max_harmonic >= 7U) ? raw_amp[7] : 0.0f;
    const float odd_sum = h3 + h5 + h7;
    const float even_sum = h2 + ((max_harmonic >= 4U) ? raw_amp[4] : 0.0f) +
                           ((max_harmonic >= 6U) ? raw_amp[6] : 0.0f);

    if (h3 < h1 * 0.12f && h5 < h1 * 0.06f) {
        return ESP_RECON_SHAPE_SINE;
    }
    if (odd_sum > h1 * 0.35f && even_sum < odd_sum * 0.25f) {
        if (h3 > h5 && h5 > h7 * 0.7f) {
            return ESP_RECON_SHAPE_TRIANGLE;
        }
        return ESP_RECON_SHAPE_SQUARE;
    }
    if (even_sum > h1 * 0.10f) {
        return ESP_RECON_SHAPE_ARB;
    }
    return ESP_RECON_SHAPE_TRIANGLE;
}

static bool ShouldKeepHarmonic(esp_recon_mode_t mode,
                               esp_recon_detected_shape_t detected_shape,
                               uint32_t harmonic,
                               uint32_t period_samples,
                               uint32_t freq_hz,
                               float amp,
                               float h1_amp,
                               uint32_t *kept_count)
{
    if (harmonic == 0U || kept_count == nullptr) {
        return false;
    }

    const esp_recon_mode_t effective_mode = EffectiveReconMode(mode, detected_shape);
    const bool square_mode = (effective_mode == ESP_RECON_MODE_SQUARE);
    const uint32_t max_kept = square_mode ? kSquareMaxKeptHarmonics : 50U;
    if (*kept_count >= max_kept) {
        return false;
    }

    bool keep = false;
    if (square_mode) {
        const uint32_t odd_limit = SquareOddHarmonicLimit(period_samples);
        if ((harmonic & 1U) != 0U && harmonic <= odd_limit &&
            harmonic <= kSquareOddMaxHarmonic &&
            freq_hz <= kSquareMaxHarmonicFreqHz) {
            keep = true;
        } else if ((harmonic & 1U) == 0U && amp >= h1_amp * 0.20f &&
                   freq_hz <= kSquareMaxHarmonicFreqHz) {
            keep = true;
        }
    } else if (effective_mode == ESP_RECON_MODE_ARB) {
        keep = harmonic <= 40U;
    } else {
        keep = ((harmonic & 1U) != 0U) && harmonic <= 9U;
        if (detected_shape == ESP_RECON_SHAPE_SINE && harmonic > 3U) {
            keep = false;
        }
    }

    if (keep) {
        ++(*kept_count);
    }
    return keep;
}

static float AmdfScore(const int16_t *capture, uint32_t n, int32_t mean, uint32_t lag)
{
    if (capture == nullptr || lag == 0U || lag >= n) {
        return 1.0e30f;
    }
    uint64_t sum = 0;
    const uint32_t count = n - lag;
    for (uint32_t i = 0; i < count; ++i) {
        const int32_t a = static_cast<int32_t>(capture[i]) - mean;
        const int32_t b = static_cast<int32_t>(capture[i + lag]) - mean;
        const int32_t d = a - b;
        sum += static_cast<uint32_t>((d < 0) ? -d : d);
    }
    return static_cast<float>(sum) / static_cast<float>(count);
}

static uint32_t EstimatePeriodByAmdf(const int16_t *capture,
                                     uint32_t n,
                                     uint32_t sample_rate_hz,
                                     int32_t mean,
                                     float *best_score_out)
{
    if (capture == nullptr || n < 32U || sample_rate_hz == 0U) {
        return 0U;
    }

    uint32_t lag_min = sample_rate_hz / kF0MaxHz;
    uint32_t lag_max = sample_rate_hz / kF0MinHz;
    lag_min = ClampU32(lag_min, kAmdfLagMinFloor, n / 2U);
    lag_max = ClampU32(lag_max, lag_min, kAmdfLagMaxCeil);
    if (lag_max >= n / 2U) {
        lag_max = (n / 2U) - 1U;
    }
    if (lag_min == 0U || lag_min > lag_max) {
        return 0U;
    }

    float best_score = 1.0e30f;
    uint32_t best_lag = 0U;
    for (uint32_t lag = lag_min; lag <= lag_max; ++lag) {
        const float score = AmdfScore(capture, n, mean, lag);
        if (score < best_score) {
            best_score = score;
            best_lag = lag;
        }
        if ((lag & 15U) == 0U) {
            vTaskDelay(1);
        }
    }

    if (best_lag == 0U || best_score <= 0.0f) {
        return 0U;
    }

    const float close_score =
        best_score * (static_cast<float>(kAmdfClosePercent) / 100.0f);
    uint32_t corrected_lag = best_lag;
    for (uint32_t lag = lag_min; lag <= best_lag; ++lag) {
        if (AmdfScore(capture, n, mean, lag) <= close_score) {
            corrected_lag = lag;
            break;
        }
        if ((lag & 15U) == 0U) {
            vTaskDelay(1);
        }
    }

    if (best_score_out != nullptr) {
        *best_score_out = best_score;
    }
    return corrected_lag;
}

static void MakeLoopContinuous(float *x, uint32_t n)
{
    if (x == nullptr || n < 2U) {
        return;
    }

    // The DAC repeats the arbitrary-wave RAM forever. If the captured/rebuilt
    // frame does not contain an integer number of periods, sample[n-1] jumps
    // back to sample[0] at wrap and the scope shows a notch. Remove the linear
    // end-to-start mismatch first.
    const float end_error = x[n - 1U] - x[0];
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n - 1U);
        x[i] -= end_error * t;
    }

    // Then crossfade a small wrap region. This is deliberately conservative:
    // it only touches the first/last 16 samples and is meant to hide the loop
    // boundary, not reshape the waveform.
    constexpr uint32_t kFade = 16U;
    if (n <= kFade * 2U) {
        return;
    }
    for (uint32_t i = 0; i < kFade; ++i) {
        const float a = static_cast<float>(i + 1U) / static_cast<float>(kFade + 1U);
        const uint32_t tail = n - kFade + i;
        const float blended = x[tail] * (1.0f - a) + x[i] * a;
        x[tail] = blended;
    }
    x[n - 1U] = x[0];
}

static bool InterpolateModel(const circuit_model_t *model, uint32_t freq_hz,
                             float *gain, float *phase_rad)
{
    if (gain != nullptr) {
        *gain = 1.0f;
    }
    if (phase_rad != nullptr) {
        *phase_rad = 0.0f;
    }
    if (model == nullptr || !model->valid || model->point_count == 0U) {
        return false;
    }

    const uint32_t count = (model->point_count > MODEL_POINTS_MAX) ? MODEL_POINTS_MAX : model->point_count;
    const model_point_t *p = model->points;
    if (freq_hz <= p[0].freq_hz || count == 1U) {
        if (gain != nullptr) {
            *gain = fmaxf(static_cast<float>(p[0].gain_x1000) / 1000.0f, kMinGain);
        }
        if (phase_rad != nullptr) {
            *phase_rad = static_cast<float>(p[0].phase_deg_x10) * (kPi / 1800.0f);
        }
        return true;
    }

    for (uint32_t i = 1; i < count; ++i) {
        if (freq_hz <= p[i].freq_hz) {
            const float f0 = static_cast<float>(p[i - 1].freq_hz);
            const float f1 = static_cast<float>(p[i].freq_hz);
            const float t = (f1 > f0) ? ((static_cast<float>(freq_hz) - f0) / (f1 - f0)) : 0.0f;
            if (gain != nullptr) {
                const float g0 = static_cast<float>(p[i - 1].gain_x1000) / 1000.0f;
                const float g1 = static_cast<float>(p[i].gain_x1000) / 1000.0f;
                *gain = fmaxf(g0 + (g1 - g0) * t, kMinGain);
            }
            if (phase_rad != nullptr) {
                const float ph0 = static_cast<float>(p[i - 1].phase_deg_x10) * (kPi / 1800.0f);
                const float ph1 = static_cast<float>(p[i].phase_deg_x10) * (kPi / 1800.0f);
                *phase_rad = WrapPhaseRad(ph0 + WrapPhaseRad(ph1 - ph0) * t);
            }
            return true;
        }
    }

    if (gain != nullptr) {
        *gain = fmaxf(static_cast<float>(p[count - 1].gain_x1000) / 1000.0f, kMinGain);
    }
    if (phase_rad != nullptr) {
        *phase_rad = static_cast<float>(p[count - 1].phase_deg_x10) * (kPi / 1800.0f);
    }
    return true;
}

[[maybe_unused]] static void ForwardDft(const int16_t *capture, uint32_t n, int32_t mean, Work *w)
{
    for (uint32_t k = 0; k < n; ++k) {
        float re = 0.0f;
        float im = 0.0f;
        const float step = -kTwoPi * static_cast<float>(k) / static_cast<float>(n);
        const float c_step = cosf(step);
        const float s_step = sinf(step);
        float c = 1.0f;
        float s = 0.0f;
        for (uint32_t t = 0; t < n; ++t) {
            const float x = static_cast<float>(capture[t] - mean);
            re += x * c;
            im += x * s;
            const float next_c = c * c_step - s * s_step;
            const float next_s = s * c_step + c * s_step;
            c = next_c;
            s = next_s;
        }
        w->xr[k] = re;
        w->xi[k] = im;
        if ((k & (kYieldEveryBins - 1U)) == 0U) {
            vTaskDelay(1);
        }
    }
}

[[maybe_unused]] static void InverseDft(uint32_t n, Work *w)
{
    for (uint32_t t = 0; t < n; ++t) {
        float x = 0.0f;
        const float step = kTwoPi * static_cast<float>(t) / static_cast<float>(n);
        const float c_step = cosf(step);
        const float s_step = sinf(step);
        float c = 1.0f;
        float s = 0.0f;
        for (uint32_t k = 0; k < n; ++k) {
            x += w->xr[k] * c - w->xi[k] * s;
            const float next_c = c * c_step - s * s_step;
            const float next_s = s * c_step + c * s_step;
            c = next_c;
            s = next_s;
        }
        w->tr[t] = x / static_cast<float>(n);
        if ((t & (kYieldEveryBins - 1U)) == 0U) {
            vTaskDelay(1);
        }
    }
}

[[maybe_unused]] static uint32_t DominantBin(const Work *w, uint32_t n)
{
    uint32_t best = 1U;
    float best_mag = 0.0f;
    for (uint32_t k = 1; k < n / 2U; ++k) {
        const float mag = w->xr[k] * w->xr[k] + w->xi[k] * w->xi[k];
        if (mag > best_mag) {
            best_mag = mag;
            best = k;
        }
    }
    return best;
}

[[maybe_unused]] static void SmoothCircular3Tap(float *x, float *tmp, uint32_t n)
{
    if (x == nullptr || tmp == nullptr || n < 3U) {
        return;
    }
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t prev = (i == 0U) ? (n - 1U) : (i - 1U);
        const uint32_t next = (i + 1U >= n) ? 0U : (i + 1U);
        tmp[i] = 0.25f * x[prev] + 0.5f * x[i] + 0.25f * x[next];
    }
    memcpy(x, tmp, n * sizeof(x[0]));
}

static float BinMag(const Work *w, uint32_t bin)
{
    return sqrtf(w->xr[bin] * w->xr[bin] + w->xi[bin] * w->xi[bin]);
}

[[maybe_unused]] static float RefinePeakBin(const Work *w, uint32_t n, uint32_t bin)
{
    if (bin <= 1U || bin + 1U >= n / 2U) {
        return static_cast<float>(bin);
    }
    const float ym1 = BinMag(w, bin - 1U);
    const float y0 = BinMag(w, bin);
    const float yp1 = BinMag(w, bin + 1U);
    const float denom = ym1 - 2.0f * y0 + yp1;
    if (fabsf(denom) < 1.0e-6f) {
        return static_cast<float>(bin);
    }
    float delta = 0.5f * (ym1 - yp1) / denom;
    if (delta > 0.5f) {
        delta = 0.5f;
    } else if (delta < -0.5f) {
        delta = -0.5f;
    }
    return static_cast<float>(bin) + delta;
}

static uint32_t FindLocalPeakBin(const Work *w, uint32_t n, float expected_bin)
{
    const uint32_t nyquist = n / 2U;
    int32_t center = static_cast<int32_t>(lrintf(expected_bin));
    if (center < 1) {
        center = 1;
    }
    const int32_t span = static_cast<int32_t>(ESP_RECON_HARMONIC_SEARCH_SPAN);
    int32_t start = center - span;
    int32_t stop = center + span;
    if (start < 1) {
        start = 1;
    }
    if (stop >= static_cast<int32_t>(nyquist)) {
        stop = static_cast<int32_t>(nyquist) - 1;
    }

    uint32_t best = static_cast<uint32_t>(start);
    float best_mag = -1.0f;
    for (int32_t k = start; k <= stop; ++k) {
        const float mag = BinMag(w, static_cast<uint32_t>(k));
        if (mag > best_mag) {
            best_mag = mag;
            best = static_cast<uint32_t>(k);
        }
    }
    return best;
}

static void DftAtLockedHarmonic(const int16_t *capture,
                                uint32_t use_n,
                                int32_t mean,
                                uint32_t period_samples,
                                uint32_t harmonic,
                                float *re_out,
                                float *im_out)
{
    float re = 0.0f;
    float im = 0.0f;
    const float step =
        -kTwoPi * static_cast<float>(harmonic) / static_cast<float>(period_samples);
    const float c_step = cosf(step);
    const float s_step = sinf(step);
    float c = 1.0f;
    float s = 0.0f;

    for (uint32_t i = 0; i < use_n; ++i) {
        const float x = static_cast<float>(capture[i] - mean);
        re += x * c;
        im += x * s;
        const float next_c = c * c_step - s * s_step;
        const float next_s = s * c_step + c * s_step;
        c = next_c;
        s = next_s;
    }

    if (re_out != nullptr) {
        *re_out = re;
    }
    if (im_out != nullptr) {
        *im_out = im;
    }
}

static bool ExtractLockedHarmonics(const int16_t *capture,
                                   uint32_t n,
                                   uint32_t sample_rate_hz,
                                   uint32_t period_samples,
                                   const circuit_model_t *model,
                                   esp_recon_result_t *out,
                                   uint32_t *use_n_out)
{
    if (capture == nullptr || out == nullptr || sample_rate_hz == 0U ||
        period_samples == 0U || period_samples >= n) {
        return false;
    }

    const uint32_t cycles = n / period_samples;
    if (cycles < kMinLockedCycles) {
        ESP_LOGW(TAG,
                 "AMDF reject: period=%lu cycles=%lu",
                 static_cast<unsigned long>(period_samples),
                 static_cast<unsigned long>(cycles));
        return false;
    }

    const uint32_t use_n = cycles * period_samples;
    const uint32_t f0_hz = static_cast<uint32_t>(
        (static_cast<uint64_t>(sample_rate_hz) + period_samples / 2U) /
        period_samples);
    uint32_t max_harmonic = period_samples / 2U;
    if (max_harmonic > kMaxLockedHarmonics) {
        max_harmonic = kMaxLockedHarmonics;
    }
    if (max_harmonic > ESP_RECON_HARMONIC_MAX) {
        max_harmonic = ESP_RECON_HARMONIC_MAX;
    }
    if (f0_hz == 0U || max_harmonic == 0U) {
        return false;
    }

    const int32_t mean = MeanOfWindow(capture, use_n);
    float raw_amp[ESP_RECON_HARMONIC_MAX + 1U] = {};
    float raw_phase[ESP_RECON_HARMONIC_MAX + 1U] = {};

    for (uint32_t harmonic = 1U; harmonic <= max_harmonic; ++harmonic) {
        float re = 0.0f;
        float im = 0.0f;
        DftAtLockedHarmonic(capture, use_n, mean, period_samples,
                            harmonic, &re, &im);
        raw_amp[harmonic] = sqrtf(re * re + im * im) * 2.0f / static_cast<float>(use_n);
        raw_phase[harmonic] = atan2f(-im, re);
        if ((harmonic & 3U) == 0U) {
            vTaskDelay(1);
        }
    }

    out->detected_shape = DetectReconShape(raw_amp, max_harmonic, f0_hz);

    const bool square_mode = IsSquareReconMode(out->mode, out->detected_shape);
    const float amp_floor = square_mode ?
        fmaxf(kSquareAbsoluteFloorMv, raw_amp[1] * kSquareRelativeFloor) :
        fmaxf(kHarmonicAbsoluteFloor, raw_amp[1] * kHarmonicRelativeFloor);
    out->harmonic_count = 0U;
    g_last_harmonic_count = 0U;
    out->dominant_bin = static_cast<uint32_t>(
        (static_cast<uint64_t>(f0_hz) * n + sample_rate_hz / 2U) /
        sample_rate_hz);
    out->dominant_freq_hz = f0_hz;

    uint32_t kept_count = 0U;
    for (uint32_t harmonic = 1U; harmonic <= max_harmonic; ++harmonic) {
        if (raw_amp[harmonic] < amp_floor) {
            continue;
        }

        const uint32_t freq_hz = f0_hz * harmonic;
        if (freq_hz >= sample_rate_hz / 2U) {
            break;
        }

        float gain = 1.0f;
        float phase = 0.0f;
        const bool have_model = InterpolateModel(model, freq_hz, &gain, &phase);
        const float min_gain = square_mode ? kSquareMinHarmonicGain : kMinHarmonicGain;
        if (have_model && gain < min_gain) {
            ESP_LOGW(TAG,
                     "H%lu drop: freq=%luHz raw_amp=%.1f H=%.3f",
                     static_cast<unsigned long>(harmonic),
                     static_cast<unsigned long>(freq_hz),
                     static_cast<double>(raw_amp[harmonic]),
                     static_cast<double>(gain));
            continue;
        }

        float inv_gain = 1.0f / gain;
        if (inv_gain > kMaxInvGain) {
            inv_gain = kMaxInvGain;
        }
        float comp_amp = raw_amp[harmonic] * inv_gain;
        const float phase_sign = PhaseSignFloat();
        float comp_phase = WrapPhaseRad(phase_sign * raw_phase[harmonic]);
#if ESP_RECON_STAGE >= 2
        comp_phase = WrapPhaseRad(comp_phase + phase_sign * phase);
#endif

        if (!ShouldKeepHarmonic(out->mode,
                                out->detected_shape,
                                harmonic,
                                period_samples,
                                freq_hz,
                                comp_amp,
                                raw_amp[1],
                                &kept_count)) {
            continue;
        }

        esp_recon_harmonic_t h = {};
        h.index = harmonic;
        h.bin = static_cast<uint32_t>(
            (static_cast<uint64_t>(freq_hz) * n + sample_rate_hz / 2U) /
            sample_rate_hz);
        h.freq_hz = freq_hz;
        h.amp_mv = static_cast<int32_t>(lrintf(comp_amp));
        h.phase_deg_x10 = static_cast<int32_t>(lrintf(comp_phase * (1800.0f / kPi)));
        h.flags = have_model ? 0U : 0x01U;

        if (g_last_harmonic_count < ESP_RECON_HARMONIC_MAX) {
            g_last_harmonics[g_last_harmonic_count++] = h;
        }
        if (out->harmonic_count < ESP_RECON_HARMONIC_MAX) {
            out->harmonics[out->harmonic_count++] = h;
        }

        if (out->harmonic_count <= 10U) {
            ESP_LOGW(TAG,
                     "H%lu locked freq=%luHz raw_amp=%.1f amp=%.1f phase=%.1fdeg shape=%u mode=%u",
                     static_cast<unsigned long>(harmonic),
                     static_cast<unsigned long>(freq_hz),
                     static_cast<double>(raw_amp[harmonic]),
                     static_cast<double>(comp_amp),
                     static_cast<double>(h.phase_deg_x10) / 10.0,
                     static_cast<unsigned>(out->detected_shape),
                     static_cast<unsigned>(out->mode));
        }
    }

    if (use_n_out != nullptr) {
        *use_n_out = use_n;
    }
    return out->harmonic_count != 0U;
}

[[maybe_unused]] static void LogHarmonics(const Work *w,
                                          uint32_t n,
                                          uint32_t sample_rate_hz,
                                          float fundamental_bin,
                                          const circuit_model_t *model,
                                          esp_recon_result_t *out)
{
    g_last_harmonic_count = 0;
    if (out != nullptr) {
        out->harmonic_count = 0;
    }
    if (fundamental_bin < 0.5f) {
        return;
    }

    const float nyquist_bin = static_cast<float>(n / 2U);
    bool used_bins[kN / 2U] = {};
    for (uint32_t harmonic = 1U; harmonic <= ESP_RECON_HARMONIC_MAX; ++harmonic) {
#if ESP_RECON_SYNTH_ODD_HARMONICS_ONLY
        if ((harmonic & 1U) == 0U) {
            continue;
        }
#endif
        const float expected_bin = fundamental_bin * static_cast<float>(harmonic);
        if (expected_bin >= nyquist_bin) {
            break;
        }
        const uint32_t peak_bin = FindLocalPeakBin(w, n, expected_bin);
        if (peak_bin == 0U || peak_bin >= n / 2U || used_bins[peak_bin]) {
            ESP_LOGW(TAG,
                     "H%lu skip duplicate/invalid peak_bin=%lu expect_bin=%.2f",
                     static_cast<unsigned long>(harmonic),
                     static_cast<unsigned long>(peak_bin),
                     static_cast<double>(expected_bin));
            continue;
        }
        used_bins[peak_bin] = true;
        const float mag = BinMag(w, peak_bin) * 2.0f / static_cast<float>(n);
        const uint32_t freq_hz = (peak_bin * sample_rate_hz) / n;
        float gain = 1.0f;
        float phase = 0.0f;
        const bool have_model = InterpolateModel(model, freq_hz, &gain, &phase);
        const float min_gain = IsSquareReconMode(out != nullptr ? out->mode : ESP_RECON_MODE_AUTO,
                                                 out != nullptr ? out->detected_shape : ESP_RECON_SHAPE_UNKNOWN) ?
                               kSquareMinHarmonicGain : kMinHarmonicGain;
        if (have_model && gain < min_gain) {
            continue;
        }
        float inv_gain = 1.0f / gain;
        if (inv_gain > kMaxInvGain) {
            inv_gain = kMaxInvGain;
        }
        const float amp = mag * inv_gain;
        const float phase_sign = PhaseSignFloat();
        float phase_cos = WrapPhaseRad(phase_sign * atan2f(-w->xi[peak_bin], w->xr[peak_bin]));
#if ESP_RECON_STAGE >= 2
        phase_cos = WrapPhaseRad(phase_cos + phase_sign * phase);
#endif
        const int32_t phase_x10 =
            static_cast<int32_t>(lrintf(phase_cos * (1800.0f / kPi)));
        esp_recon_harmonic_t h = {};
        h.index = harmonic;
        h.bin = peak_bin;
        h.freq_hz = freq_hz;
        h.amp_mv = static_cast<int32_t>(lrintf(amp));
        h.phase_deg_x10 = phase_x10;
        h.flags = have_model ? 0U : 0x01U;
        if (g_last_harmonic_count < ESP_RECON_HARMONIC_MAX) {
            g_last_harmonics[g_last_harmonic_count++] = h;
        }
        if (out != nullptr && out->harmonic_count < ESP_RECON_HARMONIC_MAX) {
            out->harmonics[out->harmonic_count++] = h;
        }
        ESP_LOGW(TAG,
                 "H%lu expect_bin=%.2f peak_bin=%lu freq=%luHz raw_amp=%.1f amp=%.1f phase=%.1fdeg",
                 static_cast<unsigned long>(harmonic),
                 static_cast<double>(expected_bin),
                 static_cast<unsigned long>(peak_bin),
                 static_cast<unsigned long>(freq_hz),
                 static_cast<double>(mag),
                 static_cast<double>(amp),
                 static_cast<double>(phase_x10) / 10.0);
    }
}

static uint32_t RoundedPlaybackCycles(uint32_t freq_hz,
                                      uint32_t sample_count,
                                      uint32_t playback_rate_hz)
{
    if (freq_hz == 0U || sample_count == 0U || playback_rate_hz == 0U) {
        return 0U;
    }
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(freq_hz) * sample_count + playback_rate_hz / 2U) /
        playback_rate_hz);
}

static bool IsOutputGainValid(uint32_t gain_x1000)
{
    return gain_x1000 == 250U ||
           gain_x1000 == 500U ||
           gain_x1000 == 1000U ||
           gain_x1000 == 2000U ||
           gain_x1000 == 4000U;
}

static int32_t ReconTargetPeak(void)
{
    uint32_t target_peak =
        static_cast<uint32_t>((static_cast<uint64_t>(ESP_RECON_TARGET_PEAK) *
                               g_recon_output_gain_x1000 + 500U) /
                              1000U);
    if (target_peak == 0U) {
        target_peak = 1U;
    }
    if (target_peak > 32767U) {
        target_peak = 32767U;
    }
    return static_cast<int32_t>(target_peak);
}

static bool BuildPeriodicWaveFromHarmonics(const esp_recon_result_t *out,
                                           float *dst,
                                           uint32_t dst_capacity,
                                           uint32_t *playback_sample_rate_hz,
                                           uint32_t *playback_sample_count)
{
    if (out == nullptr || dst == nullptr || dst_capacity < 8U || out->harmonic_count == 0U ||
        out->dominant_freq_hz == 0U) {
        return false;
    }

    const uint32_t playback_rate_hz = g_recon_dds_playback_rate_hz;
    uint32_t cycles = RoundedPlaybackCycles(out->dominant_freq_hz,
                                            dst_capacity,
                                            playback_rate_hz);
    if (cycles == 0U) {
        cycles = 1U;
    }

    uint32_t dst_n = static_cast<uint32_t>(
        ((static_cast<uint64_t>(cycles) * playback_rate_hz) +
         (out->dominant_freq_hz / 2U)) /
        out->dominant_freq_hz);
    if (dst_n > dst_capacity) {
        dst_n = dst_capacity;
    }
    if (dst_n < 8U || cycles >= dst_n / 2U) {
        return false;
    }

    for (uint32_t i = 0; i < dst_n; ++i) {
        float y = 0.0f;
        for (uint32_t h = 0; h < out->harmonic_count; ++h) {
            const esp_recon_harmonic_t *harm = &out->harmonics[h];
            if (harm->index == 0U) {
                continue;
            }
#if ESP_RECON_SYNTH_ODD_HARMONICS_ONLY
            if ((harm->index & 1U) == 0U) {
                continue;
            }
#endif
            if (harm->index * cycles >= dst_n / 2U) {
                continue;
            }
            const float amp = static_cast<float>(harm->amp_mv);
            if (fabsf(amp) < 0.5f) {
                continue;
            }
            const float phase_cos =
                static_cast<float>(harm->phase_deg_x10) * (kPi / 1800.0f);
            const float phase_ref =
                SynthReferencePhaseRad() * static_cast<float>(harm->index);
            const float theta =
                kTwoPi *
                static_cast<float>(harm->index * cycles) *
                static_cast<float>(i) /
                static_cast<float>(dst_n);
            const float synth_phase = theta + phase_cos + phase_ref;
            y += (ClampSinBasis(g_phase_debug.sin_basis) != 0) ?
                 (amp * sinf(synth_phase)) : (amp * cosf(synth_phase));
        }
        dst[i] = y;
        if ((i & 127U) == 0U) {
            vTaskDelay(1);
        }
    }

    if (playback_sample_rate_hz != nullptr) {
        *playback_sample_rate_hz = playback_rate_hz;
    }
    if (playback_sample_count != nullptr) {
        *playback_sample_count = dst_n;
    }
    return true;
}

} // namespace

bool EspRecon_BuildFromCapture(const int16_t *capture,
                               uint32_t sample_count,
                               uint32_t sample_rate_hz,
                               const circuit_model_t *model,
                               esp_recon_result_t *out)
{
    if (capture == nullptr || out == nullptr || sample_count != kN || sample_rate_hz == 0U) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->sample_count = sample_count;
    out->sample_rate_hz = sample_rate_hz;
    out->capture_sample_rate_hz = sample_rate_hz;
    out->playback_sample_rate_hz = g_recon_dds_playback_rate_hz;
    out->mode = g_recon_mode;
    out->detected_shape = ESP_RECON_SHAPE_UNKNOWN;
    CaptureStats(capture, sample_count, out);
    g_last_harmonic_count = 0;

    Work *w = AllocWork();
    if (w == nullptr) {
        ESP_LOGE(TAG, "work alloc failed");
        return false;
    }

    bool locked_harmonics = false;
    uint32_t locked_period = 0U;
    uint32_t locked_use_n = 0U;
    float amdf_best = 0.0f;
    clean_capture_stats_t clean_stats = {};

    BuildCleanCaptureAndStats(capture, sample_count, out->cap_mean, w->clean_capture, &clean_stats);

#if ESP_RECON_STAGE == 0
    for (uint32_t i = 0; i < sample_count; ++i) {
        w->tr[i] = static_cast<float>(w->clean_capture[i] - out->cap_mean);
    }
    out->dominant_bin = 0;
    out->dominant_freq_hz = 0;
#else
    locked_period = EstimatePeriodByAmdf(w->clean_capture, sample_count, sample_rate_hz,
                                         out->cap_mean, &amdf_best);
    if (locked_period != 0U) {
        locked_harmonics = ExtractLockedHarmonics(w->clean_capture, sample_count,
                                                  sample_rate_hz, locked_period,
                                                  model, out, &locked_use_n);
    }

    if (!locked_harmonics) {
        ESP_LOGW(TAG,
                 "AMDF did not lock f0: period=%lu score=%.1f",
                 static_cast<unsigned long>(locked_period),
                 static_cast<double>(amdf_best));
#if ESP_RECON_OUTPUT_USE_HARMONIC_SYNTH
        for (uint32_t i = 0; i < sample_count; ++i) {
            w->tr[i] = static_cast<float>(w->clean_capture[i] - out->cap_mean);
        }
        out->dominant_bin = 0U;
        out->dominant_freq_hz = 0U;
        out->harmonic_count = 0U;
#else
        ForwardDft(w->clean_capture, sample_count, out->cap_mean, w);
        out->dominant_bin = DominantBin(w, sample_count);
        const float fundamental_bin = RefinePeakBin(w, sample_count, out->dominant_bin);
        out->dominant_freq_hz = static_cast<uint32_t>(
            lrintf((fundamental_bin * static_cast<float>(sample_rate_hz)) /
                   static_cast<float>(sample_count)));
        LogHarmonics(w, sample_count, sample_rate_hz, fundamental_bin, model, out);
#endif
    }

#if !ESP_RECON_OUTPUT_USE_HARMONIC_SYNTH
    if (!locked_harmonics) {
        w->xr[0] = 0.0f;
        w->xi[0] = 0.0f;
        for (uint32_t k = 1; k < sample_count; ++k) {
            const uint32_t folded_bin = (k <= sample_count / 2U) ? k : (sample_count - k);
            const uint32_t freq_hz = (folded_bin * sample_rate_hz) / sample_count;
            float gain = 1.0f;
            float phase = 0.0f;
            InterpolateModel(model, freq_hz, &gain, &phase);
            float inv_gain = 1.0f / gain;
            if (inv_gain > kMaxInvGain) {
                inv_gain = kMaxInvGain;
            }

            float re = w->xr[k] * inv_gain;
            float im = w->xi[k] * inv_gain;

#if ESP_RECON_STAGE >= 2
            const float phase_sign = PhaseSignFloat();
            const float signed_phase =
                phase_sign * phase;
            const float comp_phase = (k <= sample_count / 2U) ? -signed_phase : signed_phase;
            const float c = cosf(comp_phase);
            const float s = sinf(comp_phase);
            const float rr = re * c - im * s;
            const float ii = re * s + im * c;
            re = rr;
            im = ii;
#endif

            w->xr[k] = re;
            w->xi[k] = im;
            if ((k & 63U) == 0U) {
                vTaskDelay(1);
            }
        }
        InverseDft(sample_count, w);
    }
#endif
#endif

    out->quality.spike_count = clean_stats.spike_count;
    out->quality.spike_replaced_count = clean_stats.spike_replaced_count;
    out->quality.max_consecutive_spikes = clean_stats.max_consecutive_spikes;
    out->quality.amdf_confidence_x1000 =
        EstimateAmdfConfidenceX1000(amdf_best, clean_stats.rms, locked_period);
    EvaluateCaptureQuality(&clean_stats,
                           amdf_best,
                           out->quality.amdf_confidence_x1000,
                           &out->quality);

    uint32_t output_count = sample_count;
    const float *output_src = w->tr;
    uint32_t playback_sample_rate_hz = g_recon_dds_playback_rate_hz;
    uint32_t periodic_sample_count = kOutN;
    uint32_t output_cycles = 0U;
#if ESP_RECON_STAGE >= 1 && ESP_RECON_PERIODIC_SYNTH_ENABLE && ESP_RECON_OUTPUT_USE_HARMONIC_SYNTH
    if (BuildPeriodicWaveFromHarmonics(out, w->synth, kOutN,
                                       &playback_sample_rate_hz,
                                       &periodic_sample_count)) {
        output_src = w->synth;
        output_count = periodic_sample_count;
        out->playback_sample_rate_hz = playback_sample_rate_hz;
        out->sample_rate_hz = out->capture_sample_rate_hz;
        output_cycles = RoundedPlaybackCycles(out->dominant_freq_hz,
                                              output_count,
                                              out->playback_sample_rate_hz);
    } else
#endif
    {
        if (locked_harmonics && locked_use_n != 0U) {
            const int32_t locked_mean = MeanOfWindow(w->clean_capture, locked_use_n);
            for (uint32_t i = 0; i < locked_use_n; ++i) {
                w->tr[i] = static_cast<float>(w->clean_capture[i] - locked_mean);
            }
            output_count = locked_use_n;
        } else {
            output_count = sample_count;
        }
        MakeLoopContinuous(w->tr, output_count);
        output_src = w->tr;
        out->playback_sample_rate_hz = playback_sample_rate_hz;
        out->sample_rate_hz = out->capture_sample_rate_hz;
    }

#if ESP_RECON_OUTPUT_SMOOTH_ENABLE
    if (output_count <= kN) {
        memcpy(w->synth, output_src, output_count * sizeof(w->synth[0]));
        SmoothCircular3Tap(w->synth, w->tr, output_count);
        output_src = w->synth;
    } else if (output_count <= kOutN) {
        memcpy(w->tr, output_src, output_count * sizeof(w->tr[0]));
        SmoothCircular3Tap(w->tr, w->synth, output_count);
        output_src = w->tr;
    }
#endif

#if ESP_RECON_OUTPUT_INVERT
    if (output_count <= kOutN) {
        for (uint32_t i = 0; i < output_count; ++i) {
            w->synth[i] = -output_src[i];
        }
        output_src = w->synth;
    }
#endif

    NormalizeToInt16(output_src, output_count, ReconTargetPeak(), w->wave,
                     &out->out_min, &out->out_max);
    out->sample_count = output_count;
    out->out_vpp = out->out_max - out->out_min;
    memcpy(out->samples, w->wave, output_count * sizeof(out->samples[0]));

    const uint32_t actual_out_freq = (out->sample_count != 0U && output_cycles != 0U) ?
        static_cast<uint32_t>((static_cast<uint64_t>(output_cycles) * out->playback_sample_rate_hz +
                               out->sample_count / 2U) / out->sample_count) : 0U;
    const uint32_t window_ms = (out->capture_sample_rate_hz != 0U) ?
        static_cast<uint32_t>((static_cast<uint64_t>(sample_count) * 1000ULL) /
                              out->capture_sample_rate_hz) : 0U;
    ESP_LOGW(TAG,
             "recon mode=%u shape=%u spikes=%lu repl=%lu conf=%lu flags=0x%08lX stage=%d phase_sign=%ld ref_x10=%d sin_basis=%d cap=[%ld,%ld] vpp=%ld mean=%ld out=[%ld,%ld] vpp=%ld in_n=%lu use_n=%lu out_n=%lu capture_rate=%luHz playback_rate=%luHz window=%lums f0=%luHz period=%lu output_cycles=%lu actual_out=%luHz locked=%u",
             static_cast<unsigned>(out->mode),
             static_cast<unsigned>(out->detected_shape),
             static_cast<unsigned long>(out->quality.spike_count),
             static_cast<unsigned long>(out->quality.spike_replaced_count),
             static_cast<unsigned long>(out->quality.amdf_confidence_x1000),
             static_cast<unsigned long>(out->quality.flags),
             ESP_RECON_STAGE,
             static_cast<long>(g_phase_debug.phase_sign),
             static_cast<int>(g_phase_debug.reference_deg_x10),
             static_cast<int>(g_phase_debug.sin_basis),
             static_cast<long>(out->cap_min),
             static_cast<long>(out->cap_max),
             static_cast<long>(out->cap_vpp),
             static_cast<long>(out->cap_mean),
             static_cast<long>(out->out_min),
             static_cast<long>(out->out_max),
             static_cast<long>(out->out_vpp),
             static_cast<unsigned long>(sample_count),
             static_cast<unsigned long>(locked_use_n),
             static_cast<unsigned long>(out->sample_count),
             static_cast<unsigned long>(out->capture_sample_rate_hz),
             static_cast<unsigned long>(out->playback_sample_rate_hz),
             static_cast<unsigned long>(window_ms),
             static_cast<unsigned long>(out->dominant_freq_hz),
             static_cast<unsigned long>(locked_period),
             static_cast<unsigned long>(output_cycles),
             static_cast<unsigned long>(actual_out_freq),
             locked_harmonics ? 1U : 0U);

    heap_caps_free(w);
    return true;
}

bool EspRecon_SendFromCapture(const int16_t *capture,
                              uint32_t sample_count,
                              uint32_t sample_rate_hz,
                              const circuit_model_t *model)
{
    esp_recon_result_t *result = static_cast<esp_recon_result_t *>(
        heap_caps_calloc(1, sizeof(esp_recon_result_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (result == nullptr) {
        result = static_cast<esp_recon_result_t *>(
            heap_caps_calloc(1, sizeof(esp_recon_result_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (result == nullptr) {
        ESP_LOGE(TAG, "result alloc failed");
        return false;
    }

    const bool built = EspRecon_BuildFromCapture(capture, sample_count, sample_rate_hz, model, result);
    const bool sent = built && DdsDirect_SendWave(result->samples,
                                                  result->sample_count,
                                                  result->playback_sample_rate_hz);
    heap_caps_free(result);
    return sent;
}

uint32_t EspRecon_GetLastHarmonics(esp_recon_harmonic_t *out, uint32_t max_count)
{
    const uint32_t count =
        (g_last_harmonic_count < max_count) ? g_last_harmonic_count : max_count;
    if (out != nullptr && count != 0U) {
        memcpy(out, g_last_harmonics, count * sizeof(out[0]));
    }
    return count;
}

void EspRecon_SetMode(esp_recon_mode_t mode)
{
    g_recon_mode = mode;
}

esp_recon_mode_t EspRecon_GetMode(void)
{
    return g_recon_mode;
}

bool EspRecon_SetPlaybackRateHz(uint32_t sample_rate_hz)
{
    if (!DdsDirect_GetPlaybackRate(sample_rate_hz, nullptr)) {
        return false;
    }
    g_recon_dds_playback_rate_hz = sample_rate_hz;
    return true;
}

uint32_t EspRecon_GetPlaybackRateHz(void)
{
    return g_recon_dds_playback_rate_hz;
}

bool EspRecon_SetOutputGainX1000(uint32_t gain_x1000)
{
    if (!IsOutputGainValid(gain_x1000)) {
        return false;
    }
    g_recon_output_gain_x1000 = gain_x1000;
    return true;
}

uint32_t EspRecon_GetOutputGainX1000(void)
{
    return g_recon_output_gain_x1000;
}

void EspRecon_SetPhaseDebug(const esp_recon_phase_debug_t *config)
{
    if (config == nullptr) {
        return;
    }
    g_phase_debug.phase_sign = ClampPhaseSign(config->phase_sign);
    g_phase_debug.reference_deg_x10 = config->reference_deg_x10;
    g_phase_debug.sin_basis = ClampSinBasis(config->sin_basis);
}

void EspRecon_GetPhaseDebug(esp_recon_phase_debug_t *config)
{
    if (config == nullptr) {
        return;
    }
    *config = g_phase_debug;
}

void EspRecon_CyclePhaseDebug(void)
{
    constexpr uint32_t preset_count =
        sizeof(kPhaseDebugPresets) / sizeof(kPhaseDebugPresets[0]);
    g_phase_debug_preset = (g_phase_debug_preset + 1U) % preset_count;
    EspRecon_SetPhaseDebug(&kPhaseDebugPresets[g_phase_debug_preset]);
}
