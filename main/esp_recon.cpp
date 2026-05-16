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
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kMinGain = 0.05f;
constexpr float kMaxInvGain = 16.0f;
constexpr uint32_t kYieldEveryBins = 16U;

static esp_recon_harmonic_t g_last_harmonics[ESP_RECON_HARMONIC_MAX] = {};
static uint32_t g_last_harmonic_count = 0;

struct Work {
    float xr[kN];
    float xi[kN];
    float tr[kN];
    int16_t wave[kN];
};

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

static void MakeLoopContinuous(float *x, uint32_t n)
{
    if (x == nullptr || n < 2U) {
        return;
    }

    // The DAC repeats the 1024-point RAM forever. If the captured/rebuilt
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

static void ForwardDft(const int16_t *capture, uint32_t n, int32_t mean, Work *w)
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

static void InverseDft(uint32_t n, Work *w)
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

static uint32_t DominantBin(const Work *w, uint32_t n)
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

static void LogHarmonics(const Work *w, uint32_t n, uint32_t sample_rate_hz, esp_recon_result_t *out)
{
    uint32_t top_bin[8] = {};
    float top_mag[8] = {};
    for (uint32_t k = 1; k < n / 2U; ++k) {
        const float mag = sqrtf(w->xr[k] * w->xr[k] + w->xi[k] * w->xi[k]) * 2.0f / static_cast<float>(n);
        for (uint32_t s = 0; s < 8U; ++s) {
            if (mag > top_mag[s]) {
                for (uint32_t j = 7U; j > s; --j) {
                    top_mag[j] = top_mag[j - 1U];
                    top_bin[j] = top_bin[j - 1U];
                }
                top_mag[s] = mag;
                top_bin[s] = k;
                break;
            }
        }
    }
    g_last_harmonic_count = 0;
    if (out != nullptr) {
        out->harmonic_count = 0;
    }
    for (uint32_t i = 0; i < 8U; ++i) {
        if (top_bin[i] == 0U) {
            continue;
        }
        const uint32_t freq_hz = (top_bin[i] * sample_rate_hz) / n;
        const int32_t amp = static_cast<int32_t>(lrintf(top_mag[i]));
        const int32_t phase_x10 =
            static_cast<int32_t>(lrintf(atan2f(w->xi[top_bin[i]], w->xr[top_bin[i]]) * (1800.0f / kPi)));
        esp_recon_harmonic_t h = {};
        h.index = i + 1U;
        h.bin = top_bin[i];
        h.freq_hz = freq_hz;
        h.amp_mv = amp;
        h.phase_deg_x10 = phase_x10;
        h.flags = 0U;
        if (g_last_harmonic_count < ESP_RECON_HARMONIC_MAX) {
            g_last_harmonics[g_last_harmonic_count++] = h;
        }
        if (out != nullptr && out->harmonic_count < ESP_RECON_HARMONIC_MAX) {
            out->harmonics[out->harmonic_count++] = h;
        }
        ESP_LOGW(TAG,
                 "H%lu bin=%lu freq=%luHz amp=%.1f",
                 static_cast<unsigned long>(i + 1U),
                 static_cast<unsigned long>(top_bin[i]),
                 static_cast<unsigned long>(freq_hz),
                 static_cast<double>(top_mag[i]));
    }
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
    CaptureStats(capture, sample_count, out);
    g_last_harmonic_count = 0;

    Work *w = AllocWork();
    if (w == nullptr) {
        ESP_LOGE(TAG, "work alloc failed");
        return false;
    }

#if ESP_RECON_STAGE == 0
    for (uint32_t i = 0; i < sample_count; ++i) {
        w->tr[i] = static_cast<float>(capture[i] - out->cap_mean);
    }
    out->dominant_bin = 0;
    out->dominant_freq_hz = 0;
#else
    ForwardDft(capture, sample_count, out->cap_mean, w);
    out->dominant_bin = DominantBin(w, sample_count);
    out->dominant_freq_hz = (out->dominant_bin * sample_rate_hz) / sample_count;
    LogHarmonics(w, sample_count, sample_rate_hz, out);

    // Keep DC removed. Positive and negative bins are both compensated.
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
        const float comp_phase = (k <= sample_count / 2U) ? -phase : phase;
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
#endif

    MakeLoopContinuous(w->tr, sample_count);
    NormalizeToInt16(w->tr, sample_count, ESP_RECON_TARGET_PEAK, w->wave,
                     &out->out_min, &out->out_max);
    out->out_vpp = out->out_max - out->out_min;
    memcpy(out->samples, w->wave, sizeof(out->samples));

    ESP_LOGW(TAG,
             "recon stage=%d cap=[%ld,%ld] vpp=%ld mean=%ld out=[%ld,%ld] vpp=%ld dom=%luHz",
             ESP_RECON_STAGE,
             static_cast<long>(out->cap_min),
             static_cast<long>(out->cap_max),
             static_cast<long>(out->cap_vpp),
             static_cast<long>(out->cap_mean),
             static_cast<long>(out->out_min),
             static_cast<long>(out->out_max),
             static_cast<long>(out->out_vpp),
             static_cast<unsigned long>(out->dominant_freq_hz));

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
    const bool sent = built && DdsDirect_SendWave(result->samples, result->sample_count, result->sample_rate_hz);
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
