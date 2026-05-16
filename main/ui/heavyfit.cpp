#include "heavyfit.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace {

constexpr char TAG[] = "HeavyFit";
constexpr uint32_t kHeavyFitTimeBudgetMs = 10000U;
constexpr uint32_t kHeavyFitMaxSamples = 300U;
constexpr uint32_t kHeavyFitGridCount = 24U;

#ifndef HEAVYFIT_ENABLE_BAND_SHAPE_PRIOR
#define HEAVYFIT_ENABLE_BAND_SHAPE_PRIOR 1
#endif

#ifndef HEAVYFIT_OUTLIER_LOW_PERCENT
#define HEAVYFIT_OUTLIER_LOW_PERCENT 45
#endif

#ifndef HEAVYFIT_OUTLIER_HIGH_PERCENT
#define HEAVYFIT_OUTLIER_HIGH_PERCENT 220
#endif

#ifndef HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT
#define HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT 110
#endif

#ifndef HEAVYFIT_BAND_MID_PROMINENCE_PERCENT
#define HEAVYFIT_BAND_MID_PROMINENCE_PERCENT 106
#endif

#ifndef HEAVYFIT_BAND_PREFER_RMS_PERCENT
#define HEAVYFIT_BAND_PREFER_RMS_PERCENT 120
#endif

#ifndef HEAVYFIT_BANDPASS_EDGE_MAX_PERCENT
#define HEAVYFIT_BANDPASS_EDGE_MAX_PERCENT 75
#endif

#ifndef HEAVYFIT_RESONANT_LP2_PREFER_RMS_PERCENT
#define HEAVYFIT_RESONANT_LP2_PREFER_RMS_PERCENT 125
#endif

enum {
    MODEL_MASK_LP1 = 1U << 0,
    MODEL_MASK_HP1 = 1U << 1,
    MODEL_MASK_LP2 = 1U << 2,
    MODEL_MASK_HP2 = 1U << 3,
    MODEL_MASK_BP2 = 1U << 4,
    MODEL_MASK_BS2 = 1U << 5,
    MODEL_MASK_LP = MODEL_MASK_LP1 | MODEL_MASK_LP2,
    MODEL_MASK_HP = MODEL_MASK_HP1 | MODEL_MASK_HP2,
};

typedef enum {
    FIT_REJECT_NONE = 0,
    FIT_REJECT_INDEX,
    FIT_REJECT_FREQ,
    FIT_REJECT_LOW_VIN,
    FIT_REJECT_GAIN,
    FIT_REJECT_NON_MONOTONIC,
    FIT_REJECT_OUTLIER,
} fit_reject_reason_t;

typedef struct {
    uint32_t raw_point_count;
    uint32_t valid_point_count;
    uint32_t rejected_point_count;
    uint32_t rejected_outlier_count;
    uint32_t rejected_low_vin_count;
    uint32_t rejected_non_monotonic_count;
    uint32_t candidate_mask;
} fit_debug_stats_t;

typedef struct {
    uint32_t candidate_mask;
    uint32_t valid_count;
    uint32_t low_avg;
    uint32_t high_avg;
    uint32_t peak_gain;
    uint32_t peak_freq;
    uint32_t valley_gain;
    uint32_t valley_freq;
    bool low_high_low;
    bool high_low_high;
    bool true_band_pass;
    bool true_band_stop;
    bool resonant_low_pass;
} fit_shape_t;

typedef struct {
    uint32_t freq_hz;
    double freq_d;
    double gain;
    int32_t gain_x1000;
    bool phase_valid;
    double phase_deg;
} fit_work_point_t;

typedef struct {
    bool valid;
    uint8_t model_type;
    double score;
    double rms_db;
    double rms_rel;
    double max_rel;
    double phase_rms_deg;
    uint32_t phase_points;
    uint32_t valid_points;
    double freq_hz;
    double q;
    double k;
} fit_candidate_t;

static heavyfit_input_t *g_input_ptr = nullptr;
static heavyfit_output_t *g_result_ptr = nullptr;
static heavyfit_output_t *g_work_result_ptr = nullptr;
static fit_work_point_t *g_fit_points_ptr = nullptr;
static uint32_t g_fit_point_count = 0;
static uint32_t g_heavy_fit_sample_stride = 1;
static TaskHandle_t g_task = nullptr;
static volatile bool g_busy = false;
static volatile bool g_ready = false;
static volatile bool g_cancel = false;
static volatile bool g_result_copying = false;
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

#define g_input (*g_input_ptr)
#define g_result (*g_result_ptr)
#define g_work_result (*g_work_result_ptr)
#define g_fit_points (g_fit_points_ptr)

static bool ensure_heavyfit_buffers(void)
{
    if (g_input_ptr == nullptr) {
        g_input_ptr = static_cast<heavyfit_input_t *>(
            heap_caps_calloc(1, sizeof(heavyfit_input_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_result_ptr == nullptr) {
        g_result_ptr = static_cast<heavyfit_output_t *>(
            heap_caps_calloc(1, sizeof(heavyfit_output_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_work_result_ptr == nullptr) {
        g_work_result_ptr = static_cast<heavyfit_output_t *>(
            heap_caps_calloc(1, sizeof(heavyfit_output_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_fit_points_ptr == nullptr) {
        g_fit_points_ptr = static_cast<fit_work_point_t *>(
            heap_caps_calloc(kHeavyFitMaxSamples, sizeof(fit_work_point_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }

    if (g_input_ptr == nullptr ||
        g_result_ptr == nullptr ||
        g_work_result_ptr == nullptr ||
        g_fit_points_ptr == nullptr) {
        ESP_LOGE(TAG,
                 "PSRAM buffer alloc failed: in=%p result=%p work=%p points=%p psram=%u internal=%u dma=%u",
                 g_input_ptr,
                 g_result_ptr,
                 g_work_result_ptr,
                 g_fit_points_ptr,
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 heap_caps_get_free_size(MALLOC_CAP_DMA));
        return false;
    }

    return true;
}

static uint32_t now_ms(void)
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char *model_kind_text(uint8_t type)
{
    switch (type) {
    case MODEL_TYPE_LP1: return "LP1";
    case MODEL_TYPE_HP1: return "HP1";
    case MODEL_TYPE_LP2: return "LP2";
    case MODEL_TYPE_HP2: return "HP2";
    case MODEL_TYPE_BP2: return "BP2";
    case MODEL_TYPE_BS2: return "BS2";
    default: return "LP1";
    }
}

static uint8_t model_filter_type(uint8_t type)
{
    switch (type) {
    case MODEL_TYPE_LP1:
    case MODEL_TYPE_LP2:
        return FILTER_TYPE_LOW_PASS;
    case MODEL_TYPE_HP1:
    case MODEL_TYPE_HP2:
        return FILTER_TYPE_HIGH_PASS;
    case MODEL_TYPE_BP2:
        return FILTER_TYPE_BAND_PASS;
    case MODEL_TYPE_BS2:
        return FILTER_TYPE_BAND_STOP;
    default:
        return FILTER_TYPE_UNKNOWN;
    }
}

static double clamp_double(double value, double lo, double hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static uint32_t clamp_u32(uint32_t value, uint32_t lo, uint32_t hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static bool fit_point_basic_valid(const freq_point_t *p)
{
    return p != nullptr &&
           (p->flags & (FREQ_POINT_FLAG_MISSING | FREQ_POINT_FLAG_UNSTABLE)) == 0U &&
           p->freq_hz > 0U &&
           p->vin_mv >= 80 &&
           p->gain_x1000 > 0 &&
           p->gain_x1000 <= 3000;
}

static void sort_i32(int32_t *values, uint32_t count)
{
    for (uint32_t i = 1; i < count; ++i) {
        const int32_t v = values[i];
        int32_t j = static_cast<int32_t>(i) - 1;
        while (j >= 0 && values[j] > v) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = v;
    }
}

static int32_t median_i32(int32_t *values, uint32_t count)
{
    if (count == 0U) {
        return 0;
    }
    sort_i32(values, count);
    return values[count / 2U];
}

static fit_reject_reason_t fit_point_reject_reason_strict(uint32_t index)
{
    if (index >= g_input.point_count) {
        return FIT_REJECT_INDEX;
    }

    const freq_point_t *p = &g_input.points[index];
    if (p->freq_hz == 0U || (p->flags & FREQ_POINT_FLAG_MISSING) != 0U) {
        return FIT_REJECT_FREQ;
    }
    if (p->vin_mv < 80) {
        return FIT_REJECT_LOW_VIN;
    }
    if ((p->flags & FREQ_POINT_FLAG_UNSTABLE) != 0U ||
        p->gain_x1000 <= 0 ||
        p->gain_x1000 > 3000) {
        return FIT_REJECT_GAIN;
    }

    for (int32_t i = static_cast<int32_t>(index) - 1; i >= 0; --i) {
        const freq_point_t *prev = &g_input.points[i];
        if (!fit_point_basic_valid(prev)) {
            continue;
        }
        if (prev->freq_hz >= p->freq_hz) {
            return FIT_REJECT_NON_MONOTONIC;
        }
        break;
    }
    for (uint32_t i = index + 1U; i < g_input.point_count; ++i) {
        const freq_point_t *next = &g_input.points[i];
        if (!fit_point_basic_valid(next)) {
            continue;
        }
        if (next->freq_hz <= p->freq_hz) {
            return FIT_REJECT_NON_MONOTONIC;
        }
        break;
    }

    int32_t gains[6] = {};
    uint32_t gain_count = 0;
    bool have_before = false;
    bool have_after = false;
    for (int32_t i = static_cast<int32_t>(index) - 1; i >= 0 && gain_count < 3U; --i) {
        const freq_point_t *prev = &g_input.points[i];
        if (!fit_point_basic_valid(prev) || prev->freq_hz >= p->freq_hz) {
            continue;
        }
        gains[gain_count++] = prev->gain_x1000;
        have_before = true;
    }
    const uint32_t before_count = gain_count;
    for (uint32_t i = index + 1U; i < g_input.point_count && (gain_count - before_count) < 3U; ++i) {
        const freq_point_t *next = &g_input.points[i];
        if (!fit_point_basic_valid(next) || next->freq_hz <= p->freq_hz) {
            continue;
        }
        gains[gain_count++] = next->gain_x1000;
        have_after = true;
    }

    if (have_before && have_after && gain_count >= 2U) {
        const int32_t median_gain = median_i32(gains, gain_count);
        if (median_gain > 0) {
            const int64_t gain = p->gain_x1000;
            const int64_t median = median_gain;
            if (gain * 100LL < median * HEAVYFIT_OUTLIER_LOW_PERCENT ||
                gain * 100LL > median * HEAVYFIT_OUTLIER_HIGH_PERCENT) {
                return FIT_REJECT_OUTLIER;
            }
        }
    }

    return FIT_REJECT_NONE;
}

static bool fit_point_valid_strict(uint32_t index)
{
    return fit_point_reject_reason_strict(index) == FIT_REJECT_NONE;
}

static fit_debug_stats_t collect_fit_debug_stats(void)
{
    fit_debug_stats_t stats = {};
    stats.raw_point_count = g_input.point_count;
    for (uint32_t i = 0; i < g_input.point_count; ++i) {
        const fit_reject_reason_t reason = fit_point_reject_reason_strict(i);
        if (reason == FIT_REJECT_NONE) {
            ++stats.valid_point_count;
        } else {
            ++stats.rejected_point_count;
            if (reason == FIT_REJECT_OUTLIER) {
                ++stats.rejected_outlier_count;
            } else if (reason == FIT_REJECT_LOW_VIN) {
                ++stats.rejected_low_vin_count;
            } else if (reason == FIT_REJECT_NON_MONOTONIC) {
                ++stats.rejected_non_monotonic_count;
            }
        }
    }
    return stats;
}

static void build_fit_work_points(void)
{
    g_fit_point_count = 0;
    uint32_t valid_seq = 0;
    for (uint32_t i = 0; i < g_input.point_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }
        if ((valid_seq++ % g_heavy_fit_sample_stride) != 0U) {
            continue;
        }
        if (g_fit_point_count >= kHeavyFitMaxSamples) {
            break;
        }
        const freq_point_t *src = &g_input.points[i];
        fit_work_point_t *dst = &g_fit_points[g_fit_point_count++];
        dst->freq_hz = src->freq_hz;
        dst->freq_d = static_cast<double>(src->freq_hz);
        dst->gain_x1000 = src->gain_x1000;
        dst->gain = static_cast<double>(src->gain_x1000) / 1000.0;
        dst->phase_valid = src->phase_valid;
        dst->phase_deg = static_cast<double>(src->phase_deg_x10) / 10.0;
    }
}

static double gain_db_from_gain(double gain)
{
    if (gain < 0.000001) {
        gain = 0.000001;
    }
    return 20.0 * log10(gain);
}

static double wrap_phase_deg(double phase)
{
    while (phase > 180.0) {
        phase -= 360.0;
    }
    while (phase < -180.0) {
        phase += 360.0;
    }
    return phase;
}

static double model_gain_no_k(uint8_t model_type, double f_hz, double f0_hz, double q)
{
    if (f_hz <= 0.0 || f0_hz <= 0.0) {
        return 0.0;
    }
    const double r = f_hz / f0_hz;
    switch (model_type) {
    case MODEL_TYPE_LP1:
        return 1.0 / sqrt(1.0 + r * r);
    case MODEL_TYPE_HP1:
        return r / sqrt(1.0 + r * r);
    case MODEL_TYPE_LP2: {
        const double a = 1.0 - r * r;
        const double b = r / q;
        return 1.0 / sqrt(a * a + b * b);
    }
    case MODEL_TYPE_HP2: {
        const double a = 1.0 - r * r;
        const double b = r / q;
        return (r * r) / sqrt(a * a + b * b);
    }
    case MODEL_TYPE_BP2: {
        const double a = 1.0 - r * r;
        const double b = r / q;
        return (r / q) / sqrt(a * a + b * b);
    }
    case MODEL_TYPE_BS2: {
        const double a = 1.0 - r * r;
        const double b = r / q;
        return fabs(a) / sqrt(a * a + b * b);
    }
    default:
        return 0.0;
    }
}

static double model_phase_deg(uint8_t model_type, double f_hz, double f0_hz, double q)
{
    if (f_hz <= 0.0 || f0_hz <= 0.0) {
        return 0.0;
    }
    const double pi = 3.14159265358979323846;
    const double r = f_hz / f0_hz;
    switch (model_type) {
    case MODEL_TYPE_LP1:
        return -atan(r) * 180.0 / pi;
    case MODEL_TYPE_HP1:
        return 90.0 - atan(r) * 180.0 / pi;
    case MODEL_TYPE_LP2:
        return wrap_phase_deg(-atan2(r / q, 1.0 - r * r) * 180.0 / pi);
    case MODEL_TYPE_HP2:
        return wrap_phase_deg(180.0 - atan2(r / q, 1.0 - r * r) * 180.0 / pi);
    case MODEL_TYPE_BP2:
        return wrap_phase_deg(90.0 - atan2(r / q, 1.0 - r * r) * 180.0 / pi);
    case MODEL_TYPE_BS2:
        return wrap_phase_deg(-atan2(r / q, 1.0 - r * r) * 180.0 / pi);
    default:
        return 0.0;
    }
}

static fit_candidate_t invalid_candidate(uint8_t model_type)
{
    fit_candidate_t c = {};
    c.model_type = model_type;
    c.score = 1.0e9;
    c.rms_db = 1.0e9;
    c.rms_rel = 1.0e9;
    c.max_rel = 1.0e9;
    return c;
}

static fit_candidate_t evaluate_candidate(uint8_t model_type, double freq_hz, double q)
{
    fit_candidate_t c = invalid_candidate(model_type);
    c.freq_hz = freq_hz;
    c.q = q;

    double sum_mh = 0.0;
    double sum_hh = 0.0;
    uint32_t used = 0;
    for (uint32_t i = 0; i < g_fit_point_count; ++i) {
        const fit_work_point_t *p = &g_fit_points[i];
        const double h0 = model_gain_no_k(model_type, p->freq_d, freq_hz, q);
        if (h0 <= 0.000001) {
            continue;
        }
        sum_mh += p->gain * h0;
        sum_hh += h0 * h0;
        ++used;
    }
    if (used < 8U || sum_hh <= 0.000001) {
        return c;
    }

    const double k = clamp_double(sum_mh / sum_hh, 0.05, 5.0);
    double err_db_sum = 0.0;
    double rel_sum = 0.0;
    double max_rel = 0.0;
    double phase_sum = 0.0;
    double phase_inv_sum = 0.0;
    double phase_offset = 0.0;
    double phase_inv_offset = 0.0;
    bool have_phase_offset = false;
    uint32_t phase_points = 0;

    for (uint32_t i = 0; i < g_fit_point_count; ++i) {
        const fit_work_point_t *p = &g_fit_points[i];
        const double h0 = model_gain_no_k(model_type, p->freq_d, freq_hz, q);
        const double theory = k * h0;
        if (theory <= 0.000001) {
            continue;
        }
        const double err_db = gain_db_from_gain(p->gain) - gain_db_from_gain(theory);
        const double rel = fabs(p->gain - theory) / theory;
        err_db_sum += err_db * err_db;
        rel_sum += rel * rel;
        if (rel > max_rel) {
            max_rel = rel;
        }
        if (p->phase_valid) {
            const double theory_phase = model_phase_deg(model_type, p->freq_d, freq_hz, q);
            if (!have_phase_offset) {
                phase_offset = p->phase_deg - theory_phase;
                phase_inv_offset = p->phase_deg + theory_phase;
                have_phase_offset = true;
            }
            const double diff = wrap_phase_deg(p->phase_deg - (theory_phase + phase_offset));
            const double diff_inv = wrap_phase_deg(p->phase_deg - (-theory_phase + phase_inv_offset));
            phase_sum += diff * diff;
            phase_inv_sum += diff_inv * diff_inv;
            ++phase_points;
        }
    }

    c.valid = true;
    c.k = k;
    c.valid_points = used;
    c.phase_points = phase_points;
    c.rms_db = sqrt(err_db_sum / static_cast<double>(used));
    c.rms_rel = sqrt(rel_sum / static_cast<double>(used));
    c.max_rel = max_rel;
    if (phase_points >= 4U) {
        const double phase_rms = sqrt(phase_sum / static_cast<double>(phase_points));
        const double phase_inv_rms = sqrt(phase_inv_sum / static_cast<double>(phase_points));
        c.phase_rms_deg = (phase_inv_rms < phase_rms) ? phase_inv_rms : phase_rms;
        c.score = c.rms_db + 0.02 * c.phase_rms_deg;
    } else {
        c.score = c.rms_db;
    }
    return c;
}

static void keep_best_candidate(fit_candidate_t *best, const fit_candidate_t *cand)
{
    if (best != nullptr && cand != nullptr && cand->valid &&
        (!best->valid || cand->score < best->score)) {
        *best = *cand;
    }
}

static double slope_db_per_decade(uint32_t start, uint32_t stop)
{
    if (g_fit_point_count == 0U || start >= g_fit_point_count) {
        return 0.0;
    }
    if (stop >= g_fit_point_count) {
        stop = g_fit_point_count - 1U;
    }
    if (start >= stop) {
        return 0.0;
    }
    const fit_work_point_t *p0 = &g_fit_points[start];
    const fit_work_point_t *p1 = &g_fit_points[stop];
    if (p0->freq_d <= 0.0 || p1->freq_d <= p0->freq_d) {
        return 0.0;
    }
    return (gain_db_from_gain(p1->gain) - gain_db_from_gain(p0->gain)) /
           log10(p1->freq_d / p0->freq_d);
}

static int32_t confidence_from_candidate(const fit_candidate_t *best, const fit_candidate_t *second)
{
    if (best == nullptr || !best->valid) {
        return 0;
    }
    int32_t confidence = 1000;
    confidence -= static_cast<int32_t>(best->rms_rel * 3000.0 + 0.5);
    if (best->max_rel > 0.12) {
        confidence -= static_cast<int32_t>((best->max_rel - 0.12) * 1000.0 + 0.5);
    }
    if (best->valid_points < 12U) {
        confidence -= static_cast<int32_t>((12U - best->valid_points) * 35U);
    }
    if (second != nullptr && second->valid) {
        const double gap = second->score - best->score;
        if (gap < 0.20) {
            confidence -= 220;
        } else if (gap < 0.50) {
            confidence -= 120;
        } else if (gap < 1.00) {
            confidence -= 60;
        }
    }
    if (best->phase_points >= 4U && best->phase_rms_deg > 45.0) {
        confidence -= 120;
    }
    return clamp_i32(confidence, 0, 1000);
}

static bool same_filter_family(uint8_t a, uint8_t b)
{
    if ((a == MODEL_TYPE_LP1 || a == MODEL_TYPE_LP2) &&
        (b == MODEL_TYPE_LP1 || b == MODEL_TYPE_LP2)) {
        return true;
    }
    if ((a == MODEL_TYPE_HP1 || a == MODEL_TYPE_HP2) &&
        (b == MODEL_TYPE_HP1 || b == MODEL_TYPE_HP2)) {
        return true;
    }
    if (a == b) {
        return true;
    }
    return false;
}

static uint32_t interpolate_cutoff_hz(uint32_t f0, uint32_t f1, int32_t g0, int32_t g1, int32_t target)
{
    if (f1 <= f0 || g0 == g1) {
        return f0;
    }
    int32_t denom = g1 - g0;
    int32_t numer = target - g0;
    if (denom < 0) {
        denom = -denom;
        numer = -numer;
    }
    numer = clamp_i32(numer, 0, denom);
    return f0 + static_cast<uint32_t>(
        (static_cast<uint64_t>(f1 - f0) * static_cast<uint32_t>(numer)) /
        static_cast<uint32_t>(denom)
    );
}

static uint32_t find_light_cutoff_hz(uint8_t filter_type, uint32_t target_gain_x1000)
{
    bool have_prev = false;
    freq_point_t prev = {};
    for (uint32_t i = 0; i < g_input.point_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }
        const freq_point_t *cur = &g_input.points[i];
        if (have_prev) {
            if (filter_type == FILTER_TYPE_LOW_PASS &&
                prev.gain_x1000 >= static_cast<int32_t>(target_gain_x1000) &&
                cur->gain_x1000 <= static_cast<int32_t>(target_gain_x1000)) {
                return interpolate_cutoff_hz(prev.freq_hz, cur->freq_hz, prev.gain_x1000, cur->gain_x1000, target_gain_x1000);
            }
            if (filter_type == FILTER_TYPE_HIGH_PASS &&
                prev.gain_x1000 <= static_cast<int32_t>(target_gain_x1000) &&
                cur->gain_x1000 >= static_cast<int32_t>(target_gain_x1000)) {
                return interpolate_cutoff_hz(prev.freq_hz, cur->freq_hz, prev.gain_x1000, cur->gain_x1000, target_gain_x1000);
            }
        }
        prev = *cur;
        have_prev = true;
    }
    return 0U;
}

static void compute_band_edges(uint32_t f0_hz, double q, uint32_t *fl_hz, uint32_t *fh_hz)
{
    if (fl_hz == nullptr || fh_hz == nullptr) {
        return;
    }
    *fl_hz = 0U;
    *fh_hz = 0U;
    if (f0_hz == 0U || q < 0.05) {
        return;
    }
    const double inv_q = 1.0 / q;
    const double root = sqrt(inv_q * inv_q + 4.0);
    const double r_low = (root - inv_q) * 0.5;
    const double r_high = (root + inv_q) * 0.5;
    *fl_hz = static_cast<uint32_t>(static_cast<double>(f0_hz) * r_low + 0.5);
    *fh_hz = static_cast<uint32_t>(static_cast<double>(f0_hz) * r_high + 0.5);
}

static uint32_t fallback_mid_freq_hz(uint32_t min_freq, uint32_t max_freq)
{
    if (min_freq != 0U && max_freq > min_freq) {
        return static_cast<uint32_t>(sqrt(static_cast<double>(min_freq) * static_cast<double>(max_freq)) + 0.5);
    }
    if (max_freq != 0U) {
        return max_freq;
    }
    if (min_freq != 0U) {
        return min_freq;
    }
    return 1000U;
}

static uint8_t forced_model_from_shape(const fit_shape_t *shape, const filter_fit_result_t *light_fit)
{
    if (light_fit != nullptr && light_fit->valid && light_fit->model_type != MODEL_TYPE_UNKNOWN) {
        return light_fit->model_type;
    }
    if (shape == nullptr) {
        return MODEL_TYPE_LP1;
    }
    if ((shape->candidate_mask & MODEL_MASK_BP2) != 0U) {
        return MODEL_TYPE_BP2;
    }
    if ((shape->candidate_mask & MODEL_MASK_BS2) != 0U) {
        return MODEL_TYPE_BS2;
    }
    if (shape->low_avg > shape->high_avg) {
        return MODEL_TYPE_LP1;
    }
    if (shape->high_avg > shape->low_avg) {
        return MODEL_TYPE_HP1;
    }
    if ((shape->candidate_mask & MODEL_MASK_LP2) != 0U) {
        return MODEL_TYPE_LP2;
    }
    if ((shape->candidate_mask & MODEL_MASK_HP2) != 0U) {
        return MODEL_TYPE_HP2;
    }
    return MODEL_TYPE_LP1;
}

static uint32_t forced_freq_from_shape(uint8_t model_type,
                                       const fit_shape_t *shape,
                                       const filter_fit_result_t *light_fit,
                                       uint32_t min_freq,
                                       uint32_t max_freq)
{
    if (light_fit != nullptr && light_fit->valid) {
        const uint32_t f = (light_fit->f0_hz != 0U) ? light_fit->f0_hz : light_fit->fc_hz;
        if (f != 0U) {
            return f;
        }
    }
    if (shape != nullptr) {
        if (model_type == MODEL_TYPE_BP2 && shape->peak_freq != 0U) {
            return shape->peak_freq;
        }
        if (model_type == MODEL_TYPE_BS2 && shape->valley_freq != 0U) {
            return shape->valley_freq;
        }
    }
    return fallback_mid_freq_hz(min_freq, max_freq);
}

static void force_non_unknown_fit(heavyfit_output_t *out,
                                  uint8_t model_type,
                                  uint32_t freq_hz,
                                  uint32_t valid_point_count,
                                  int32_t confidence_x1000)
{
    if (out == nullptr) {
        return;
    }
    if (model_type == MODEL_TYPE_UNKNOWN || model_type > MODEL_TYPE_BS2) {
        model_type = MODEL_TYPE_LP1;
    }
    if (freq_hz == 0U) {
        freq_hz = 1000U;
    }

    filter_fit_result_t fit = {};
    fit.valid = true;
    fit.model_type = model_type;
    fit.fc_hz = freq_hz;
    fit.f0_hz = freq_hz;
    fit.q_x1000 = 707;
    fit.k_x1000 = 1000;
    fit.rms_error_x10 = 999;
    fit.max_error_x10 = 999;
    fit.confidence_x1000 = clamp_i32(confidence_x1000, 100, 500);
    fit.valid_point_count = valid_point_count;
    if (model_type == MODEL_TYPE_BP2 || model_type == MODEL_TYPE_BS2) {
        compute_band_edges(fit.f0_hz, 0.707, &fit.fl_hz, &fit.fh_hz);
    }

    out->fit = fit;
    out->quality = HEAVYFIT_RESULT_HEAVY_LOW_CONF;
    out->status.filter_type = model_filter_type(model_type);
    out->status.cutoff_freq_hz = fit.fc_hz;
}

static fit_shape_t classify_shape(void)
{
    fit_shape_t shape = {};
    shape.valid_count = g_fit_point_count;
    if (shape.valid_count == 0U) {
        return shape;
    }
    const uint32_t edge_count = clamp_u32(shape.valid_count / 10U, 3U, 16U);
    uint64_t low_sum = 0;
    uint64_t high_sum = 0;
    uint32_t low_count = 0;
    uint32_t high_count = 0;
    uint32_t peak_pos = 0;
    uint32_t valley_pos = 0;
    uint64_t first_sum = 0;
    uint64_t mid_sum = 0;
    uint64_t last_sum = 0;
    uint32_t first_count = 0;
    uint32_t mid_count = 0;
    uint32_t last_count = 0;
    shape.valley_gain = UINT32_MAX;
    for (uint32_t i = 0; i < g_fit_point_count; ++i) {
        const uint32_t gain = static_cast<uint32_t>(g_fit_points[i].gain_x1000);
        if (i < shape.valid_count / 3U) {
            first_sum += gain;
            ++first_count;
        } else if (i < (shape.valid_count * 2U) / 3U) {
            mid_sum += gain;
            ++mid_count;
        } else {
            last_sum += gain;
            ++last_count;
        }
        if (low_count < edge_count) {
            low_sum += gain;
            ++low_count;
        }
        if (gain > shape.peak_gain) {
            shape.peak_gain = gain;
            shape.peak_freq = g_fit_points[i].freq_hz;
            peak_pos = i;
        }
        if (gain < shape.valley_gain) {
            shape.valley_gain = gain;
            shape.valley_freq = g_fit_points[i].freq_hz;
            valley_pos = i;
        }
    }
    for (int32_t i = static_cast<int32_t>(g_fit_point_count) - 1; i >= 0 && high_count < edge_count; --i) {
        high_sum += static_cast<uint32_t>(g_fit_points[i].gain_x1000);
        ++high_count;
    }
    shape.low_avg = (low_count == 0U) ? 0U : static_cast<uint32_t>(low_sum / low_count);
    shape.high_avg = (high_count == 0U) ? 0U : static_cast<uint32_t>(high_sum / high_count);
    if (shape.valley_gain == UINT32_MAX) {
        shape.valley_gain = 0U;
    }
    const uint32_t first_avg = (first_count == 0U) ? 0U : static_cast<uint32_t>(first_sum / first_count);
    const uint32_t mid_avg = (mid_count == 0U) ? 0U : static_cast<uint32_t>(mid_sum / mid_count);
    const uint32_t last_avg = (last_count == 0U) ? 0U : static_cast<uint32_t>(last_sum / last_count);
    const uint32_t edge_mid_hi = (first_avg > last_avg) ? first_avg : last_avg;
    const uint32_t edge_mid_lo = (first_avg < last_avg) ? first_avg : last_avg;
    const uint32_t edge_hi = (shape.low_avg > shape.high_avg) ? shape.low_avg : shape.high_avg;
    const uint32_t edge_lo = (shape.low_avg < shape.high_avg) ? shape.low_avg : shape.high_avg;
    const bool peak_in_middle =
        peak_pos > (shape.valid_count / 5U) && peak_pos < ((shape.valid_count * 4U) / 5U);
    const bool valley_in_middle =
        valley_pos > (shape.valid_count / 5U) && valley_pos < ((shape.valid_count * 4U) / 5U);
    const bool edge_prominent_peak =
        edge_hi != 0U &&
        static_cast<uint64_t>(shape.peak_gain) * 100ULL >
            static_cast<uint64_t>(edge_hi) * HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT;
    const bool edge_prominent_valley =
        edge_lo != 0U &&
        static_cast<uint64_t>(shape.valley_gain) * HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT <
            static_cast<uint64_t>(edge_lo) * 100ULL;
    const bool mid_prominent_peak =
        first_count >= 3U && mid_count >= 3U && last_count >= 3U &&
        edge_mid_hi != 0U &&
        static_cast<uint64_t>(mid_avg) * 100ULL >
            static_cast<uint64_t>(edge_mid_hi) * HEAVYFIT_BAND_MID_PROMINENCE_PERCENT;
    const bool mid_prominent_valley =
        first_count >= 3U && mid_count >= 3U && last_count >= 3U &&
        edge_mid_lo != 0U &&
        static_cast<uint64_t>(mid_avg) * HEAVYFIT_BAND_MID_PROMINENCE_PERCENT <
            static_cast<uint64_t>(edge_mid_lo) * 100ULL;
    shape.low_high_low = peak_in_middle && (edge_prominent_peak || mid_prominent_peak);
    shape.high_low_high = valley_in_middle && (edge_prominent_valley || mid_prominent_valley);
    const bool low_edge_low_for_bp =
        shape.peak_gain != 0U &&
        static_cast<uint64_t>(shape.low_avg) * 100ULL <
            static_cast<uint64_t>(shape.peak_gain) * HEAVYFIT_BANDPASS_EDGE_MAX_PERCENT;
    const bool high_edge_low_for_bp =
        shape.peak_gain != 0U &&
        static_cast<uint64_t>(shape.high_avg) * 100ULL <
            static_cast<uint64_t>(shape.peak_gain) * HEAVYFIT_BANDPASS_EDGE_MAX_PERCENT;
    shape.true_band_pass = shape.low_high_low && low_edge_low_for_bp && high_edge_low_for_bp;
    shape.true_band_stop =
        shape.high_low_high &&
        shape.valley_gain != 0U &&
        static_cast<uint64_t>(shape.low_avg) * 100ULL >
            static_cast<uint64_t>(shape.valley_gain) * HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT &&
        static_cast<uint64_t>(shape.high_avg) * 100ULL >
            static_cast<uint64_t>(shape.valley_gain) * HEAVYFIT_BAND_EDGE_PROMINENCE_PERCENT;
    shape.resonant_low_pass =
        shape.low_high_low && !low_edge_low_for_bp &&
        static_cast<uint64_t>(shape.low_avg) * 100ULL >
            static_cast<uint64_t>(shape.high_avg) * 115ULL;

    if (static_cast<uint64_t>(shape.low_avg) * 100ULL > static_cast<uint64_t>(shape.high_avg) * 125ULL) {
        shape.candidate_mask |= MODEL_MASK_LP;
    }
    if (static_cast<uint64_t>(shape.high_avg) * 100ULL > static_cast<uint64_t>(shape.low_avg) * 125ULL) {
        shape.candidate_mask |= MODEL_MASK_HP;
    }
    if (shape.resonant_low_pass) {
        shape.candidate_mask |= MODEL_MASK_LP2;
    }
    if (shape.true_band_pass) {
        shape.candidate_mask |= MODEL_MASK_BP2;
    }
    if (shape.true_band_stop) {
        shape.candidate_mask |= MODEL_MASK_BS2;
    }
    if (first_count >= 3U && mid_count >= 3U && last_count >= 3U) {
        if (mid_prominent_peak && shape.true_band_pass) {
            shape.candidate_mask |= MODEL_MASK_BP2;
        }
        if (mid_prominent_valley && shape.true_band_stop) {
            shape.candidate_mask |= MODEL_MASK_BS2;
        }
    }
    return shape;
}

static void clear_theory_columns(heavyfit_output_t *out)
{
    for (uint32_t i = 0; i < out->point_count; ++i) {
        out->theory_gain_x1000[i] = 0;
        out->error_x10[i] = 0;
    }
}

static void fill_theory_columns(heavyfit_output_t *out, const fit_candidate_t *best)
{
    clear_theory_columns(out);
    if (best == nullptr || !best->valid) {
        return;
    }
    for (uint32_t i = 0; i < g_input.point_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }
        const double h0 = model_gain_no_k(best->model_type, static_cast<double>(g_input.points[i].freq_hz), best->freq_hz, best->q);
        const double theory = best->k * h0;
        if (theory <= 0.000001) {
            continue;
        }
        out->theory_gain_x1000[i] = static_cast<int32_t>(theory * 1000.0 + 0.5);
        const double measured = static_cast<double>(g_input.points[i].gain_x1000) / 1000.0;
        out->error_x10[i] = static_cast<int32_t>((fabs(measured - theory) / theory) * 1000.0 + 0.5);
    }
}

static void analyze_sweep_response_light(heavyfit_output_t *out)
{
    clear_theory_columns(out);
    out->fit = {};
    out->quality = HEAVYFIT_RESULT_NONE;
    out->status.filter_type = FILTER_TYPE_UNKNOWN;
    out->status.cutoff_freq_hz = 0;

    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < g_input.point_count; ++i) {
        if (fit_point_valid_strict(i)) {
            ++valid_count;
        }
    }
    if (valid_count < 8U) {
        return;
    }

    const uint32_t edge_count = clamp_u32(valid_count / 10U, 3U, 16U);
    uint64_t low_sum = 0;
    uint64_t high_sum = 0;
    uint32_t low_count = 0;
    uint32_t high_count = 0;
    for (uint32_t i = 0; i < g_input.point_count && low_count < edge_count; ++i) {
        if (fit_point_valid_strict(i)) {
            low_sum += static_cast<uint32_t>(g_input.points[i].gain_x1000);
            ++low_count;
        }
    }
    for (int32_t i = static_cast<int32_t>(g_input.point_count) - 1; i >= 0 && high_count < edge_count; --i) {
        if (fit_point_valid_strict(static_cast<uint32_t>(i))) {
            high_sum += static_cast<uint32_t>(g_input.points[i].gain_x1000);
            ++high_count;
        }
    }
    if (low_count == 0U || high_count == 0U) {
        return;
    }

    const uint32_t low_avg = static_cast<uint32_t>(low_sum / low_count);
    const uint32_t high_avg = static_cast<uint32_t>(high_sum / high_count);
    uint8_t filter_type = FILTER_TYPE_UNKNOWN;
    uint8_t model_type = MODEL_TYPE_UNKNOWN;
    uint32_t cutoff_hz = 0;
    if (static_cast<uint64_t>(low_avg) * 100ULL > static_cast<uint64_t>(high_avg) * 125ULL) {
        filter_type = FILTER_TYPE_LOW_PASS;
        model_type = MODEL_TYPE_LP1;
        cutoff_hz = find_light_cutoff_hz(filter_type, static_cast<uint32_t>((static_cast<uint64_t>(low_avg) * 707ULL) / 1000ULL));
    } else if (static_cast<uint64_t>(high_avg) * 100ULL > static_cast<uint64_t>(low_avg) * 125ULL) {
        filter_type = FILTER_TYPE_HIGH_PASS;
        model_type = MODEL_TYPE_HP1;
        cutoff_hz = find_light_cutoff_hz(filter_type, static_cast<uint32_t>((static_cast<uint64_t>(high_avg) * 707ULL) / 1000ULL));
    }
    if (filter_type == FILTER_TYPE_UNKNOWN || cutoff_hz == 0U) {
        return;
    }

    const uint32_t edge_hi = (low_avg > high_avg) ? low_avg : high_avg;
    uint32_t edge_lo = (low_avg < high_avg) ? low_avg : high_avg;
    if (edge_lo == 0U) {
        edge_lo = 1U;
    }
    filter_fit_result_t fit = {};
    fit.valid = true;
    fit.model_type = model_type;
    fit.fc_hz = cutoff_hz;
    fit.f0_hz = cutoff_hz;
    fit.q_x1000 = 707;
    fit.k_x1000 = 1000;
    fit.confidence_x1000 = static_cast<int32_t>(
        clamp_u32(static_cast<uint32_t>(((edge_hi - edge_lo) * 1000ULL) / edge_lo), 0U, 800U)
    );
    fit.valid_point_count = valid_count;
    out->fit = fit;
    out->quality = HEAVYFIT_RESULT_LIGHT;
    out->status.filter_type = filter_type;
    out->status.cutoff_freq_hz = cutoff_hz;
}

static void analyze_sweep_response(heavyfit_output_t *out)
{
    const uint32_t analyze_start = now_ms();
    analyze_sweep_response_light(out);
    const filter_fit_result_t light_fit = out->fit;
    const uint8_t light_filter_type = out->status.filter_type;
    const uint32_t light_cutoff_hz = out->status.cutoff_freq_hz;

    fit_debug_stats_t stats = collect_fit_debug_stats();
    g_heavy_fit_sample_stride =
        (stats.valid_point_count > kHeavyFitMaxSamples) ?
        ((stats.valid_point_count + kHeavyFitMaxSamples - 1U) / kHeavyFitMaxSamples) : 1U;
    build_fit_work_points();
    fit_shape_t shape = classify_shape();
    stats.candidate_mask = shape.candidate_mask;
    ESP_LOGI(TAG,
             "FIT shape: low=%lu high=%lu peak=%lu@%lu valley=%lu@%lu low_high_low=%u high_low_high=%u bp=%u bs=%u res_lp2=%u mask=0x%02lx outlier=%d..%d%% band_pref=%d%%",
             static_cast<unsigned long>(shape.low_avg),
             static_cast<unsigned long>(shape.high_avg),
             static_cast<unsigned long>(shape.peak_gain),
             static_cast<unsigned long>(shape.peak_freq),
             static_cast<unsigned long>(shape.valley_gain),
             static_cast<unsigned long>(shape.valley_freq),
             shape.low_high_low ? 1U : 0U,
             shape.high_low_high ? 1U : 0U,
             shape.true_band_pass ? 1U : 0U,
             shape.true_band_stop ? 1U : 0U,
             shape.resonant_low_pass ? 1U : 0U,
             static_cast<unsigned long>(shape.candidate_mask),
             HEAVYFIT_OUTLIER_LOW_PERCENT,
             HEAVYFIT_OUTLIER_HIGH_PERCENT,
             HEAVYFIT_BAND_PREFER_RMS_PERCENT);

    uint32_t min_freq = 0;
    uint32_t max_freq = 0;
    if (g_fit_point_count != 0U) {
        min_freq = g_fit_points[0].freq_hz;
        max_freq = g_fit_points[g_fit_point_count - 1U].freq_hz;
    }
    if (g_fit_point_count < 8U || min_freq == 0U || max_freq <= min_freq || shape.candidate_mask == 0U) {
        ESP_LOGW(TAG, "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu candidate_mask=0x%02lx fit_points=%lu selected=%s elapsed_ms=%lu reason=not_enough_shape",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.rejected_low_vin_count),
                 static_cast<unsigned long>(stats.rejected_non_monotonic_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(out->fit.model_type),
                 static_cast<unsigned long>(now_ms() - analyze_start));
        const uint8_t forced_model = forced_model_from_shape(&shape, &light_fit);
        force_non_unknown_fit(out,
                              forced_model,
                              forced_freq_from_shape(forced_model, &shape, &light_fit, min_freq, max_freq),
                              stats.valid_point_count,
                              200);
        return;
    }

    fit_candidate_t best_by_model[MODEL_TYPE_BS2 + 1];
    for (uint32_t i = 0; i <= MODEL_TYPE_BS2; ++i) {
        best_by_model[i] = invalid_candidate(static_cast<uint8_t>(i));
    }
    if ((shape.candidate_mask & MODEL_MASK_LP1) != 0U && light_fit.valid && light_fit.model_type == MODEL_TYPE_LP1) {
        fit_candidate_t lp1 = evaluate_candidate(MODEL_TYPE_LP1, static_cast<double>(light_fit.fc_hz), 0.707);
        keep_best_candidate(&best_by_model[MODEL_TYPE_LP1], &lp1);
    }
    if ((shape.candidate_mask & MODEL_MASK_HP1) != 0U && light_fit.valid && light_fit.model_type == MODEL_TYPE_HP1) {
        fit_candidate_t hp1 = evaluate_candidate(MODEL_TYPE_HP1, static_cast<double>(light_fit.fc_hz), 0.707);
        keep_best_candidate(&best_by_model[MODEL_TYPE_HP1], &hp1);
    }

    const uint32_t fit_start = now_ms();
    const double log_min = log10(static_cast<double>(min_freq));
    const double log_max = log10(static_cast<double>(max_freq));
    static const double q_values[] = {0.50, 0.707, 1.00, 1.60, 2.50, 4.00};
    bool timed_out = false;

    for (uint32_t n = 0; n < kHeavyFitGridCount && !timed_out && !g_cancel; ++n) {
        if ((now_ms() - fit_start) > kHeavyFitTimeBudgetMs) {
            timed_out = true;
            break;
        }
        const double t = (kHeavyFitGridCount <= 1U) ? 0.0 : (static_cast<double>(n) / static_cast<double>(kHeavyFitGridCount - 1U));
        const double freq = pow(10.0, log_min + (log_max - log_min) * t);
        for (uint32_t qi = 0; qi < sizeof(q_values) / sizeof(q_values[0]); ++qi) {
            if ((now_ms() - fit_start) > kHeavyFitTimeBudgetMs) {
                timed_out = true;
                break;
            }
            if ((shape.candidate_mask & MODEL_MASK_LP2) != 0U) {
                fit_candidate_t c = evaluate_candidate(MODEL_TYPE_LP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_LP2], &c);
            }
            if ((shape.candidate_mask & MODEL_MASK_HP2) != 0U) {
                fit_candidate_t c = evaluate_candidate(MODEL_TYPE_HP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_HP2], &c);
            }
            if ((shape.candidate_mask & MODEL_MASK_BP2) != 0U) {
                fit_candidate_t c = evaluate_candidate(MODEL_TYPE_BP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_BP2], &c);
            }
            if ((shape.candidate_mask & MODEL_MASK_BS2) != 0U) {
                fit_candidate_t c = evaluate_candidate(MODEL_TYPE_BS2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_BS2], &c);
            }
        }
    }

    if (g_cancel) {
        out->canceled = true;
        return;
    }
    if (timed_out) {
        out->fit = light_fit;
        if (out->fit.valid && out->fit.confidence_x1000 > 600) {
            out->fit.confidence_x1000 = 600;
        }
        out->quality = HEAVYFIT_RESULT_HEAVY_TIMEOUT;
        out->status.filter_type = light_filter_type;
        out->status.cutoff_freq_hz = light_cutoff_hz;
        if (!out->fit.valid || out->fit.model_type == MODEL_TYPE_UNKNOWN) {
            const uint8_t forced_model = forced_model_from_shape(&shape, &light_fit);
            force_non_unknown_fit(out,
                                  forced_model,
                                  forced_freq_from_shape(forced_model, &shape, &light_fit, min_freq, max_freq),
                                  stats.valid_point_count,
                                  300);
        }
        ESP_LOGW(TAG, "FIT summary: raw=%lu valid=%lu rejected=%lu candidate_mask=0x%02lx fit_points=%lu selected=%s elapsed_ms=%lu reason=timeout",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(out->fit.model_type),
                 static_cast<unsigned long>(now_ms() - analyze_start));
        return;
    }

    fit_candidate_t best = invalid_candidate(MODEL_TYPE_UNKNOWN);
    fit_candidate_t second = invalid_candidate(MODEL_TYPE_UNKNOWN);
    for (uint32_t i = MODEL_TYPE_LP1; i <= MODEL_TYPE_BS2; ++i) {
        const fit_candidate_t *cand = &best_by_model[i];
        if (!cand->valid) {
            continue;
        }
        if (!best.valid || cand->score < best.score) {
            second = best;
            best = *cand;
        } else if (!second.valid || cand->score < second.score) {
            second = *cand;
        }
    }

    const double high_slope = slope_db_per_decade((g_fit_point_count * 2U) / 3U, (g_fit_point_count == 0U) ? 0U : (g_fit_point_count - 1U));
    const double low_slope = slope_db_per_decade(0, g_fit_point_count / 3U);
    const bool lp2_much_better = best_by_model[MODEL_TYPE_LP1].valid && best_by_model[MODEL_TYPE_LP2].valid &&
                                 (best_by_model[MODEL_TYPE_LP1].score - best_by_model[MODEL_TYPE_LP2].score) > 0.35;
    const bool hp2_much_better = best_by_model[MODEL_TYPE_HP1].valid && best_by_model[MODEL_TYPE_HP2].valid &&
                                 (best_by_model[MODEL_TYPE_HP1].score - best_by_model[MODEL_TYPE_HP2].score) > 0.35;
    if (best.model_type == MODEL_TYPE_LP2 && best_by_model[MODEL_TYPE_LP1].valid &&
        (!((high_slope < -30.0) ||
           (best_by_model[MODEL_TYPE_LP2].q > 0.90) ||
           shape.resonant_low_pass) ||
         !lp2_much_better)) {
        best = best_by_model[MODEL_TYPE_LP1];
    } else if (best.model_type == MODEL_TYPE_HP2 && best_by_model[MODEL_TYPE_HP1].valid &&
               (!((low_slope > 30.0) || (best_by_model[MODEL_TYPE_HP2].q > 0.90)) || !hp2_much_better)) {
        best = best_by_model[MODEL_TYPE_HP1];
    }

    if (shape.resonant_low_pass && best_by_model[MODEL_TYPE_LP2].valid &&
        (!best.valid ||
         best_by_model[MODEL_TYPE_LP2].rms_rel * 100.0 <=
             best.rms_rel * static_cast<double>(HEAVYFIT_RESONANT_LP2_PREFER_RMS_PERCENT))) {
        if (best.valid && best.model_type != MODEL_TYPE_LP2) {
            second = best;
        }
        best = best_by_model[MODEL_TYPE_LP2];
    }

#if HEAVYFIT_ENABLE_BAND_SHAPE_PRIOR
    if (shape.true_band_pass && best_by_model[MODEL_TYPE_BP2].valid &&
        (!best.valid ||
         best_by_model[MODEL_TYPE_BP2].rms_rel * 100.0 <=
             best.rms_rel * static_cast<double>(HEAVYFIT_BAND_PREFER_RMS_PERCENT))) {
        if (best.valid && best.model_type != MODEL_TYPE_BP2) {
            second = best;
        }
        best = best_by_model[MODEL_TYPE_BP2];
    }
    if (shape.true_band_stop && best_by_model[MODEL_TYPE_BS2].valid &&
        (!best.valid ||
         best_by_model[MODEL_TYPE_BS2].rms_rel * 100.0 <=
             best.rms_rel * static_cast<double>(HEAVYFIT_BAND_PREFER_RMS_PERCENT))) {
        if (best.valid && best.model_type != MODEL_TYPE_BS2) {
            second = best;
        }
        best = best_by_model[MODEL_TYPE_BS2];
    }
#endif

    int32_t confidence = confidence_from_candidate(&best, &second);
    const bool same_family_ambiguous =
        best.valid && second.valid && same_filter_family(best.model_type, second.model_type);
    const bool family_only =
        (best.valid && (shape.candidate_mask & ~MODEL_MASK_LP) == 0U &&
         (best.model_type == MODEL_TYPE_LP1 || best.model_type == MODEL_TYPE_LP2)) ||
        (best.valid && (shape.candidate_mask & ~MODEL_MASK_HP) == 0U &&
         (best.model_type == MODEL_TYPE_HP1 || best.model_type == MODEL_TYPE_HP2));
    if ((same_family_ambiguous || family_only) && confidence < 500 && best.rms_rel <= 0.250) {
        confidence = 500;
    }
    const bool band_shape =
        best.valid &&
        (best.model_type == MODEL_TYPE_BP2 || best.model_type == MODEL_TYPE_BS2) &&
        ((shape.candidate_mask & (MODEL_MASK_BP2 | MODEL_MASK_BS2)) != 0U);
    if (band_shape && confidence < 500 && best.rms_rel <= 0.400) {
        confidence = 500;
    }
    if (best.valid && confidence < 450 &&
        ((shape.candidate_mask != 0U && best.rms_rel <= 0.320) || best.rms_rel <= 0.260)) {
        confidence = 450;
    }
    const double rms_limit = band_shape ? 0.400 : (family_only ? 0.300 : 0.280);
    const bool fit_ok = best.valid && best.rms_rel <= rms_limit && confidence >= 420;
    if (!fit_ok) {
        clear_theory_columns(out);
        const uint8_t forced_model = best.valid ?
            best.model_type : forced_model_from_shape(&shape, &light_fit);
        const uint32_t forced_freq = best.valid ?
            static_cast<uint32_t>(best.freq_hz + 0.5) :
            forced_freq_from_shape(forced_model, &shape, &light_fit, min_freq, max_freq);
        force_non_unknown_fit(out,
                              forced_model,
                              forced_freq,
                              stats.valid_point_count,
                              clamp_i32(confidence, 100, 500));
        ESP_LOGW(TAG, "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu candidate_mask=0x%02lx fit_points=%lu selected=%s forced=1 confidence=%ld elapsed_ms=%lu best=%s second=%s",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(out->fit.model_type),
                 static_cast<long>(clamp_i32(confidence, 0, 500)),
                 static_cast<unsigned long>(now_ms() - analyze_start),
                 model_kind_text(best.model_type),
                 model_kind_text(second.model_type));
        return;
    }

    fill_theory_columns(out, &best);
    filter_fit_result_t fit = {};
    fit.valid = true;
    fit.model_type = best.model_type;
    fit.fc_hz = static_cast<uint32_t>(best.freq_hz + 0.5);
    fit.f0_hz = fit.fc_hz;
    if (fit.model_type == MODEL_TYPE_BP2 || fit.model_type == MODEL_TYPE_BS2) {
        compute_band_edges(fit.f0_hz, best.q, &fit.fl_hz, &fit.fh_hz);
    }
    fit.q_x1000 = static_cast<int32_t>(best.q * 1000.0 + 0.5);
    fit.k_x1000 = static_cast<int32_t>(best.k * 1000.0 + 0.5);
    fit.rms_error_x10 = static_cast<int32_t>(best.rms_rel * 1000.0 + 0.5);
    fit.max_error_x10 = static_cast<int32_t>(best.max_rel * 1000.0 + 0.5);
    fit.confidence_x1000 = confidence;
    fit.valid_point_count = stats.valid_point_count;
    out->fit = fit;
    out->quality = HEAVYFIT_RESULT_HEAVY_OK;
    out->status.filter_type = model_filter_type(best.model_type);
    out->status.cutoff_freq_hz = fit.fc_hz;
    if (out->point_count != 0U) {
        out->status.theory_gain_x1000 = out->theory_gain_x1000[out->point_count - 1U];
        out->status.error_x10 = out->error_x10[out->point_count - 1U];
    }

    ESP_LOGI(TAG, "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu candidate_mask=0x%02lx fit_points=%lu selected=%s fc_hz=%lu confidence=%ld elapsed_ms=%lu second=%s K=%.3f Q=%.3f rms=%.2f%% max=%.2f%%",
             static_cast<unsigned long>(stats.raw_point_count),
             static_cast<unsigned long>(stats.valid_point_count),
             static_cast<unsigned long>(stats.rejected_point_count),
             static_cast<unsigned long>(stats.rejected_outlier_count),
             static_cast<unsigned long>(stats.rejected_low_vin_count),
             static_cast<unsigned long>(stats.rejected_non_monotonic_count),
             static_cast<unsigned long>(stats.candidate_mask),
             static_cast<unsigned long>(g_fit_point_count),
             model_kind_text(best.model_type),
             static_cast<unsigned long>(fit.fc_hz),
             static_cast<long>(confidence),
             static_cast<unsigned long>(now_ms() - analyze_start),
             model_kind_text(second.model_type),
             best.k,
             best.q,
             best.rms_rel * 100.0,
             best.max_rel * 100.0);
}

static void HeavyFit_Task(void *arg)
{
    (void)arg;
    memset(&g_work_result, 0, sizeof(g_work_result));
    g_work_result.valid = true;
    g_work_result.status = g_input.status;
    g_work_result.point_count = g_input.point_count;
    analyze_sweep_response(&g_work_result);

    portENTER_CRITICAL(&g_lock);
    if (g_cancel) {
        g_work_result.canceled = true;
        g_work_result.valid = false;
    }
    portEXIT_CRITICAL(&g_lock);

    g_result = g_work_result;

    portENTER_CRITICAL(&g_lock);
    g_ready = true;
    g_busy = false;
    g_task = nullptr;
    portEXIT_CRITICAL(&g_lock);

    vTaskDelete(nullptr);
}

}  // namespace

bool HeavyFit_StartAsync(const heavyfit_input_t *input)
{
    if (input == nullptr || input->point_count == 0U || input->point_count > MAX_POINTS) {
        return false;
    }
    if (!ensure_heavyfit_buffers()) {
        return false;
    }

    bool can_start = false;
    portENTER_CRITICAL(&g_lock);
    if (!g_busy && !g_ready && !g_result_copying) {
        g_busy = true;
        g_ready = false;
        g_cancel = false;
        can_start = true;
    }
    portEXIT_CRITICAL(&g_lock);

    if (!can_start) {
        return false;
    }

    g_input = *input;
    memset(&g_result, 0, sizeof(g_result));

    if (xTaskCreate(HeavyFit_Task, "heavyfit", 8192, nullptr, tskIDLE_PRIORITY + 1, &g_task) != pdPASS) {
        portENTER_CRITICAL(&g_lock);
        g_busy = false;
        g_task = nullptr;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "Failed to create heavy fit task");
        return false;
    }
    return true;
}

bool HeavyFit_PollResult(heavyfit_output_t *out)
{
    if (out == nullptr || g_result_ptr == nullptr) {
        return false;
    }

    bool have_result = false;
    portENTER_CRITICAL(&g_lock);
    if (g_ready) {
        g_ready = false;
        g_result_copying = true;
        have_result = true;
    }
    portEXIT_CRITICAL(&g_lock);

    if (!have_result) {
        return false;
    }

    *out = g_result;

    portENTER_CRITICAL(&g_lock);
    g_result_copying = false;
    portEXIT_CRITICAL(&g_lock);

    return have_result;
}

bool HeavyFit_IsBusy(void)
{
    return g_busy;
}

void HeavyFit_Cancel(void)
{
    portENTER_CRITICAL(&g_lock);
    g_cancel = true;
    g_ready = false;
    portEXIT_CRITICAL(&g_lock);
}
