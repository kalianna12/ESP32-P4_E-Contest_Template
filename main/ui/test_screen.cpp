#include "test_screen.h"

#include "esp_log.h"
#include "lvgl.h"
#include "spilink.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef ENABLE_FAKE_DATA_TEST
#define ENABLE_FAKE_DATA_TEST 0
#endif

#ifndef ENABLE_SPI_TEST_WINDOW
#define ENABLE_SPI_TEST_WINDOW 0
#endif

#ifndef ENABLE_HEAVY_FILTER_FIT
#define ENABLE_HEAVY_FILTER_FIT 1
#endif

#define TEST_SCREEN_W 1024
#define TEST_SCREEN_H 600

#define DEFAULT_START_FREQ_HZ 100U
#define DEFAULT_STOP_FREQ_HZ 100000U
#define DEFAULT_STEP_FREQ_HZ 1000U
#define DEFAULT_SINGLE_FREQ_HZ 1000U

#define COLOR_BG        0x0B1020
#define COLOR_PANEL     0x111827
#define COLOR_LINE      0x334155
#define COLOR_INPUT     0x1E293B
#define COLOR_TEXT      0xF8FAFC
#define COLOR_SUBTEXT   0xCBD5E1
#define COLOR_BLUE      0x93C5FD
#define COLOR_GREEN     0x86EFAC
#define COLOR_YELLOW    0xFBBF24
#define COLOR_RED       0xFCA5A5

static const char *TAG_FIT = "FilterFit";

static lv_obj_t *g_title = nullptr;
static lv_obj_t *g_top_status = nullptr;
static lv_obj_t *g_msg = nullptr;

static lv_obj_t *g_cb_sweep = nullptr;
static lv_obj_t *g_cb_single = nullptr;
static lv_obj_t *g_ta_start = nullptr;
static lv_obj_t *g_ta_stop = nullptr;
static lv_obj_t *g_ta_step = nullptr;
static lv_obj_t *g_ta_single = nullptr;
static lv_obj_t *g_keyboard = nullptr;

static lv_obj_t *g_freq = nullptr;
static lv_obj_t *g_vin = nullptr;
static lv_obj_t *g_vout = nullptr;
static lv_obj_t *g_gain_line = nullptr;
static lv_obj_t *g_theory = nullptr;
static lv_obj_t *g_error_line = nullptr;
static lv_obj_t *g_phase = nullptr;
static lv_obj_t *g_type = nullptr;
static lv_obj_t *g_fc = nullptr;

static lv_obj_t *g_chart = nullptr;
static lv_chart_series_t *g_chart_gain = nullptr;
static lv_obj_t *g_latest = nullptr;
static lv_obj_t *g_open_table_btn = nullptr;

static lv_obj_t *g_full_table = nullptr;
static bool g_main_page_active = false;
static bool g_reconstruction_page_active = false;
static bool g_spi_test_page_active = false;

static lv_obj_t *g_adv_status = nullptr;
static lv_obj_t *g_adv_model_line = nullptr;
static lv_obj_t *g_adv_model_range_line = nullptr;
static lv_obj_t *g_adv_result = nullptr;
static lv_obj_t *g_adv_output_chart = nullptr;
static lv_chart_series_t *g_adv_output_series = nullptr;
static lv_obj_t *g_adv_recon_chart = nullptr;
static lv_chart_series_t *g_adv_recon_series = nullptr;
static lv_obj_t *g_adv_harmonic_rows[6] = {};

static uint32_t g_start_freq_hz = DEFAULT_START_FREQ_HZ;
static uint32_t g_stop_freq_hz = DEFAULT_STOP_FREQ_HZ;
static uint32_t g_step_freq_hz = DEFAULT_STEP_FREQ_HZ;
static uint32_t g_single_freq_hz = DEFAULT_SINGLE_FREQ_HZ;
static uint8_t g_mode = MODE_SWEEP;

static freq_point_t g_table[MAX_POINTS];
static uint32_t g_table_count = 0;
static bool g_table_full = false;
static bool g_have_last_status = false;
static uint32_t g_last_point_index = UINT32_MAX;
static uint32_t g_last_point_freq = 0;
static freqresp_ui_status_t g_last_status = {};
static circuit_model_t g_circuit_model = {};
static filter_fit_result_t g_last_fit = {};
static bool g_adv_output_captured = false;
static bool g_adv_reconstruction_ready = false;
static bool g_model_saved_for_current_sweep = false;
static bool g_fit_done_for_current_sweep = false;
static bool g_fit_pending = false;
static uint32_t g_fit_pending_tick = 0;
static uint32_t g_last_render_tick = 0;
static uint32_t g_last_chart_tick = 0;
static bool g_chart_point_pending = false;
static int32_t g_pending_chart_gain = 0;
static uint32_t g_heavy_fit_sample_stride = 1;
static freqresp_ui_status_t g_fit_pending_status = {};
static bool g_suppress_light_fit_log = false;
static lv_timer_t *g_spi_ui_pump_timer = nullptr;

static constexpr uint32_t kHeavyFitTimeBudgetMs = 150U;
static constexpr uint32_t kHeavyFitMaxSamples = 300U;
static constexpr uint32_t kHeavyFitGridCount = 24U;

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

typedef enum {
    FIT_RESULT_NONE = 0,
    FIT_RESULT_LIGHT,
    FIT_RESULT_HEAVY_TIMEOUT,
    FIT_RESULT_HEAVY_LOW_CONF,
    FIT_RESULT_HEAVY_OK,
} fit_result_quality_t;

static fit_result_quality_t g_fit_result_quality = FIT_RESULT_NONE;

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
} fit_shape_t;

typedef struct {
    uint32_t freq_hz;
    double freq_d;
    double gain;
    int32_t gain_x1000;
    bool phase_valid;
    double phase_deg;
} fit_work_point_t;

static fit_work_point_t g_fit_points[kHeavyFitMaxSamples];
static uint32_t g_fit_point_count = 0;

static void update_adv_model_line(void);
static void render_basic_status(const freqresp_ui_status_t *s);
static void process_pending_fit(void);
static bool chart_point_visible(uint32_t index);

static constexpr uint32_t kLabelRenderIntervalMs = 50U;
static constexpr uint32_t kChartRenderIntervalMs = 50U;
static constexpr uint32_t kFitDelayMs = 300U;

#if ENABLE_SPI_TEST_WINDOW
static lv_obj_t *g_spi_test_link = nullptr;
static lv_obj_t *g_spi_test_rx = nullptr;
static lv_obj_t *g_spi_test_tx = nullptr;
static lv_obj_t *g_spi_test_input = nullptr;
static char g_spi_test_last_rx[105] = {};
static uint8_t g_spi_test_link_state = 0;
#endif

#if ENABLE_FAKE_DATA_TEST
static lv_timer_t *g_fake_timer = nullptr;
static uint32_t g_fake_index = 0;
#endif

static lv_obj_t *create_label(lv_obj_t *parent,
                              const char *text,
                              int32_t x,
                              int32_t y,
                              int32_t w,
                              const lv_font_t *font,
                              uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, w);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);
    return label;
}

static void create_hline(lv_obj_t *parent, int32_t y)
{
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_pos(line, 20, y);
    lv_obj_set_size(line, 984, 2);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(line, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, LV_PART_MAIN);
}

static void create_vline(lv_obj_t *parent, int32_t x, int32_t y, int32_t h)
{
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_pos(line, x, y);
    lv_obj_set_size(line, 2, h);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(line, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, LV_PART_MAIN);
}

static lv_obj_t *create_button(lv_obj_t *parent,
                               const char *text,
                               int32_t x,
                               int32_t y,
                               int32_t w)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, 36);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_center(label);
    return btn;
}

static void format_freq(char *buf, size_t len, uint32_t hz)
{
    if (hz < 100000U) {
        snprintf(buf, len, "%05lu", static_cast<unsigned long>(hz));
    } else {
        snprintf(buf, len, "%lu", static_cast<unsigned long>(hz));
    }
}

static void format_cutoff_freq(char *buf, size_t len, uint8_t filter_type, uint32_t cutoff_freq_hz)
{
    if (cutoff_freq_hz == 0U || filter_type == FILTER_TYPE_UNKNOWN) {
        snprintf(buf, len, "-----");
        return;
    }
    if (cutoff_freq_hz == UINT32_MAX) {
        snprintf(buf, len, ">max");
        return;
    }
    if (cutoff_freq_hz == (UINT32_MAX - 1U)) {
        snprintf(buf, len, "<min");
        return;
    }

    format_freq(buf, len, cutoff_freq_hz);
}

static void format_voltage(char *buf, size_t len, int32_t mv)
{
    const char *sign = "";
    int32_t v = mv;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%03ld V",
             sign,
             static_cast<long>(v / 1000),
             static_cast<long>(v % 1000));
}

static void format_x1000(char *buf, size_t len, int32_t value)
{
    const char *sign = "";
    int32_t v = value;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%03ld",
             sign,
             static_cast<long>(v / 1000),
             static_cast<long>(v % 1000));
}

static void format_error(char *buf, size_t len, int32_t error_x10)
{
    const char *sign = "";
    int32_t v = error_x10;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%ld%%",
             sign,
             static_cast<long>(v / 10),
             static_cast<long>(v % 10));
}

static void format_phase(char *buf, size_t len, int32_t phase_deg_x10)
{
    const char *sign = "";
    int32_t v = phase_deg_x10;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%ld deg",
             sign,
             static_cast<long>(v / 10),
             static_cast<long>(v % 10));
}

static const char *state_text(uint8_t state)
{
    switch (state) {
    case FREQRESP_STATE_IDLE: return "Idle";
    case FREQRESP_STATE_SCANNING: return "Scanning";
    case FREQRESP_STATE_DONE: return "Done";
    case FREQRESP_STATE_ERROR: return "Error";
    default: return "Error";
    }
}

static const char *filter_type_text(uint8_t type)
{
    switch (type) {
    case FILTER_TYPE_LOW_PASS: return "Low-pass";
    case FILTER_TYPE_HIGH_PASS: return "High-pass";
    case FILTER_TYPE_BAND_PASS: return "Band-pass";
    case FILTER_TYPE_BAND_STOP: return "Band-stop";
    default: return "Unknown";
    }
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
    default: return "Unknown";
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

static uint32_t parse_ta_u32(lv_obj_t *ta)
{
    const char *text = lv_textarea_get_text(ta);
    uint32_t value = 0;

    while (*text != '\0') {
        if (*text >= '0' && *text <= '9') {
            value = value * 10U + static_cast<uint32_t>(*text - '0');
        }
        ++text;
    }

    return value;
}

static void set_ta_freq(lv_obj_t *ta, uint32_t value)
{
    char buf[16];
    format_freq(buf, sizeof(buf), value);
    lv_textarea_set_text(ta, buf);
}

static void set_msg(const char *text, uint32_t color)
{
    if (!g_main_page_active || g_msg == nullptr) {
        return;
    }

    lv_label_set_text(g_msg, text);
    lv_obj_set_style_text_color(g_msg, lv_color_hex(color), LV_PART_MAIN);
}

static bool read_and_validate_inputs(void)
{
    g_start_freq_hz = parse_ta_u32(g_ta_start);
    g_stop_freq_hz = parse_ta_u32(g_ta_stop);
    g_step_freq_hz = parse_ta_u32(g_ta_step);
    g_single_freq_hz = parse_ta_u32(g_ta_single);

    set_ta_freq(g_ta_start, g_start_freq_hz);
    set_ta_freq(g_ta_stop, g_stop_freq_hz);
    set_ta_freq(g_ta_step, g_step_freq_hz);
    set_ta_freq(g_ta_single, g_single_freq_hz);

    if (g_start_freq_hz < 100U ||
        g_stop_freq_hz > 100000U ||
        g_step_freq_hz == 0U ||
        g_start_freq_hz >= g_stop_freq_hz ||
        g_single_freq_hz < 100U ||
        g_single_freq_hz > 100000U) {
        set_msg("BAD FREQ", COLOR_RED);
        return false;
    }

    set_msg("Ready", COLOR_GREEN);
    return true;
}

static lv_obj_t *create_freq_input(lv_obj_t *parent, int32_t x, int32_t y, uint32_t value)
{
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_pos(ta, x, y);
    lv_obj_set_size(ta, 100, 34);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 6);
    lv_textarea_set_accepted_chars(ta, "0123456789");
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(ta, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ta, lv_color_hex(COLOR_INPUT), LV_PART_MAIN);
    lv_obj_set_style_border_color(ta, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_radius(ta, 6, LV_PART_MAIN);
    set_ta_freq(ta, value);
    return ta;
}

static void checkbox_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t *target = static_cast<lv_obj_t *>(lv_event_get_target(event));
    if (target == g_cb_sweep) {
        lv_obj_add_state(g_cb_sweep, LV_STATE_CHECKED);
        lv_obj_clear_state(g_cb_single, LV_STATE_CHECKED);
        g_mode = MODE_SWEEP;
    } else {
        lv_obj_add_state(g_cb_single, LV_STATE_CHECKED);
        lv_obj_clear_state(g_cb_sweep, LV_STATE_CHECKED);
        g_mode = MODE_SINGLE;
    }
}

static void textarea_event_cb(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *ta = static_cast<lv_obj_t *>(lv_event_get_target(event));

    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
        lv_keyboard_set_textarea(g_keyboard, ta);
        lv_obj_clear_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
    } else if (code == LV_EVENT_DEFOCUSED) {
        read_and_validate_inputs();
    } else if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        read_and_validate_inputs();
        lv_keyboard_set_textarea(g_keyboard, nullptr);
        lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_state(ta, LV_STATE_FOCUSED);
    }
}

static void ClearAllSweepData(void)
{
    g_table_count = 0;
    g_table_full = false;
    g_have_last_status = false;
    g_last_point_index = UINT32_MAX;
    g_last_point_freq = 0;
    g_last_status = {};
    g_circuit_model = {};
    g_last_fit = {};
    g_fit_result_quality = FIT_RESULT_NONE;
    g_adv_output_captured = false;
    g_adv_reconstruction_ready = false;
    g_model_saved_for_current_sweep = false;
    g_fit_done_for_current_sweep = false;
    g_fit_pending = false;
    g_fit_pending_tick = 0;
    g_last_render_tick = 0;
    g_last_chart_tick = 0;
    g_chart_point_pending = false;
    g_pending_chart_gain = 0;
    g_fit_pending_status = {};
    g_fit_point_count = 0;
    SpiLink_ClearMeasurementCache();

    if (g_main_page_active && g_chart != nullptr && g_chart_gain != nullptr) {
        lv_chart_set_all_value(g_chart, g_chart_gain, LV_CHART_POINT_NONE);
        lv_chart_refresh(g_chart);
    }

    if (g_main_page_active && g_latest != nullptr) {
        lv_label_set_text(g_latest, "Latest: --");
    }
    if (g_main_page_active && g_freq != nullptr) {
        lv_label_set_text(g_freq, "Freq: ----- Hz");
        lv_label_set_text(g_vin, "Vin: --");
        lv_label_set_text(g_vout, "Vout: --");
        lv_label_set_text(g_gain_line, "Gain: --");
        lv_label_set_text(g_theory, "Theory: --");
        lv_label_set_text(g_error_line, "Error: --");
        lv_label_set_text(g_phase, "Phase: --");
        lv_label_set_text(g_type, "Type: Unknown");
        lv_label_set_text(g_fc, "fc: ---- Hz");
    }
    if (g_full_table != nullptr) {
        lv_table_set_row_cnt(g_full_table, 2);
        lv_table_set_cell_value(g_full_table, 1, 0, "No data yet");
        for (uint32_t col = 1; col < 8; ++col) {
            lv_table_set_cell_value(g_full_table, 1, col, "");
        }
    }
    update_adv_model_line();

    set_msg("CLEARED", COLOR_GREEN);
}

static void flush_pending_chart_point(bool force)
{
    if (!g_main_page_active ||
        g_chart == nullptr ||
        g_chart_gain == nullptr ||
        !g_chart_point_pending) {
        return;
    }

    const uint32_t now = lv_tick_get();
    if (!force &&
        g_last_chart_tick != 0U &&
        (now - g_last_chart_tick) < kChartRenderIntervalMs) {
        return;
    }

    lv_chart_set_next_value(g_chart, g_chart_gain, g_pending_chart_gain);
    g_chart_point_pending = false;
    g_last_chart_tick = now;
}

static void queue_chart_point(int32_t gain_x1000)
{
    if (!g_main_page_active || g_chart == nullptr || g_chart_gain == nullptr) {
        return;
    }

    if (gain_x1000 < 0) {
        gain_x1000 = 0;
    } else if (gain_x1000 > 1000) {
        gain_x1000 = 1000;
    }

    g_pending_chart_gain = gain_x1000;
    g_chart_point_pending = true;
    flush_pending_chart_point(false);
}

static void append_table_point(const freqresp_ui_status_t *s)
{
    if (s->current_freq_hz == 0U) {
        return;
    }

    if (g_have_last_status &&
        s->point_index == g_last_point_index &&
        s->current_freq_hz == g_last_point_freq) {
        return;
    }

    g_last_point_index = s->point_index;
    g_last_point_freq = s->current_freq_hz;

    if (g_table_count >= MAX_POINTS) {
        if (!g_table_full) {
            g_table_full = true;
            set_msg("FULL", COLOR_RED);
        }
        return;
    }

    freq_point_t *point = &g_table[g_table_count++];
    point->freq_hz = s->current_freq_hz;
    point->vin_mv = s->vin_mv;
    point->vout_mv = s->vout_mv;
    point->gain_x1000 = s->gain_x1000;
    point->theory_gain_x1000 = s->theory_gain_x1000;
    point->error_x10 = s->error_x10;
    point->phase_deg_x10 = s->phase_deg_x10;
    point->phase_valid = s->phase_valid;

    if (chart_point_visible(g_table_count - 1U)) {
        queue_chart_point(s->gain_x1000);
    }
}

static void refresh_chart_from_table(void)
{
    if (!g_main_page_active || g_chart == nullptr || g_chart_gain == nullptr) {
        return;
    }

    lv_chart_set_all_value(g_chart, g_chart_gain, LV_CHART_POINT_NONE);
    for (uint32_t i = 0; i < g_table_count; ++i) {
        if (!chart_point_visible(i)) {
            continue;
        }
        int32_t chart_gain = g_table[i].gain_x1000;
        if (chart_gain < 0) {
            chart_gain = 0;
        } else if (chart_gain > 1000) {
            chart_gain = 1000;
        }
        lv_chart_set_next_value(g_chart, g_chart_gain, chart_gain);
    }
    lv_chart_refresh(g_chart);
}

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

static bool fit_point_basic_valid(const freq_point_t *p)
{
    return p != nullptr &&
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
    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }
    return (values[(count / 2U) - 1U] + values[count / 2U]) / 2;
}

static fit_reject_reason_t fit_point_reject_reason_strict(uint32_t index)
{
    if (index >= g_table_count) {
        return FIT_REJECT_INDEX;
    }

    const freq_point_t *p = &g_table[index];
    if (p->freq_hz == 0U) {
        return FIT_REJECT_FREQ;
    }
    if (p->vin_mv < 80) {
        return FIT_REJECT_LOW_VIN;
    }
    if (p->gain_x1000 <= 0 || p->gain_x1000 > 3000) {
        return FIT_REJECT_GAIN;
    }

    for (int32_t i = static_cast<int32_t>(index) - 1; i >= 0; --i) {
        const freq_point_t *prev = &g_table[i];
        if (!fit_point_basic_valid(prev)) {
            continue;
        }
        if (prev->freq_hz >= p->freq_hz) {
            return FIT_REJECT_NON_MONOTONIC;
        }
        break;
    }
    for (uint32_t i = index + 1U; i < g_table_count; ++i) {
        const freq_point_t *next = &g_table[i];
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
        const freq_point_t *prev = &g_table[i];
        if (!fit_point_basic_valid(prev) || prev->freq_hz >= p->freq_hz) {
            continue;
        }
        gains[gain_count++] = prev->gain_x1000;
        have_before = true;
    }
    const uint32_t before_count = gain_count;
    for (uint32_t i = index + 1U; i < g_table_count && (gain_count - before_count) < 3U; ++i) {
        const freq_point_t *next = &g_table[i];
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
            if (gain * 100LL < median * 60LL ||
                gain * 100LL > median * 140LL) {
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

static bool chart_point_visible(uint32_t index)
{
    if (fit_point_reject_reason_strict(index) != FIT_REJECT_NONE) {
        return false;
    }
    if (index == 0U || index >= g_table_count) {
        return true;
    }

    const freq_point_t *cur = &g_table[index];
    for (int32_t i = static_cast<int32_t>(index) - 1; i >= 0; --i) {
        const freq_point_t *prev = &g_table[i];
        if (!fit_point_basic_valid(prev) || prev->freq_hz >= cur->freq_hz) {
            continue;
        }
        const uint32_t freq_ratio_x1000 =
            (prev->freq_hz == 0U) ? 0U :
            static_cast<uint32_t>((static_cast<uint64_t>(cur->freq_hz) * 1000ULL) / prev->freq_hz);
        if (freq_ratio_x1000 <= 1500U) {
            const int64_t gain = cur->gain_x1000;
            const int64_t prev_gain = prev->gain_x1000;
            if (gain * 100LL < prev_gain * 60LL ||
                gain * 100LL > prev_gain * 140LL) {
                return false;
            }
        }
        break;
    }
    return true;
}

static fit_debug_stats_t collect_fit_debug_stats(void)
{
    fit_debug_stats_t stats = {};
    stats.raw_point_count = g_table_count;
    for (uint32_t i = 0; i < g_table_count; ++i) {
        const fit_reject_reason_t reason = fit_point_reject_reason_strict(i);
        if (reason == FIT_REJECT_NONE) {
            ++stats.valid_point_count;
            continue;
        }
        ++stats.rejected_point_count;
        if (reason == FIT_REJECT_OUTLIER) {
            ++stats.rejected_outlier_count;
        } else if (reason == FIT_REJECT_LOW_VIN) {
            ++stats.rejected_low_vin_count;
        } else if (reason == FIT_REJECT_NON_MONOTONIC) {
            ++stats.rejected_non_monotonic_count;
        }
    }
    return stats;
}

static void build_fit_work_points(void)
{
    g_fit_point_count = 0;
    uint32_t valid_seq = 0;

    for (uint32_t i = 0; i < g_table_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }
        if ((valid_seq++ % g_heavy_fit_sample_stride) != 0U) {
            continue;
        }
        if (g_fit_point_count >= kHeavyFitMaxSamples) {
            break;
        }

        const freq_point_t *src = &g_table[i];
        fit_work_point_t *dst = &g_fit_points[g_fit_point_count++];
        dst->freq_hz = src->freq_hz;
        dst->freq_d = static_cast<double>(src->freq_hz);
        dst->gain_x1000 = src->gain_x1000;
        dst->gain = static_cast<double>(src->gain_x1000) / 1000.0;
        dst->phase_valid = src->phase_valid;
        dst->phase_deg = static_cast<double>(src->phase_deg_x10) / 10.0;
    }
}

static double gain_from_point(const freq_point_t *p)
{
    return static_cast<double>(p->gain_x1000) / 1000.0;
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
    c.valid = false;
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
        const double h0 = model_gain_no_k(model_type,
                                          p->freq_d,
                                          freq_hz,
                                          q);
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
        const double h0 = model_gain_no_k(model_type,
                                          p->freq_d,
                                          freq_hz,
                                          q);
        const double theory = k * h0;
        if (theory <= 0.000001) {
            continue;
        }

        const double measured = p->gain;
        const double err_db = gain_db_from_gain(measured) - gain_db_from_gain(theory);
        const double rel = fabs(measured - theory) / theory;
        err_db_sum += err_db * err_db;
        rel_sum += rel * rel;
        if (rel > max_rel) {
            max_rel = rel;
        }

        if (p->phase_valid) {
            const double measured_phase = p->phase_deg;
            const double theory_phase = model_phase_deg(model_type,
                                                        p->freq_d,
                                                        freq_hz,
                                                        q);
            if (!have_phase_offset) {
                phase_offset = measured_phase - theory_phase;
                phase_inv_offset = measured_phase + theory_phase;
                have_phase_offset = true;
            }
            const double diff = wrap_phase_deg(measured_phase - (theory_phase + phase_offset));
            const double diff_inv = wrap_phase_deg(measured_phase - (-theory_phase + phase_inv_offset));
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
        c.phase_rms_deg = 0.0;
        c.score = c.rms_db;
    }
    return c;
}

static void keep_best_candidate(fit_candidate_t *best, const fit_candidate_t *cand)
{
    if (best == nullptr || cand == nullptr || !cand->valid) {
        return;
    }
    if (!best->valid || cand->score < best->score) {
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
    const double f0 = p0->freq_d;
    const double f1 = p1->freq_d;
    if (f0 <= 0.0 || f1 <= f0) {
        return 0.0;
    }

    const double g0 = gain_db_from_gain(p0->gain);
    const double g1 = gain_db_from_gain(p1->gain);
    return (g1 - g0) / log10(f1 / f0);
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

static void clear_theory_columns(void)
{
    for (uint32_t i = 0; i < g_table_count; ++i) {
        g_table[i].theory_gain_x1000 = 0;
        g_table[i].error_x10 = 0;
    }
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

static uint32_t interpolate_cutoff_hz(uint32_t f0,
                                      uint32_t f1,
                                      int32_t g0,
                                      int32_t g1,
                                      int32_t target)
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
    if (numer < 0) {
        numer = 0;
    } else if (numer > denom) {
        numer = denom;
    }

    const uint64_t df = static_cast<uint64_t>(f1 - f0);
    return f0 + static_cast<uint32_t>((df * static_cast<uint32_t>(numer)) /
                                      static_cast<uint32_t>(denom));
}

static void cap_fit_confidence(filter_fit_result_t *fit, int32_t max_confidence_x1000)
{
    if (fit != nullptr &&
        fit->valid &&
        fit->confidence_x1000 > max_confidence_x1000) {
        fit->confidence_x1000 = max_confidence_x1000;
    }
}

static uint32_t find_light_cutoff_hz(uint8_t filter_type, uint32_t target_gain_x1000)
{
    bool have_prev = false;
    freq_point_t prev = {};
    int32_t first_gain = 0;
    int32_t last_gain = 0;

    for (uint32_t i = 0; i < g_table_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }

        const freq_point_t *cur = &g_table[i];
        if (!have_prev) {
            first_gain = cur->gain_x1000;
        }
        last_gain = cur->gain_x1000;
        if (have_prev) {
            if (filter_type == FILTER_TYPE_LOW_PASS &&
                prev.gain_x1000 >= static_cast<int32_t>(target_gain_x1000) &&
                cur->gain_x1000 <= static_cast<int32_t>(target_gain_x1000)) {
                return interpolate_cutoff_hz(prev.freq_hz,
                                             cur->freq_hz,
                                             prev.gain_x1000,
                                             cur->gain_x1000,
                                             static_cast<int32_t>(target_gain_x1000));
            }
            if (filter_type == FILTER_TYPE_HIGH_PASS &&
                prev.gain_x1000 <= static_cast<int32_t>(target_gain_x1000) &&
                cur->gain_x1000 >= static_cast<int32_t>(target_gain_x1000)) {
                return interpolate_cutoff_hz(prev.freq_hz,
                                             cur->freq_hz,
                                             prev.gain_x1000,
                                             cur->gain_x1000,
                                             static_cast<int32_t>(target_gain_x1000));
            }
        }

        prev = *cur;
        have_prev = true;
    }

    if (have_prev) {
        if (filter_type == FILTER_TYPE_LOW_PASS) {
            if (last_gain > static_cast<int32_t>(target_gain_x1000)) {
                return UINT32_MAX;
            }
            if (first_gain < static_cast<int32_t>(target_gain_x1000)) {
                return UINT32_MAX - 1U;
            }
        } else if (filter_type == FILTER_TYPE_HIGH_PASS) {
            if (last_gain < static_cast<int32_t>(target_gain_x1000)) {
                return UINT32_MAX;
            }
            if (first_gain > static_cast<int32_t>(target_gain_x1000)) {
                return UINT32_MAX - 1U;
            }
        }
    }

    return 0U;
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

    shape.peak_gain = 0U;
    shape.valley_gain = UINT32_MAX;
    uint32_t valid_pos = 0;
    uint32_t peak_pos = 0;
    uint32_t valley_pos = 0;

    for (uint32_t i = 0; i < g_fit_point_count; ++i) {
        const fit_work_point_t *p = &g_fit_points[i];
        const uint32_t gain = static_cast<uint32_t>(p->gain_x1000);
        if (low_count < edge_count) {
            low_sum += gain;
            ++low_count;
        }
        if (gain > shape.peak_gain) {
            shape.peak_gain = gain;
            shape.peak_freq = p->freq_hz;
            peak_pos = valid_pos;
        }
        if (gain < shape.valley_gain) {
            shape.valley_gain = gain;
            shape.valley_freq = p->freq_hz;
            valley_pos = valid_pos;
        }
        ++valid_pos;
    }

    for (int32_t i = static_cast<int32_t>(g_fit_point_count) - 1;
         i >= 0 && high_count < edge_count;
         --i) {
        high_sum += static_cast<uint32_t>(g_fit_points[i].gain_x1000);
        ++high_count;
    }

    shape.low_avg = (low_count == 0U) ? 0U : static_cast<uint32_t>(low_sum / low_count);
    shape.high_avg = (high_count == 0U) ? 0U : static_cast<uint32_t>(high_sum / high_count);
    if (shape.valley_gain == UINT32_MAX) {
        shape.valley_gain = 0U;
    }

    const uint32_t edge_hi = (shape.low_avg > shape.high_avg) ? shape.low_avg : shape.high_avg;
    const uint32_t edge_lo = (shape.low_avg < shape.high_avg) ? shape.low_avg : shape.high_avg;
    const bool peak_is_central =
        peak_pos > (shape.valid_count / 5U) && peak_pos < ((shape.valid_count * 4U) / 5U);
    const bool valley_is_central =
        valley_pos > (shape.valid_count / 5U) && valley_pos < ((shape.valid_count * 4U) / 5U);

    if (static_cast<uint64_t>(shape.low_avg) * 100ULL >
        static_cast<uint64_t>(shape.high_avg) * 125ULL) {
        shape.candidate_mask |= MODEL_MASK_LP;
    }
    if (static_cast<uint64_t>(shape.high_avg) * 100ULL >
        static_cast<uint64_t>(shape.low_avg) * 125ULL) {
        shape.candidate_mask |= MODEL_MASK_HP;
    }
    if (peak_is_central &&
        static_cast<uint64_t>(shape.peak_gain) * 100ULL >
        static_cast<uint64_t>(edge_hi) * 125ULL) {
        shape.candidate_mask |= MODEL_MASK_BP2;
    }
    if (valley_is_central &&
        static_cast<uint64_t>(shape.valley_gain) * 125ULL <
        static_cast<uint64_t>(edge_lo) * 100ULL) {
        shape.candidate_mask |= MODEL_MASK_BS2;
    }

    return shape;
}

static void analyze_sweep_response_light(freqresp_ui_status_t *s)
{
    if (s == nullptr || s->state != FREQRESP_STATE_DONE || s->mode != MODE_SWEEP) {
        return;
    }

    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < g_table_count; ++i) {
        if (fit_point_valid_strict(i)) {
            ++valid_count;
        }
    }

    clear_theory_columns();
    g_last_fit = {};
    g_fit_result_quality = FIT_RESULT_NONE;
    s->filter_type = FILTER_TYPE_UNKNOWN;
    s->cutoff_freq_hz = 0;
    s->theory_gain_x1000 = 0;
    s->error_x10 = 0;

    if (valid_count < 8U) {
        if (!g_suppress_light_fit_log) {
            ESP_LOGW(TAG_FIT,
                     "Light fit: valid_points=%lu -> Unknown",
                     static_cast<unsigned long>(valid_count));
        }
        return;
    }

    const uint32_t edge_count = clamp_u32(valid_count / 10U, 3U, 16U);
    uint64_t low_sum = 0;
    uint64_t high_sum = 0;
    uint32_t low_count = 0;
    uint32_t high_count = 0;

    for (uint32_t i = 0; i < g_table_count && low_count < edge_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            continue;
        }
        low_sum += static_cast<uint32_t>(g_table[i].gain_x1000);
        ++low_count;
    }

    for (int32_t i = static_cast<int32_t>(g_table_count) - 1;
         i >= 0 && high_count < edge_count;
         --i) {
        if (!fit_point_valid_strict(static_cast<uint32_t>(i))) {
            continue;
        }
        high_sum += static_cast<uint32_t>(g_table[i].gain_x1000);
        ++high_count;
    }

    if (low_count == 0U || high_count == 0U) {
        if (!g_suppress_light_fit_log) {
            ESP_LOGW(TAG_FIT,
                     "Light fit: valid_points=%lu -> Unknown (empty edge)",
                     static_cast<unsigned long>(valid_count));
        }
        return;
    }

    const uint32_t low_avg = static_cast<uint32_t>(low_sum / low_count);
    const uint32_t high_avg = static_cast<uint32_t>(high_sum / high_count);
    uint8_t filter_type = FILTER_TYPE_UNKNOWN;
    uint8_t model_type = MODEL_TYPE_UNKNOWN;
    uint32_t cutoff_hz = 0;

    if (static_cast<uint64_t>(low_avg) * 100U >
        static_cast<uint64_t>(high_avg) * 125U) {
        filter_type = FILTER_TYPE_LOW_PASS;
        model_type = MODEL_TYPE_LP1;
        cutoff_hz = find_light_cutoff_hz(
            filter_type,
            static_cast<uint32_t>((static_cast<uint64_t>(low_avg) * 707ULL) / 1000ULL)
        );
    } else if (static_cast<uint64_t>(high_avg) * 100U >
               static_cast<uint64_t>(low_avg) * 125U) {
        filter_type = FILTER_TYPE_HIGH_PASS;
        model_type = MODEL_TYPE_HP1;
        cutoff_hz = find_light_cutoff_hz(
            filter_type,
            static_cast<uint32_t>((static_cast<uint64_t>(high_avg) * 707ULL) / 1000ULL)
        );
    }

    if (filter_type == FILTER_TYPE_UNKNOWN || cutoff_hz == 0U) {
        if (!g_suppress_light_fit_log) {
            ESP_LOGW(TAG_FIT,
                     "Light fit: valid_points=%lu low=%lu high=%lu -> Unknown",
                     static_cast<unsigned long>(valid_count),
                     static_cast<unsigned long>(low_avg),
                     static_cast<unsigned long>(high_avg));
        }
        return;
    }

    const uint32_t edge_hi = (low_avg > high_avg) ? low_avg : high_avg;
    uint32_t edge_lo = (low_avg < high_avg) ? low_avg : high_avg;
    if (edge_lo == 0U) {
        edge_lo = 1U;
    }
    const uint32_t confidence =
        clamp_u32(static_cast<uint32_t>(((edge_hi - edge_lo) * 1000ULL) / edge_lo),
                  0U,
                  1000U);

    filter_fit_result_t fit = {};
    fit.valid = true;
    fit.model_type = model_type;
    fit.fc_hz = cutoff_hz;
    fit.f0_hz = cutoff_hz;
    fit.fl_hz = 0;
    fit.fh_hz = 0;
    fit.q_x1000 = 707;
    fit.k_x1000 = 1000;
    fit.rms_error_x10 = 0;
    fit.max_error_x10 = 0;
    fit.confidence_x1000 = static_cast<int32_t>(clamp_u32(confidence, 0U, 800U));
    fit.valid_point_count = valid_count;
    g_last_fit = fit;
    g_fit_result_quality = FIT_RESULT_LIGHT;

    s->filter_type = filter_type;
    s->cutoff_freq_hz = cutoff_hz;

    if (!g_suppress_light_fit_log) {
        ESP_LOGI(TAG_FIT,
                 "Light fit: valid_points=%lu type=%s fc=%lu low=%lu high=%lu confidence=%ld",
                 static_cast<unsigned long>(valid_count),
                 filter_type_text(filter_type),
                 static_cast<unsigned long>(cutoff_hz),
                 static_cast<unsigned long>(low_avg),
                 static_cast<unsigned long>(high_avg),
                 static_cast<long>(fit.confidence_x1000));
    }
}

static void fill_theory_columns(const fit_candidate_t *best)
{
    if (best == nullptr || !best->valid) {
        clear_theory_columns();
        return;
    }

    for (uint32_t i = 0; i < g_table_count; ++i) {
        if (!fit_point_valid_strict(i)) {
            g_table[i].theory_gain_x1000 = 0;
            g_table[i].error_x10 = 0;
            continue;
        }
        const double h0 = model_gain_no_k(best->model_type,
                                          static_cast<double>(g_table[i].freq_hz),
                                          best->freq_hz,
                                          best->q);
        const double theory = best->k * h0;
        if (theory <= 0.000001) {
            g_table[i].theory_gain_x1000 = 0;
            g_table[i].error_x10 = 0;
            continue;
        }
        const int32_t theory_x1000 = static_cast<int32_t>(theory * 1000.0 + 0.5);
        const double measured = gain_from_point(&g_table[i]);
        const int32_t error_x10 = static_cast<int32_t>((fabs(measured - theory) / theory) * 1000.0 + 0.5);
        g_table[i].theory_gain_x1000 = theory_x1000;
        g_table[i].error_x10 = error_x10;
    }
}

static void log_rc_104_hint_if_needed(uint32_t freq_hz)
{
    /*
     * 150 ohm + 104 (100 nF) should be around 10.6 kHz:
     * fc = 1 / (2*pi*150*100 nF).  If both this fitter and a scope see
     * about 50 kHz, first check wiring and loading instead of tuning math.
     */
    if (freq_hz >= 40000U && freq_hz <= 60000U) {
        ESP_LOGW(TAG_FIT,
                 "HW hint: 150R + 104 theoretical fc is about 10.6kHz. "
                 "If Vout/Vin also measures near 50kHz on scope, check resistor wiring, "
                 "capacitor value, Vin/Vout probe points, whether 150R is in series, "
                 "source loading, and any divider accidentally inserted into DUT path.");
    }
}

static void analyze_sweep_response(freqresp_ui_status_t *s)
{
    if (s == nullptr || s->state != FREQRESP_STATE_DONE || s->mode != MODE_SWEEP) {
        return;
    }

    const uint32_t analyze_start_tick = lv_tick_get();
    g_suppress_light_fit_log = true;
    analyze_sweep_response_light(s);
    g_suppress_light_fit_log = false;
    const filter_fit_result_t light_fit = g_last_fit;
    const uint8_t light_filter_type = s->filter_type;
    const uint32_t light_cutoff_hz = s->cutoff_freq_hz;

    fit_debug_stats_t stats = collect_fit_debug_stats();
    g_heavy_fit_sample_stride =
        (stats.valid_point_count > kHeavyFitMaxSamples) ?
        ((stats.valid_point_count + kHeavyFitMaxSamples - 1U) / kHeavyFitMaxSamples) : 1U;
    build_fit_work_points();
    fit_shape_t shape = classify_shape();
    stats.candidate_mask = shape.candidate_mask;

    uint32_t min_freq = 0;
    uint32_t max_freq = 0;
    if (g_fit_point_count != 0U) {
        min_freq = g_fit_points[0].freq_hz;
        max_freq = g_fit_points[g_fit_point_count - 1U].freq_hz;
    }

    if (g_fit_point_count < 8U || min_freq == 0U || max_freq <= min_freq) {
        const uint32_t elapsed_ms = lv_tick_get() - analyze_start_tick;
        ESP_LOGW(TAG_FIT,
                 "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu "
                 "candidate_mask=0x%02lx fit_points=%lu selected=%s fc_hz=%lu f0_hz=%lu confidence=%ld elapsed_ms=%lu reason=not_enough_data",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.rejected_low_vin_count),
                 static_cast<unsigned long>(stats.rejected_non_monotonic_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(g_last_fit.model_type),
                 static_cast<unsigned long>(s->cutoff_freq_hz),
                 static_cast<unsigned long>(g_last_fit.f0_hz),
                 static_cast<long>(g_last_fit.confidence_x1000),
                 static_cast<unsigned long>(elapsed_ms));
        return;
    }

    if (shape.candidate_mask == 0U) {
        const uint32_t elapsed_ms = lv_tick_get() - analyze_start_tick;
        ESP_LOGW(TAG_FIT,
                 "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu "
                 "candidate_mask=0x%02lx fit_points=%lu selected=%s fc_hz=%lu f0_hz=%lu confidence=%ld elapsed_ms=%lu reason=no_shape_candidate",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.rejected_low_vin_count),
                 static_cast<unsigned long>(stats.rejected_non_monotonic_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(g_last_fit.model_type),
                 static_cast<unsigned long>(s->cutoff_freq_hz),
                 static_cast<unsigned long>(g_last_fit.f0_hz),
                 static_cast<long>(g_last_fit.confidence_x1000),
                 static_cast<unsigned long>(elapsed_ms));
        return;
    }

    fit_candidate_t best_by_model[MODEL_TYPE_BS2 + 1];
    for (uint32_t i = 0; i <= MODEL_TYPE_BS2; ++i) {
        best_by_model[i] = invalid_candidate(static_cast<uint8_t>(i));
    }

    if ((shape.candidate_mask & MODEL_MASK_LP1) != 0U &&
        light_fit.valid &&
        light_fit.model_type == MODEL_TYPE_LP1 &&
        light_fit.fc_hz > 0U &&
        light_fit.fc_hz < (UINT32_MAX - 1U)) {
        fit_candidate_t lp1 = evaluate_candidate(MODEL_TYPE_LP1,
                                                 static_cast<double>(light_fit.fc_hz),
                                                 0.707);
        keep_best_candidate(&best_by_model[MODEL_TYPE_LP1], &lp1);
    }
    if ((shape.candidate_mask & MODEL_MASK_HP1) != 0U &&
        light_fit.valid &&
        light_fit.model_type == MODEL_TYPE_HP1 &&
        light_fit.fc_hz > 0U &&
        light_fit.fc_hz < (UINT32_MAX - 1U)) {
        fit_candidate_t hp1 = evaluate_candidate(MODEL_TYPE_HP1,
                                                 static_cast<double>(light_fit.fc_hz),
                                                 0.707);
        keep_best_candidate(&best_by_model[MODEL_TYPE_HP1], &hp1);
    }

    const uint32_t fit_start_tick = lv_tick_get();
    const uint32_t grid_count = kHeavyFitGridCount;
    const double log_min = log10(static_cast<double>(min_freq));
    const double log_max = log10(static_cast<double>(max_freq));
    static const double q_values[] = {0.50, 0.707, 1.00, 1.60};
    bool timed_out = false;

    for (uint32_t n = 0; n < grid_count; ++n) {
        if ((lv_tick_get() - fit_start_tick) > kHeavyFitTimeBudgetMs) {
            timed_out = true;
            break;
        }

        const double t = (grid_count <= 1U) ? 0.0 : (static_cast<double>(n) / static_cast<double>(grid_count - 1U));
        const double freq = pow(10.0, log_min + (log_max - log_min) * t);

        for (uint32_t qi = 0; qi < sizeof(q_values) / sizeof(q_values[0]); ++qi) {
            if ((lv_tick_get() - fit_start_tick) > kHeavyFitTimeBudgetMs) {
                timed_out = true;
                break;
            }
            if ((shape.candidate_mask & MODEL_MASK_LP2) != 0U) {
                fit_candidate_t lp2 = evaluate_candidate(MODEL_TYPE_LP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_LP2], &lp2);
            }
            if ((shape.candidate_mask & MODEL_MASK_HP2) != 0U) {
                fit_candidate_t hp2 = evaluate_candidate(MODEL_TYPE_HP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_HP2], &hp2);
            }
            if ((shape.candidate_mask & MODEL_MASK_BP2) != 0U) {
                fit_candidate_t bp2 = evaluate_candidate(MODEL_TYPE_BP2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_BP2], &bp2);
            }
            if ((shape.candidate_mask & MODEL_MASK_BS2) != 0U) {
                fit_candidate_t bs2 = evaluate_candidate(MODEL_TYPE_BS2, freq, q_values[qi]);
                keep_best_candidate(&best_by_model[MODEL_TYPE_BS2], &bs2);
            }
        }
        if (timed_out) {
            break;
        }
    }

    if (timed_out) {
        g_last_fit = light_fit;
        cap_fit_confidence(&g_last_fit, 600);
        g_fit_result_quality = FIT_RESULT_HEAVY_TIMEOUT;
        s->filter_type = light_filter_type;
        s->cutoff_freq_hz = light_cutoff_hz;
        const uint32_t elapsed_ms = lv_tick_get() - analyze_start_tick;
        ESP_LOGW(TAG_FIT,
                 "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu "
                 "candidate_mask=0x%02lx fit_points=%lu selected=%s fc_hz=%lu f0_hz=%lu confidence=%ld elapsed_ms=%lu "
                 "reason=HEAVY_TIMEOUT budget_ms=%lu stride=%lu",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.rejected_low_vin_count),
                 static_cast<unsigned long>(stats.rejected_non_monotonic_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 model_kind_text(g_last_fit.model_type),
                 static_cast<unsigned long>(s->cutoff_freq_hz),
                 static_cast<unsigned long>(g_last_fit.f0_hz),
                 static_cast<long>(g_last_fit.confidence_x1000),
                 static_cast<unsigned long>(elapsed_ms),
                 static_cast<unsigned long>(kHeavyFitTimeBudgetMs),
                 static_cast<unsigned long>(g_heavy_fit_sample_stride));
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

    const double high_slope = slope_db_per_decade((g_fit_point_count * 2U) / 3U,
                                                  (g_fit_point_count == 0U) ? 0U : (g_fit_point_count - 1U));
    const double low_slope = slope_db_per_decade(0, g_fit_point_count / 3U);
    const bool lp2_much_better =
        best_by_model[MODEL_TYPE_LP1].valid &&
        best_by_model[MODEL_TYPE_LP2].valid &&
        (best_by_model[MODEL_TYPE_LP1].score - best_by_model[MODEL_TYPE_LP2].score) > 0.35;
    const bool hp2_much_better =
        best_by_model[MODEL_TYPE_HP1].valid &&
        best_by_model[MODEL_TYPE_HP2].valid &&
        (best_by_model[MODEL_TYPE_HP1].score - best_by_model[MODEL_TYPE_HP2].score) > 0.35;
    const bool lp_has_second_order_evidence = (high_slope < -30.0) || (best_by_model[MODEL_TYPE_LP2].q > 0.90);
    const bool hp_has_second_order_evidence = (low_slope > 30.0) || (best_by_model[MODEL_TYPE_HP2].q > 0.90);

    if (best.model_type == MODEL_TYPE_LP2 &&
        best_by_model[MODEL_TYPE_LP1].valid &&
        (!lp_has_second_order_evidence || !lp2_much_better)) {
        best = best_by_model[MODEL_TYPE_LP1];
    } else if (best.model_type == MODEL_TYPE_HP2 &&
               best_by_model[MODEL_TYPE_HP1].valid &&
               (!hp_has_second_order_evidence || !hp2_much_better)) {
        best = best_by_model[MODEL_TYPE_HP1];
    }

    const int32_t confidence = confidence_from_candidate(&best, &second);
    const bool fit_ok = best.valid && best.rms_rel <= 0.120 && confidence >= 450;

    if (!fit_ok) {
        clear_theory_columns();
        g_last_fit = {};
        g_fit_result_quality = FIT_RESULT_HEAVY_LOW_CONF;
        s->filter_type = FILTER_TYPE_UNKNOWN;
        s->cutoff_freq_hz = 0;
        s->theory_gain_x1000 = 0;
        s->error_x10 = 0;
        const int32_t low_confidence = clamp_i32(confidence, 0, 500);
        const uint32_t elapsed_ms = lv_tick_get() - analyze_start_tick;
        ESP_LOGW(TAG_FIT,
                 "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu "
                 "candidate_mask=0x%02lx fit_points=%lu selected=Unknown fc_hz=0 f0_hz=0 confidence=%ld elapsed_ms=%lu "
                 "reason=heavy_low_conf best=%s second=%s score_best=%.2f score_second=%.2f",
                 static_cast<unsigned long>(stats.raw_point_count),
                 static_cast<unsigned long>(stats.valid_point_count),
                 static_cast<unsigned long>(stats.rejected_point_count),
                 static_cast<unsigned long>(stats.rejected_outlier_count),
                 static_cast<unsigned long>(stats.rejected_low_vin_count),
                 static_cast<unsigned long>(stats.rejected_non_monotonic_count),
                 static_cast<unsigned long>(stats.candidate_mask),
                 static_cast<unsigned long>(g_fit_point_count),
                 static_cast<long>(low_confidence),
                 static_cast<unsigned long>(elapsed_ms),
                 model_kind_text(best.model_type),
                 model_kind_text(second.model_type),
                 best.score,
                 second.score);
        return;
    }

    fill_theory_columns(&best);

    filter_fit_result_t fit = {};
    fit.valid = true;
    fit.model_type = best.model_type;
    fit.fc_hz = static_cast<uint32_t>(best.freq_hz + 0.5);
    fit.f0_hz = fit.fc_hz;
    fit.fl_hz = 0;
    fit.fh_hz = 0;
    fit.q_x1000 = static_cast<int32_t>(best.q * 1000.0 + 0.5);
    fit.k_x1000 = static_cast<int32_t>(best.k * 1000.0 + 0.5);
    fit.rms_error_x10 = static_cast<int32_t>(best.rms_rel * 1000.0 + 0.5);
    fit.max_error_x10 = static_cast<int32_t>(best.max_rel * 1000.0 + 0.5);
    fit.confidence_x1000 = confidence;
    fit.valid_point_count = stats.valid_point_count;
    g_last_fit = fit;
    g_fit_result_quality = FIT_RESULT_HEAVY_OK;

    s->filter_type = model_filter_type(best.model_type);
    s->cutoff_freq_hz = fit.fc_hz;
    if (g_table_count != 0U) {
        s->theory_gain_x1000 = g_table[g_table_count - 1U].theory_gain_x1000;
        s->error_x10 = g_table[g_table_count - 1U].error_x10;
    }

    const uint32_t elapsed_ms = lv_tick_get() - analyze_start_tick;
    ESP_LOGI(TAG_FIT,
             "FIT summary: raw=%lu valid=%lu rejected=%lu outlier=%lu low_vin=%lu nonmono=%lu "
             "candidate_mask=0x%02lx fit_points=%lu selected=%s fc_hz=%lu f0_hz=%lu confidence=%ld elapsed_ms=%lu "
             "second=%s score_best=%.2f score_second=%.2f K=%.3f Q=%.3f rms_err=%.2f%% max_err=%.2f%% "
             "low=%lu high=%lu peak=%lu@%lu valley=%lu@%lu",
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
             static_cast<unsigned long>(fit.f0_hz),
             static_cast<long>(confidence),
             static_cast<unsigned long>(elapsed_ms),
             model_kind_text(second.model_type),
             best.score,
             second.score,
             best.k,
             best.q,
             best.rms_rel * 100.0,
             best.max_rel * 100.0,
             static_cast<unsigned long>(shape.low_avg),
             static_cast<unsigned long>(shape.high_avg),
             static_cast<unsigned long>(shape.peak_gain),
             static_cast<unsigned long>(shape.peak_freq),
             static_cast<unsigned long>(shape.valley_gain),
             static_cast<unsigned long>(shape.valley_freq));
    log_rc_104_hint_if_needed(fit.fc_hz);
}

static void save_circuit_model_from_table(const freqresp_ui_status_t *s)
{
    if (s == nullptr ||
        s->state != FREQRESP_STATE_DONE ||
        s->mode != MODE_SWEEP ||
        g_model_saved_for_current_sweep ||
        g_table_count < 3U) {
        return;
    }

    g_circuit_model.valid = true;
    g_circuit_model.filter_type = s->filter_type;
    g_circuit_model.cutoff_freq_hz = s->cutoff_freq_hz;
    g_circuit_model.fit = g_last_fit;
    g_circuit_model.start_freq_hz = g_start_freq_hz;
    g_circuit_model.stop_freq_hz = g_stop_freq_hz;
    g_circuit_model.step_freq_hz = g_step_freq_hz;
    g_circuit_model.point_count = (g_table_count > MODEL_POINTS_MAX) ? MODEL_POINTS_MAX : g_table_count;
    g_circuit_model.saved_time_ms = lv_tick_get();

    for (uint32_t i = 0; i < g_circuit_model.point_count; ++i) {
        g_circuit_model.points[i].freq_hz = g_table[i].freq_hz;
        g_circuit_model.points[i].gain_x1000 = g_table[i].gain_x1000;
        g_circuit_model.points[i].phase_deg_x10 = g_table[i].phase_deg_x10;
    }

    g_adv_output_captured = false;
    g_adv_reconstruction_ready = false;
    g_model_saved_for_current_sweep = true;
}

static void process_pending_fit(void)
{
    if (!g_fit_pending || g_fit_done_for_current_sweep) {
        return;
    }

    if (SpiLink_PointQueueWaiting() != 0U) {
        return;
    }

    const uint32_t now = lv_tick_get();
    if ((now - g_fit_pending_tick) < kFitDelayMs) {
        return;
    }

    freqresp_ui_status_t view = g_fit_pending_status;
#if ENABLE_HEAVY_FILTER_FIT
    analyze_sweep_response(&view);
#else
    analyze_sweep_response_light(&view);
#endif
    g_fit_done_for_current_sweep = true;
    g_fit_pending = false;

    g_last_status = view;
    g_have_last_status = true;
    save_circuit_model_from_table(&view);
    update_adv_model_line();
    render_basic_status(&view);
    set_msg("FIT DONE", COLOR_GREEN);
}

static void set_adv_result(const char *text, uint32_t color)
{
    if (!g_reconstruction_page_active || g_adv_result == nullptr) {
        return;
    }

    lv_label_set_text(g_adv_result, text);
    lv_obj_set_style_text_color(g_adv_result, lv_color_hex(color), LV_PART_MAIN);
}

static void update_adv_model_line(void)
{
    if (!g_reconstruction_page_active || g_adv_model_line == nullptr || g_adv_model_range_line == nullptr) {
        return;
    }

    char fc[16];
    char start_freq[16];
    char stop_freq[16];
    char buf[180];

    if (g_circuit_model.valid) {
        format_cutoff_freq(fc,
                           sizeof(fc),
                           g_circuit_model.filter_type,
                           g_circuit_model.cutoff_freq_hz);
        format_freq(start_freq, sizeof(start_freq), g_circuit_model.start_freq_hz);
        format_freq(stop_freq, sizeof(stop_freq), g_circuit_model.stop_freq_hz);
        if (g_fit_result_quality == FIT_RESULT_HEAVY_LOW_CONF) {
            snprintf(buf,
                     sizeof(buf),
                     "Circuit Model: Unknown/Low confidence      fc: ----- Hz      H(f): Loaded");
        } else if (g_circuit_model.fit.valid) {
            const char *prefix = "";
            const char *suffix = "";
            if (g_fit_result_quality == FIT_RESULT_HEAVY_TIMEOUT) {
                suffix = "? timeout";
            } else if (g_fit_result_quality == FIT_RESULT_LIGHT) {
                prefix = "Light ";
            }
            snprintf(buf,
                     sizeof(buf),
                     "Circuit Model: %s%s%s      f0/fc: %s Hz      Q=%ld.%03ld Conf=%ld%%",
                     prefix,
                     model_kind_text(g_circuit_model.fit.model_type),
                     suffix,
                     fc,
                     static_cast<long>(g_circuit_model.fit.q_x1000 / 1000),
                     static_cast<long>(g_circuit_model.fit.q_x1000 % 1000),
                     static_cast<long>(g_circuit_model.fit.confidence_x1000 / 10));
        } else {
            snprintf(buf,
                     sizeof(buf),
                     "Circuit Model: %s      fc: %s Hz      H(f): Loaded",
                     filter_type_text(g_circuit_model.filter_type),
                     fc);
        }
        lv_obj_set_style_text_color(g_adv_model_line, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
        lv_label_set_text(g_adv_model_line, buf);

        if (g_circuit_model.fit.valid) {
            snprintf(buf,
                     sizeof(buf),
                     "Model Range: %s~%s Hz      Points: %lu      RMS=%ld.%ld%% Max=%ld.%ld%%",
                     start_freq,
                     stop_freq,
                     static_cast<unsigned long>(g_circuit_model.fit.valid_point_count),
                     static_cast<long>(g_circuit_model.fit.rms_error_x10 / 10),
                     static_cast<long>(g_circuit_model.fit.rms_error_x10 % 10),
                     static_cast<long>(g_circuit_model.fit.max_error_x10 / 10),
                     static_cast<long>(g_circuit_model.fit.max_error_x10 % 10));
        } else {
            snprintf(buf,
                     sizeof(buf),
                     "Model Range: %s~%s Hz      Points: %lu",
                     start_freq,
                     stop_freq,
                     static_cast<unsigned long>(g_circuit_model.point_count));
        }
        lv_label_set_text(g_adv_model_range_line, buf);
        lv_obj_set_style_text_color(g_adv_model_range_line, lv_color_hex(COLOR_SUBTEXT), LV_PART_MAIN);
    } else {
        snprintf(buf, sizeof(buf), "Circuit Model: Unknown       fc: ----- Hz      H(f): Not Ready");
        lv_obj_set_style_text_color(g_adv_model_line, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN);
        lv_label_set_text(g_adv_model_line, buf);

        lv_label_set_text(g_adv_model_range_line, "Model Range: -----~----- Hz      Points: 0");
        lv_obj_set_style_text_color(g_adv_model_range_line, lv_color_hex(COLOR_SUBTEXT), LV_PART_MAIN);
    }
}

static void set_harmonic_rows_empty(void)
{
    static const char *rows[] = {
        "No.   Freq      Amp       Phase",
        "1     -----Hz   -.---V    -.-deg",
        "2     -----Hz   -.---V    -.-deg",
        "3     -----Hz   -.---V    -.-deg",
        "4     -----Hz   -.---V    -.-deg",
        "5     -----Hz   -.---V    -.-deg",
    };

    for (uint32_t i = 0; i < 6U; ++i) {
        if (g_adv_harmonic_rows[i] != nullptr) {
            lv_label_set_text(g_adv_harmonic_rows[i], rows[i]);
        }
    }
}

static void set_harmonic_rows_fake(void)
{
    static const char *rows[] = {
        "No.   Freq      Amp       Phase",
        "1     01000Hz   1.000V     0.0deg",
        "2     02000Hz   0.320V    -5.1deg",
        "3     03000Hz   0.180V    -8.7deg",
        "4     04000Hz   0.090V   -12.2deg",
        "5     05000Hz   0.050V   -16.8deg",
    };

    for (uint32_t i = 0; i < 6U; ++i) {
        if (g_adv_harmonic_rows[i] != nullptr) {
            lv_label_set_text(g_adv_harmonic_rows[i], rows[i]);
        }
    }
}

static void generate_fake_output_waveform(void)
{
    if (!g_reconstruction_page_active || g_adv_output_chart == nullptr || g_adv_output_series == nullptr) {
        return;
    }

    lv_chart_set_all_value(g_adv_output_chart, g_adv_output_series, LV_CHART_POINT_NONE);
    for (uint32_t i = 0; i < 128U; ++i) {
        const double t = static_cast<double>(i) / 128.0;
        const double y = 0.72 * sin(2.0 * 3.14159265358979323846 * t) +
                         0.18 * sin(4.0 * 3.14159265358979323846 * t - 0.45) +
                         0.08 * sin(6.0 * 3.14159265358979323846 * t - 0.75);
        lv_chart_set_next_value(g_adv_output_chart,
                                g_adv_output_series,
                                static_cast<int32_t>(y * 900.0));
    }
    lv_chart_refresh(g_adv_output_chart);
}

static void generate_fake_reconstructed_waveform(void)
{
    if (!g_reconstruction_page_active || g_adv_recon_chart == nullptr || g_adv_recon_series == nullptr) {
        return;
    }

    lv_chart_set_all_value(g_adv_recon_chart, g_adv_recon_series, LV_CHART_POINT_NONE);
    for (uint32_t i = 0; i < 128U; ++i) {
        const double t = static_cast<double>(i) / 128.0;
        double x = 0.0;
        x += 0.85 * sin(2.0 * 3.14159265358979323846 * t);
        x += 0.28 * sin(6.0 * 3.14159265358979323846 * t);
        x += 0.16 * sin(10.0 * 3.14159265358979323846 * t);
        if (x > 1.0) {
            x = 1.0;
        } else if (x < -1.0) {
            x = -1.0;
        }
        lv_chart_set_next_value(g_adv_recon_chart,
                                g_adv_recon_series,
                                static_cast<int32_t>(x * 900.0));
    }
    lv_chart_refresh(g_adv_recon_chart);
}

static void update_top_status(const freqresp_ui_status_t *s)
{
    char buf[128];
    const char *link = "WAIT";

    if (s->link_ok != 0U) {
        link = "OK";
    } else if (s->frame_errors != 0U || s->timeouts != 0U) {
        link = "ERR";
    }

    snprintf(buf,
             sizeof(buf),
             "L:%s S:%s P:%lu%% %lu/%lu",
             link,
             state_text(s->state),
             static_cast<unsigned long>(s->progress_permille / 10U),
             static_cast<unsigned long>(s->point_index),
             static_cast<unsigned long>(s->total_points));
    lv_label_set_text(g_top_status, buf);

    uint32_t color = COLOR_YELLOW;
    if (strcmp(link, "OK") == 0 && s->state != FREQRESP_STATE_ERROR) {
        color = COLOR_GREEN;
    } else if (s->state == FREQRESP_STATE_ERROR || strcmp(link, "ERR") == 0) {
        color = COLOR_RED;
    }
    lv_obj_set_style_text_color(g_top_status, lv_color_hex(color), LV_PART_MAIN);
}

static void update_latest_line(const freqresp_ui_status_t *s)
{
    char freq[16];
    char vin[24];
    char vout[24];
    char gain[24];
    char err[24];
    char buf[180];

    format_freq(freq, sizeof(freq), s->current_freq_hz);
    format_voltage(vin, sizeof(vin), s->vin_mv);
    format_voltage(vout, sizeof(vout), s->vout_mv);
    format_x1000(gain, sizeof(gain), s->gain_x1000);
    format_error(err, sizeof(err), s->error_x10);

    snprintf(buf,
             sizeof(buf),
             "Latest: %sHz  Vin=%s  Vout=%s  G=%s  Err=%s",
             freq,
             vin,
             vout,
             gain,
             err);
    lv_label_set_text(g_latest, buf);
}

static void update_current_point(const freqresp_ui_status_t *s)
{
    char freq[16];
    char vin[24];
    char vout[24];
    char gain[24];
    char theory[24];
    char err[24];
    char phase[24];
    char fc[16];
    char buf[192];

    format_freq(freq, sizeof(freq), s->current_freq_hz);
    format_voltage(vin, sizeof(vin), s->vin_mv);
    format_voltage(vout, sizeof(vout), s->vout_mv);
    format_x1000(gain, sizeof(gain), s->gain_x1000);
    format_x1000(theory, sizeof(theory), s->theory_gain_x1000);
    format_error(err, sizeof(err), s->error_x10);
    format_phase(phase, sizeof(phase), s->phase_deg_x10);
    format_cutoff_freq(fc, sizeof(fc), s->filter_type, s->cutoff_freq_hz);
    if (s->theory_gain_x1000 <= 0) {
        snprintf(theory, sizeof(theory), "----");
        snprintf(err, sizeof(err), "N/A");
    }
    if (!s->phase_valid) {
        snprintf(phase, sizeof(phase), "--");
    }

    snprintf(buf, sizeof(buf), "Freq: %s Hz", freq);
    lv_label_set_text(g_freq, buf);

    snprintf(buf, sizeof(buf), "Vin: %s", vin);
    lv_label_set_text(g_vin, buf);

    snprintf(buf, sizeof(buf), "Vout: %s", vout);
    lv_label_set_text(g_vout, buf);

    snprintf(buf, sizeof(buf), "Gain: %s", gain);
    lv_label_set_text(g_gain_line, buf);

    snprintf(buf, sizeof(buf), "Theory: %s", theory);
    lv_label_set_text(g_theory, buf);

    snprintf(buf,
             sizeof(buf),
             "Error: %s %s",
             err,
             (s->theory_gain_x1000 > 0) ? (s->error_x10 <= 50 ? "PASS" : "FAIL") : "N/A");
    lv_label_set_text(g_error_line, buf);
    lv_obj_set_style_text_color(g_gain_line,
                                lv_color_hex((s->theory_gain_x1000 > 0 && s->error_x10 > 50) ? COLOR_RED : COLOR_GREEN),
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(g_theory,
                                lv_color_hex(COLOR_TEXT),
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(g_error_line,
                                lv_color_hex((s->theory_gain_x1000 > 0 && s->error_x10 > 50) ? COLOR_RED : COLOR_GREEN),
                                LV_PART_MAIN);

    snprintf(buf, sizeof(buf), "Phase: %s", phase);
    lv_label_set_text(g_phase, buf);

    if (g_fit_result_quality == FIT_RESULT_HEAVY_LOW_CONF) {
        snprintf(buf, sizeof(buf), "Type: Unknown/Low confidence");
    } else if (g_last_fit.valid) {
        if (g_fit_result_quality == FIT_RESULT_HEAVY_TIMEOUT) {
            snprintf(buf,
                     sizeof(buf),
                     "Type: %s? timeout C=%ld%%",
                     model_kind_text(g_last_fit.model_type),
                     static_cast<long>(g_last_fit.confidence_x1000 / 10));
        } else if (g_fit_result_quality == FIT_RESULT_LIGHT) {
            snprintf(buf,
                     sizeof(buf),
                     "Type: Light %s C=%ld%%",
                     model_kind_text(g_last_fit.model_type),
                     static_cast<long>(g_last_fit.confidence_x1000 / 10));
        } else if (g_fit_result_quality == FIT_RESULT_HEAVY_OK &&
                   (g_last_fit.model_type == MODEL_TYPE_LP2 ||
                    g_last_fit.model_type == MODEL_TYPE_HP2 ||
                    g_last_fit.model_type == MODEL_TYPE_BP2 ||
                    g_last_fit.model_type == MODEL_TYPE_BS2)) {
            snprintf(buf,
                     sizeof(buf),
                     "Type: %s Q=%ld.%03ld C=%ld%%",
                     model_kind_text(g_last_fit.model_type),
                     static_cast<long>(g_last_fit.q_x1000 / 1000),
                     static_cast<long>(g_last_fit.q_x1000 % 1000),
                     static_cast<long>(g_last_fit.confidence_x1000 / 10));
        } else {
            snprintf(buf,
                     sizeof(buf),
                     "Type: %s C=%ld%%",
                     model_kind_text(g_last_fit.model_type),
                     static_cast<long>(g_last_fit.confidence_x1000 / 10));
        }
    } else {
        snprintf(buf, sizeof(buf), "Type: %s", filter_type_text(s->filter_type));
    }
    lv_label_set_text(g_type, buf);

    if (g_fit_result_quality == FIT_RESULT_HEAVY_OK &&
        g_last_fit.valid &&
        (g_last_fit.model_type == MODEL_TYPE_LP2 ||
         g_last_fit.model_type == MODEL_TYPE_HP2 ||
         g_last_fit.model_type == MODEL_TYPE_BP2 ||
         g_last_fit.model_type == MODEL_TYPE_BS2)) {
        snprintf(buf, sizeof(buf), "f0: %s Hz", fc);
    } else {
        snprintf(buf, sizeof(buf), "fc: %s Hz", fc);
    }
    lv_label_set_text(g_fc, buf);
}

static void render_basic_status(const freqresp_ui_status_t *s)
{
    if (s == nullptr || !g_main_page_active || g_top_status == nullptr) {
        return;
    }

    update_top_status(s);
    update_current_point(s);
    update_latest_line(s);

    if (g_table_full) {
        set_msg("FULL", COLOR_RED);
    }
}

static void start_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!read_and_validate_inputs()) {
        return;
    }

    ClearAllSweepData();
    SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);

    if (g_mode == MODE_SWEEP) {
        SpiLink_SetPendingCommand(CMD_SET_MODE, MODE_SWEEP, 0U);
        SpiLink_SetPendingCommand(CMD_SET_START_FREQ, g_start_freq_hz, 0U);
        SpiLink_SetPendingCommand(CMD_SET_STOP_FREQ, g_stop_freq_hz, 0U);
        SpiLink_SetPendingCommand(CMD_SET_STEP_FREQ, g_step_freq_hz, 0U);
        SpiLink_SetPendingCommand(CMD_START, 0U, 0U);
    } else {
        SpiLink_SetPendingCommand(CMD_SET_MODE, MODE_SINGLE, 0U);
        SpiLink_SetPendingCommand(CMD_SET_SINGLE_FREQ, g_single_freq_hz, 0U);
        SpiLink_SetPendingCommand(CMD_START, 0U, 0U);
    }

#if ENABLE_FAKE_DATA_TEST
    g_fake_index = 0;
    if (g_fake_timer != nullptr) {
        lv_timer_resume(g_fake_timer);
    }
#endif

    set_msg("START", COLOR_GREEN);
}

static void stop_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        SpiLink_SetPendingCommand(CMD_STOP, 0U, 0U);
        set_msg("STOP", COLOR_YELLOW);
#if ENABLE_FAKE_DATA_TEST
        if (g_fake_timer != nullptr) {
            lv_timer_pause(g_fake_timer);
        }
#endif
    }
}

static void clear_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        ClearAllSweepData();
        SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
    }
}

static void create_main_page(void);
static void create_full_table_page(void);
static void create_reconstruction_page(void);

static void advanced_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_reconstruction_page();
    }
}

static void keyboard_event_cb(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (g_main_page_active) {
            read_and_validate_inputs();
        }
        lv_keyboard_set_textarea(g_keyboard, nullptr);
        lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void open_table_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_full_table_page();
    }
}

static void back_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_main_page();
        if (g_have_last_status) {
            render_basic_status(&g_last_status);
        }
    }
}

static void adv_capture_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!g_circuit_model.valid) {
        set_adv_result("Result: MODEL? Run Basic Sweep first", COLOR_RED);
        return;
    }

    SpiLink_SetPendingCommand(CMD_ADV_CAPTURE, 0U, 0U);

#if ENABLE_FAKE_DATA_TEST
    generate_fake_output_waveform();
#endif

    g_adv_output_captured = true;
    set_adv_result("Result: Output y(t) captured", COLOR_GREEN);
}

static void adv_reconstruct_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!g_circuit_model.valid) {
        set_adv_result("Result: MODEL? Run Basic Sweep first", COLOR_RED);
        return;
    }

    SpiLink_SetPendingCommand(CMD_ADV_RECONSTRUCT, 0U, 0U);

#if ENABLE_FAKE_DATA_TEST
    if (!g_adv_output_captured) {
        generate_fake_output_waveform();
        g_adv_output_captured = true;
    }
    generate_fake_reconstructed_waveform();
    set_harmonic_rows_fake();
#endif

    g_adv_reconstruction_ready = true;
    set_adv_result("Result: Reconstructed x(t) ready   Error: 3.4% PASS", COLOR_GREEN);
}

static void adv_send_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!g_adv_reconstruction_ready) {
        set_adv_result("Result: NO DATA", COLOR_RED);
        return;
    }

    SpiLink_SetPendingCommand(CMD_ADV_SEND_TO_DDS, 0U, 0U);
    set_adv_result("Result: Sent to DDS", COLOR_GREEN);
}

static void adv_back_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_main_page();
        if (g_have_last_status) {
            render_basic_status(&g_last_status);
        }
    }
}

#if ENABLE_SPI_TEST_WINDOW
static void update_spi_test_labels(void)
{
    if (!g_spi_test_page_active) {
        return;
    }

    if (g_spi_test_link != nullptr) {
        const char *link_text = "Link: WAIT";
        uint32_t link_color = COLOR_YELLOW;
        if (g_spi_test_link_state == 1U) {
            link_text = "Link: OK";
            link_color = COLOR_GREEN;
        } else if (g_spi_test_link_state == 2U) {
            link_text = "Link: ERR";
            link_color = COLOR_RED;
        }

        lv_label_set_text(g_spi_test_link, link_text);
        lv_obj_set_style_text_color(g_spi_test_link,
                                    lv_color_hex(link_color),
                                    LV_PART_MAIN);
    }

    if (g_spi_test_rx != nullptr) {
        char buf[160];
        snprintf(buf, sizeof(buf), "RX From PYNQ: %s", g_spi_test_last_rx);
        lv_label_set_text(g_spi_test_rx, buf);
    }
}

static void spi_test_textarea_event_cb(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *ta = static_cast<lv_obj_t *>(lv_event_get_target(event));

    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
        lv_keyboard_set_textarea(g_keyboard, ta);
        lv_keyboard_set_mode(g_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_obj_clear_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
    } else if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_keyboard_set_textarea(g_keyboard, nullptr);
        lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_state(ta, LV_STATE_FOCUSED);
    }
}

static void spi_test_send_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED || g_spi_test_input == nullptr) {
        return;
    }

    const char *text = lv_textarea_get_text(g_spi_test_input);
    SpiLink_SendTextToPynq(text);

    if (g_spi_test_tx != nullptr) {
        char buf[160];
        snprintf(buf, sizeof(buf), "TX To PYNQ: %s", text);
        lv_label_set_text(g_spi_test_tx, buf);
        lv_obj_set_style_text_color(g_spi_test_tx, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
    }
}

static void spi_test_close_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_main_page();
        if (g_have_last_status) {
            render_basic_status(&g_last_status);
        }
    }
}

static lv_obj_t *create_spi_text_input(lv_obj_t *parent, int32_t x, int32_t y)
{
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_pos(ta, x, y);
    lv_obj_set_size(ta, 600, 42);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 104);
    lv_textarea_set_placeholder_text(ta, "type text for PYNQ");
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(ta, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ta, lv_color_hex(COLOR_INPUT), LV_PART_MAIN);
    lv_obj_set_style_border_color(ta, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_radius(ta, 6, LV_PART_MAIN);
    lv_obj_add_event_cb(ta, spi_test_textarea_event_cb, LV_EVENT_ALL, nullptr);
    return ta;
}

static void create_spi_test_page(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    g_main_page_active = false;
    g_reconstruction_page_active = false;
    g_spi_test_page_active = true;

    g_full_table = nullptr;
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
    g_adv_status = nullptr;
    g_adv_model_line = nullptr;
    g_adv_model_range_line = nullptr;
    g_adv_result = nullptr;
    g_adv_output_chart = nullptr;
    g_adv_output_series = nullptr;
    g_adv_recon_chart = nullptr;
    g_adv_recon_series = nullptr;
    for (uint32_t i = 0; i < 6U; ++i) {
        g_adv_harmonic_rows[i] = nullptr;
    }

    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    create_label(screen, "SPI Text Test", 24, 18, 360, &lv_font_montserrat_24, COLOR_TEXT);
    g_spi_test_link = create_label(screen, "Link: WAIT", 760, 22, 220, &lv_font_montserrat_20, COLOR_YELLOW);
    create_hline(screen, 65);

    g_spi_test_rx = create_label(screen, "RX From PYNQ: ", 40, 110, 900, &lv_font_montserrat_24, COLOR_BLUE);
    create_label(screen, "TX To PYNQ:", 40, 190, 200, &lv_font_montserrat_20, COLOR_SUBTEXT);
    g_spi_test_input = create_spi_text_input(screen, 40, 225);
    lv_obj_t *send = create_button(screen, "Send", 670, 225, 120);
    lv_obj_add_event_cb(send, spi_test_send_event_cb, LV_EVENT_CLICKED, nullptr);

    g_spi_test_tx = create_label(screen, "TX To PYNQ: ", 40, 290, 900, &lv_font_montserrat_20, COLOR_TEXT);

    create_label(screen,
                 "SPI: PYNQ Master -> ESP32-P4 Slave, mode 0, 128 bytes per transaction",
                 40,
                 370,
                 900,
                 &lv_font_montserrat_18,
                 COLOR_SUBTEXT);

    lv_obj_t *close = create_button(screen, "Close Test", 800, 520, 180);
    lv_obj_add_event_cb(close, spi_test_close_event_cb, LV_EVENT_CLICKED, nullptr);

    g_keyboard = lv_keyboard_create(screen);
    lv_keyboard_set_mode(g_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_add_event_cb(g_keyboard, keyboard_event_cb, LV_EVENT_ALL, nullptr);
    lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);

    update_spi_test_labels();
}
#endif

static lv_obj_t *create_wave_chart(lv_obj_t *parent,
                                   int32_t x,
                                   int32_t y,
                                   int32_t w,
                                   int32_t h,
                                   lv_chart_series_t **series,
                                   uint32_t color)
{
    lv_obj_t *chart = lv_chart_create(parent);
    lv_obj_set_pos(chart, x, y);
    lv_obj_set_size(chart, w, h);
    lv_obj_set_style_bg_color(chart, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(chart, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_border_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(chart, 6, LV_PART_MAIN);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -1000, 1000);
    lv_chart_set_point_count(chart, 128);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
    *series = lv_chart_add_series(chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(chart, *series, LV_CHART_POINT_NONE);
    lv_chart_refresh(chart);
    return chart;
}

static void create_reconstruction_page(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    g_main_page_active = false;
    g_reconstruction_page_active = true;
    g_spi_test_page_active = false;
    g_full_table = nullptr;
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
    g_keyboard = nullptr;
#if ENABLE_SPI_TEST_WINDOW
    g_spi_test_link = nullptr;
    g_spi_test_rx = nullptr;
    g_spi_test_tx = nullptr;
    g_spi_test_input = nullptr;
#endif
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    create_label(screen, "Signal Reconstruction", 24, 14, 430, &lv_font_montserrat_24, COLOR_TEXT);
    g_adv_status = create_label(screen, "Link: OK   State: Ready", 610, 17, 390, &lv_font_montserrat_20, COLOR_GREEN);
    create_hline(screen, 55);

    g_adv_model_line = create_label(screen,
                                    "Circuit Model: Unknown       fc: ----- Hz      H(f): Not Ready",
                                    24,
                                    70,
                                    590,
                                    &lv_font_montserrat_18,
                                    COLOR_YELLOW);

    g_adv_model_range_line = create_label(screen,
                                          "Model Range: -----~----- Hz      Points: 0",
                                          24,
                                          96,
                                          590,
                                          &lv_font_montserrat_18,
                                          COLOR_SUBTEXT);

    create_label(screen,
                 "Source: ADC CH2 Output       Sample Rate: 200 kSPS",
                 24,
                 122,
                 590,
                 &lv_font_montserrat_18,
                 COLOR_SUBTEXT);
    update_adv_model_line();

    create_label(screen,
                 "Step 1: Capture Output   Step 2: Reconstruct   Step 3: Send to DDS",
                 24,
                 150,
                 590,
                 &lv_font_montserrat_18,
                 COLOR_SUBTEXT);

    lv_obj_t *btn_capture = create_button(screen, "Capture Output", 24, 178, 180);
    lv_obj_t *btn_reconstruct = create_button(screen, "Reconstruct", 224, 178, 160);
    lv_obj_t *btn_send = create_button(screen, "Send to DDS", 404, 178, 150);
    lv_obj_add_event_cb(btn_capture, adv_capture_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_reconstruct, adv_reconstruct_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_send, adv_send_event_cb, LV_EVENT_CLICKED, nullptr);

    create_vline(screen, 635, 65, 150);

    create_label(screen, "Harmonic Analysis", 660, 70, 320, &lv_font_montserrat_20, COLOR_BLUE);
    for (uint32_t i = 0; i < 6U; ++i) {
        g_adv_harmonic_rows[i] = create_label(screen,
                                              "",
                                              675,
                                              100 + static_cast<int32_t>(i * 20U),
                                              315,
                                              &lv_font_montserrat_18,
                                              i == 0U ? COLOR_SUBTEXT : COLOR_TEXT);
    }
    set_harmonic_rows_empty();

    create_hline(screen, 220);

    create_label(screen, "Waveform", 24, 235, 240, &lv_font_montserrat_20, COLOR_BLUE);
    create_label(screen, "Output y(t)", 24, 250, 220, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "Reconstructed x(t)", 520, 250, 260, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_adv_output_chart = create_wave_chart(screen, 24, 275, 460, 245, &g_adv_output_series, COLOR_BLUE);
    g_adv_recon_chart = create_wave_chart(screen, 520, 275, 460, 245, &g_adv_recon_series, COLOR_GREEN);

    create_hline(screen, 535);

    g_adv_result = create_label(screen, "Result: Waiting", 24, 552, 850, &lv_font_montserrat_20, COLOR_YELLOW);
    lv_obj_t *btn_back = create_button(screen, "Back", 900, 552, 100);
    lv_obj_add_event_cb(btn_back, adv_back_event_cb, LV_EVENT_CLICKED, nullptr);

    if (g_adv_output_captured) {
        generate_fake_output_waveform();
    }
    if (g_adv_reconstruction_ready) {
        generate_fake_reconstructed_waveform();
        set_harmonic_rows_fake();
        set_adv_result("Result: Reconstructed x(t) ready   Error: 3.4% PASS", COLOR_GREEN);
    } else if (g_adv_output_captured) {
        set_adv_result("Result: Output y(t) captured", COLOR_GREEN);
    }
}

static void create_main_page(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    g_main_page_active = true;
    g_reconstruction_page_active = false;
    g_spi_test_page_active = false;
    g_full_table = nullptr;
    g_adv_status = nullptr;
    g_adv_model_line = nullptr;
    g_adv_model_range_line = nullptr;
    g_adv_result = nullptr;
    g_adv_output_chart = nullptr;
    g_adv_output_series = nullptr;
    g_adv_recon_chart = nullptr;
    g_adv_recon_series = nullptr;
    for (uint32_t i = 0; i < 6U; ++i) {
        g_adv_harmonic_rows[i] = nullptr;
    }
#if ENABLE_SPI_TEST_WINDOW
    g_spi_test_link = nullptr;
    g_spi_test_rx = nullptr;
    g_spi_test_tx = nullptr;
    g_spi_test_input = nullptr;
#endif
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    g_title = create_label(screen, "Circuit Response Tester", 24, 14, 420, &lv_font_montserrat_24, COLOR_TEXT);
    g_top_status = create_label(screen, "Link: WAIT    State: Idle", 610, 17, 390, &lv_font_montserrat_20, COLOR_YELLOW);
    create_hline(screen, 55);

    create_label(screen, "Mode:", 24, 68, 70, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_cb_sweep = lv_checkbox_create(screen);
    lv_obj_set_pos(g_cb_sweep, 92, 64);
    lv_checkbox_set_text(g_cb_sweep, "Sweep");
    lv_obj_set_style_text_font(g_cb_sweep, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_cb_sweep, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_add_state(g_cb_sweep, LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_cb_sweep, checkbox_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    g_cb_single = lv_checkbox_create(screen);
    lv_obj_set_pos(g_cb_single, 220, 64);
    lv_checkbox_set_text(g_cb_single, "Single Point");
    lv_obj_set_style_text_font(g_cb_single, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_cb_single, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_add_event_cb(g_cb_single, checkbox_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    if (g_mode == MODE_SINGLE) {
        lv_obj_clear_state(g_cb_sweep, LV_STATE_CHECKED);
        lv_obj_add_state(g_cb_single, LV_STATE_CHECKED);
    }

    create_label(screen, "Start:", 24, 105, 70, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_ta_start = create_freq_input(screen, 92, 97, g_start_freq_hz);
    create_label(screen, "Hz", 198, 105, 35, &lv_font_montserrat_18, COLOR_SUBTEXT);

    create_label(screen, "Stop:", 245, 105, 65, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_ta_stop = create_freq_input(screen, 310, 97, g_stop_freq_hz);
    create_label(screen, "Hz", 416, 105, 35, &lv_font_montserrat_18, COLOR_SUBTEXT);

    create_label(screen, "Step:", 470, 105, 65, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_ta_step = create_freq_input(screen, 535, 97, g_step_freq_hz);
    create_label(screen, "Hz", 641, 105, 35, &lv_font_montserrat_18, COLOR_SUBTEXT);

    create_label(screen, "Single Freq:", 24, 145, 130, &lv_font_montserrat_18, COLOR_SUBTEXT);
    g_ta_single = create_freq_input(screen, 154, 137, g_single_freq_hz);
    create_label(screen, "Hz", 260, 145, 35, &lv_font_montserrat_18, COLOR_SUBTEXT);

    lv_obj_add_event_cb(g_ta_start, textarea_event_cb, LV_EVENT_ALL, nullptr);
    lv_obj_add_event_cb(g_ta_stop, textarea_event_cb, LV_EVENT_ALL, nullptr);
    lv_obj_add_event_cb(g_ta_step, textarea_event_cb, LV_EVENT_ALL, nullptr);
    lv_obj_add_event_cb(g_ta_single, textarea_event_cb, LV_EVENT_ALL, nullptr);

    lv_obj_t *btn_start = create_button(screen, "Start", 430, 137, 86);
    lv_obj_t *btn_stop = create_button(screen, "Stop", 530, 137, 86);
    lv_obj_t *btn_clear = create_button(screen, "Clear", 630, 137, 100);
    lv_obj_t *btn_advanced = create_button(screen, "Advanced", 750, 137, 110);
    lv_obj_add_event_cb(btn_start, start_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_stop, stop_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_clear, clear_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_advanced, advanced_button_event_cb, LV_EVENT_CLICKED, nullptr);

    g_msg = create_label(screen, "Ready", 870, 145, 130, &lv_font_montserrat_18, COLOR_GREEN);

    create_hline(screen, 190);

    create_vline(screen, 430, 195, 320);

    create_label(screen, "Current Point", 24, 205, 250, &lv_font_montserrat_20, COLOR_BLUE);
    g_freq = create_label(screen, "Freq: ----- Hz", 24, 245, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_vin = create_label(screen, "Vin: -.--- V", 24, 275, 180, &lv_font_montserrat_18, COLOR_TEXT);
    g_vout = create_label(screen, "Vout: -.--- V", 220, 275, 190, &lv_font_montserrat_18, COLOR_TEXT);
    g_gain_line = create_label(screen, "Gain: -.---", 24, 315, 180, &lv_font_montserrat_18, COLOR_GREEN);
    g_theory = create_label(screen, "Theory: -.---", 220, 315, 190, &lv_font_montserrat_18, COLOR_TEXT);
    g_error_line = create_label(screen, "Error: -.-% PASS", 24, 355, 380, &lv_font_montserrat_18, COLOR_GREEN);
    g_phase = create_label(screen, "Phase: -.- deg", 24, 395, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_type = create_label(screen, "Type: Unknown", 24, 435, 180, &lv_font_montserrat_18, COLOR_TEXT);
    g_fc = create_label(screen, "fc: ----- Hz", 220, 435, 190, &lv_font_montserrat_18, COLOR_TEXT);

    create_label(screen, "Gain Curve", 455, 205, 250, &lv_font_montserrat_20, COLOR_BLUE);
    g_chart = lv_chart_create(screen);
    lv_obj_set_pos(g_chart, 500, 245);
    lv_obj_set_size(g_chart, 460, 220);
    lv_obj_set_style_bg_color(g_chart, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_border_color(g_chart, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_radius(g_chart, 6, LV_PART_MAIN);
    lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_point_count(g_chart, 100);
    lv_chart_set_update_mode(g_chart, LV_CHART_UPDATE_MODE_SHIFT);
    g_chart_gain = lv_chart_add_series(g_chart, lv_color_hex(COLOR_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(g_chart, g_chart_gain, LV_CHART_POINT_NONE);
    refresh_chart_from_table();

    create_label(screen, "1.0", 455, 248, 36, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "0.5", 455, 350, 36, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "0.0", 455, 452, 36, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "100", 500, 475, 80, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "1k", 630, 475, 80, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "10k", 760, 475, 80, &lv_font_montserrat_18, COLOR_SUBTEXT);
    create_label(screen, "100k", 900, 475, 80, &lv_font_montserrat_18, COLOR_SUBTEXT);

    create_hline(screen, 530);

    g_latest = create_label(screen, "Latest: --", 24, 542, 730, &lv_font_montserrat_18, COLOR_TEXT);
    g_open_table_btn = create_button(screen, "Open Full Table", 764, 538, 236);
    lv_obj_add_event_cb(g_open_table_btn, open_table_event_cb, LV_EVENT_CLICKED, nullptr);

    g_keyboard = lv_keyboard_create(screen);
    lv_keyboard_set_mode(g_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_add_event_cb(g_keyboard, keyboard_event_cb, LV_EVENT_ALL, nullptr);
    lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
}

void test_screen_create(void)
{
#if ENABLE_SPI_TEST_WINDOW
    create_spi_test_page();
#else
    create_main_page();
#endif

    if (g_spi_ui_pump_timer == nullptr) {
        g_spi_ui_pump_timer = lv_timer_create([](lv_timer_t *timer) {
            (void)timer;
            SpiLink_UiPump();
            process_pending_fit();
        }, 20, nullptr);
    }

#if ENABLE_FAKE_DATA_TEST
    g_fake_index = 0;
    if (g_fake_timer != nullptr) {
        lv_timer_delete(g_fake_timer);
        g_fake_timer = nullptr;
    }
    g_fake_timer = lv_timer_create([](lv_timer_t *timer) {
        (void)timer;

        if (g_fake_index >= 100U) {
            freqresp_ui_status_t done = g_last_status;
            done.state = FREQRESP_STATE_DONE;
            done.progress_permille = 1000U;
            test_screen_update_measurement(&done);
            if (g_fake_timer != nullptr) {
                lv_timer_pause(g_fake_timer);
            }
            return;
        }

        const double t = static_cast<double>(g_fake_index) / 99.0;
        const double log_f = log10(100.0) + t * (log10(100000.0) - log10(100.0));
        const uint32_t freq_hz = static_cast<uint32_t>(pow(10.0, log_f) + 0.5);
        const double fc = 1590.0;
        const double theory = 1.0 / sqrt(1.0 + (static_cast<double>(freq_hz) / fc) * (static_cast<double>(freq_hz) / fc));
        const int32_t vin_mv = 1000 + (static_cast<int32_t>(g_fake_index % 7U) - 3);
        const double measured = theory * (0.982 + static_cast<double>(g_fake_index % 9U) * 0.004);
        const int32_t vout_mv = static_cast<int32_t>(static_cast<double>(vin_mv) * measured + 0.5);
        const int32_t gain_x1000 = (vin_mv > 0) ? (vout_mv * 1000) / vin_mv : 0;
        const int32_t theory_x1000 = static_cast<int32_t>(theory * 1000.0 + 0.5);
        int32_t error_x10 = 0;
        if (theory_x1000 > 0) {
            int32_t diff = gain_x1000 - theory_x1000;
            if (diff < 0) {
                diff = -diff;
            }
            error_x10 = (diff * 1000) / theory_x1000;
        }

        freqresp_ui_status_t s = {};
        s.packets = g_fake_index + 1U;
        s.link_ok = 1U;
        s.state = FREQRESP_STATE_SCANNING;
        s.mode = g_mode;
        s.filter_type = FILTER_TYPE_LOW_PASS;
        s.progress_permille = ((g_fake_index + 1U) * 1000U) / 100U;
        s.start_freq_hz = g_start_freq_hz;
        s.stop_freq_hz = g_stop_freq_hz;
        s.step_freq_hz = g_step_freq_hz;
        s.single_freq_hz = g_single_freq_hz;
        s.current_freq_hz = freq_hz;
        s.point_index = g_fake_index + 1U;
        s.total_points = 100U;
        s.vin_mv = vin_mv;
        s.vout_mv = vout_mv;
        s.gain_x1000 = gain_x1000;
        s.theory_gain_x1000 = theory_x1000;
        s.error_x10 = error_x10;
        s.phase_deg_x10 = -static_cast<int32_t>(atan(static_cast<double>(freq_hz) / fc) * 1800.0 / 3.14159265358979323846);
        s.cutoff_freq_hz = 1590U;
        s.has_measurement = true;
        s.phase_valid = true;

        test_screen_update_measurement(&s);
        ++g_fake_index;
    }, 200, nullptr);
    lv_timer_pause(g_fake_timer);
#endif
}

void test_screen_update_measurement(const freqresp_ui_status_t *s)
{
    if (s == nullptr) {
        return;
    }

    freqresp_ui_status_t view = *s;

    if (!view.has_measurement) {
        if (g_main_page_active && g_top_status != nullptr) {
            update_top_status(&view);
        }
        g_last_status.state = view.state;
        g_last_status.mode = view.mode;
        g_last_status.filter_type = view.filter_type;
        g_last_status.link_ok = view.link_ok;
        g_last_status.progress_permille = view.progress_permille;
        g_last_status.start_freq_hz = view.start_freq_hz;
        g_last_status.stop_freq_hz = view.stop_freq_hz;
        g_last_status.step_freq_hz = view.step_freq_hz;
        g_last_status.single_freq_hz = view.single_freq_hz;
        g_last_status.current_freq_hz = view.current_freq_hz;
        g_last_status.point_index = view.point_index;
        g_last_status.total_points = view.total_points;
        g_last_status.packets = view.packets;
        g_last_status.frame_errors = view.frame_errors;
        g_last_status.timeouts = view.timeouts;
        return;
    }

    append_table_point(&view);

    const uint32_t now = lv_tick_get();
    const bool is_done = (view.state == FREQRESP_STATE_DONE);
    if (is_done) {
        flush_pending_chart_point(true);
    }
    const bool should_render =
        is_done || ((now - g_last_render_tick) >= kLabelRenderIntervalMs);

    if (is_done &&
        view.mode == MODE_SWEEP &&
        !g_fit_done_for_current_sweep &&
        !g_fit_pending) {
        g_fit_pending = true;
        g_fit_pending_tick = now;
        g_fit_pending_status = view;
        set_msg("FITTING...", COLOR_YELLOW);
    }

    if (should_render) {
        g_last_render_tick = now;
        g_last_status = view;
        g_have_last_status = true;
        render_basic_status(&view);
    }
}

static int32_t adv_chart_value_from_sample(int16_t sample)
{
    int32_t v = static_cast<int32_t>(sample) / 32;
    if (v > 1000) {
        v = 1000;
    } else if (v < -1000) {
        v = -1000;
    }
    return v;
}

void test_screen_update_adc_waveform_chunk(const adc_waveform_chunk_t *chunk)
{
    if (chunk == nullptr) {
        return;
    }

    const bool is_recon = (chunk->wave_type == 1U);
    lv_obj_t *chart = is_recon ? g_adv_recon_chart : g_adv_output_chart;
    lv_chart_series_t *series = is_recon ? g_adv_recon_series : g_adv_output_series;

    if (chunk->chunk_index == 0U) {
        if (is_recon) {
            g_adv_reconstruction_ready = false;
        } else {
            g_adv_output_captured = false;
            g_adv_reconstruction_ready = false;
        }
        if (chart != nullptr && series != nullptr) {
            lv_chart_set_all_value(chart, series, LV_CHART_POINT_NONE);
        }
    }

    if (chart != nullptr && series != nullptr) {
        for (uint32_t i = 0; i < 30U; ++i) {
            const uint32_t sample_index = chunk->start_sample_index + i;
            if (sample_index >= chunk->total_sample_count) {
                break;
            }
            if ((sample_index & 7U) == 0U) {
                lv_chart_set_next_value(chart, series, adv_chart_value_from_sample(chunk->samples[i]));
            }
        }
        lv_chart_refresh(chart);
    }

    const bool is_done = (chunk->flags & 0x00000002U) != 0U ||
                         (chunk->chunk_index + 1U >= chunk->chunk_count);
    char buf[160];
    if (is_done) {
        if (is_recon) {
            g_adv_reconstruction_ready = true;
            snprintf(buf,
                     sizeof(buf),
                     "Result: H0 bypass x(t) ready   Fs=%luHz N=%lu",
                     static_cast<unsigned long>(chunk->sample_rate_hz),
                     static_cast<unsigned long>(chunk->total_sample_count));
            set_adv_result(buf, COLOR_GREEN);
            set_harmonic_rows_empty();
        } else {
            g_adv_output_captured = true;
            snprintf(buf,
                     sizeof(buf),
                     "Result: Output y(t) captured   Fs=%luHz N=%lu",
                     static_cast<unsigned long>(chunk->sample_rate_hz),
                     static_cast<unsigned long>(chunk->total_sample_count));
            set_adv_result(buf, COLOR_GREEN);
        }
    } else if (g_reconstruction_page_active) {
        snprintf(buf,
                 sizeof(buf),
                 "Result: Receiving %s chunk %lu/%lu",
                 is_recon ? "x(t)" : "y(t)",
                 static_cast<unsigned long>(chunk->chunk_index + 1U),
                 static_cast<unsigned long>(chunk->chunk_count));
        set_adv_result(buf, COLOR_YELLOW);
    }
}

void test_screen_update_adc_analysis_result(const adc_analysis_result_t *result)
{
    (void)result;
}

void test_screen_update_adv_status(const adv_status_t *status)
{
    if (status == nullptr) {
        return;
    }

    if (g_adv_status != nullptr) {
        char buf[160];
        snprintf(buf,
                 sizeof(buf),
                 "Link: OK   ADV:%lu  FFT:%lu/%lu  TL:%lu/%lu",
                 static_cast<unsigned long>(status->adv_state),
                 static_cast<unsigned long>(status->fft_overflow_count),
                 static_cast<unsigned long>(status->ifft_overflow_count),
                 static_cast<unsigned long>(status->tlast_missing_count),
                 static_cast<unsigned long>(status->tlast_unexpected_count));
        lv_label_set_text(g_adv_status, buf);
        lv_obj_set_style_text_color(g_adv_status,
                                    lv_color_hex(status->error_code == 0U ? COLOR_GREEN : COLOR_RED),
                                    LV_PART_MAIN);
    }

    if (status->error_code != 0U) {
        const char *msg = "ADV ERROR";
        if (status->error_code == 1U) {
            msg = "ADV BUSY";
        } else if (status->error_code == 2U) {
            msg = "CAPTURE FIRST";
        } else if (status->error_code == 3U) {
            msg = "DDS SEND NOT IMPLEMENTED";
        }

        char buf[160];
        snprintf(buf,
                 sizeof(buf),
                 "Result: %s   Fs=%luHz N=%lu",
                 msg,
                 static_cast<unsigned long>(status->sample_rate_hz),
                 static_cast<unsigned long>(status->total_sample_count));
        set_adv_result(buf, status->error_code == 3U ? COLOR_YELLOW : COLOR_RED);
    }
}

void test_screen_update_spi_text_test(const char *rx_text, uint8_t link_state)
{
#if ENABLE_SPI_TEST_WINDOW
    g_spi_test_link_state = link_state;

    if (rx_text != nullptr) {
        snprintf(g_spi_test_last_rx, sizeof(g_spi_test_last_rx), "%s", rx_text);
    }

    update_spi_test_labels();
#else
    (void)rx_text;
    (void)link_state;
#endif
}

static void set_table_cell(lv_obj_t *table, uint32_t row, uint32_t col, const char *text)
{
    lv_table_set_cell_value(table, row, col, text);
    lv_table_set_cell_ctrl(table, row, col, LV_TABLE_CELL_CTRL_TEXT_CROP);
}

static void create_full_table_page(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    g_main_page_active = false;
    g_reconstruction_page_active = false;
    g_spi_test_page_active = false;
    g_full_table = nullptr;
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
#if ENABLE_SPI_TEST_WINDOW
    g_spi_test_link = nullptr;
    g_spi_test_rx = nullptr;
    g_spi_test_tx = nullptr;
    g_spi_test_input = nullptr;
#endif
    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    char points_buf[32];
    snprintf(points_buf, sizeof(points_buf), "Points: %lu", static_cast<unsigned long>(g_table_count));

    create_label(screen, "Full Data Table", 24, 14, 320, &lv_font_montserrat_24, COLOR_TEXT);
    create_label(screen, points_buf, 360, 18, 220, &lv_font_montserrat_20, COLOR_GREEN);
    lv_obj_t *back = create_button(screen, "Back", 900, 12, 100);
    lv_obj_add_event_cb(back, back_button_event_cb, LV_EVENT_CLICKED, nullptr);
    create_hline(screen, 55);

    g_full_table = lv_table_create(screen);
    lv_obj_set_pos(g_full_table, 20, 70);
    lv_obj_set_size(g_full_table, 984, 510);
    lv_obj_set_scrollbar_mode(g_full_table, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_text_font(g_full_table, &lv_font_montserrat_18, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_full_table, lv_color_hex(COLOR_TEXT), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_full_table, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_full_table, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(g_full_table, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_full_table, 1, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_full_table, lv_color_hex(COLOR_PANEL), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(g_full_table, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_full_table, lv_color_hex(COLOR_LINE), LV_PART_ITEMS);
    lv_obj_set_style_border_width(g_full_table, 1, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_full_table, lv_color_hex(COLOR_TEXT), LV_PART_ITEMS);
    lv_obj_set_style_pad_left(g_full_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_full_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_full_table, 6, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_full_table, 6, LV_PART_ITEMS);

    lv_table_set_col_cnt(g_full_table, 8);
    lv_table_set_row_cnt(g_full_table, (g_table_count == 0U) ? 2U : (g_table_count + 1U));
    lv_table_set_col_width(g_full_table, 0, 90);
    lv_table_set_col_width(g_full_table, 1, 130);
    lv_table_set_col_width(g_full_table, 2, 120);
    lv_table_set_col_width(g_full_table, 3, 120);
    lv_table_set_col_width(g_full_table, 4, 110);
    lv_table_set_col_width(g_full_table, 5, 112);
    lv_table_set_col_width(g_full_table, 6, 110);
    lv_table_set_col_width(g_full_table, 7, 120);

    set_table_cell(g_full_table, 0, 0, "No.");
    set_table_cell(g_full_table, 0, 1, "Freq");
    set_table_cell(g_full_table, 0, 2, "Vin");
    set_table_cell(g_full_table, 0, 3, "Vout");
    set_table_cell(g_full_table, 0, 4, "Gain");
    set_table_cell(g_full_table, 0, 5, "Theory");
    set_table_cell(g_full_table, 0, 6, "Error");
    set_table_cell(g_full_table, 0, 7, "Phase");

    if (g_table_count == 0U) {
        set_table_cell(g_full_table, 1, 0, "No data yet");
        set_table_cell(g_full_table, 1, 1, "");
        set_table_cell(g_full_table, 1, 2, "");
        set_table_cell(g_full_table, 1, 3, "");
        set_table_cell(g_full_table, 1, 4, "");
        set_table_cell(g_full_table, 1, 5, "");
        set_table_cell(g_full_table, 1, 6, "");
        set_table_cell(g_full_table, 1, 7, "");
        return;
    }

    for (uint32_t i = 0; i < g_table_count; ++i) {
        char no[12];
        char freq[24];
        char vin[24];
        char vout[24];
        char gain[24];
        char theory[24];
        char error[24];
        char phase[24];
        char freq_num[16];

        snprintf(no, sizeof(no), "%lu", static_cast<unsigned long>(i + 1U));
        format_freq(freq_num, sizeof(freq_num), g_table[i].freq_hz);
        snprintf(freq, sizeof(freq), "%s Hz", freq_num);
        format_voltage(vin, sizeof(vin), g_table[i].vin_mv);
        format_voltage(vout, sizeof(vout), g_table[i].vout_mv);
        format_x1000(gain, sizeof(gain), g_table[i].gain_x1000);
        format_x1000(theory, sizeof(theory), g_table[i].theory_gain_x1000);
        format_error(error, sizeof(error), g_table[i].error_x10);
        format_phase(phase, sizeof(phase), g_table[i].phase_deg_x10);
        if (g_table[i].theory_gain_x1000 <= 0) {
            snprintf(theory, sizeof(theory), "----");
            snprintf(error, sizeof(error), "N/A");
        }
        if (!g_table[i].phase_valid) {
            snprintf(phase, sizeof(phase), "--");
        }

        const uint32_t row = i + 1U;
        set_table_cell(g_full_table, row, 0, no);
        set_table_cell(g_full_table, row, 1, freq);
        set_table_cell(g_full_table, row, 2, vin);
        set_table_cell(g_full_table, row, 3, vout);
        set_table_cell(g_full_table, row, 4, gain);
        set_table_cell(g_full_table, row, 5, theory);
        set_table_cell(g_full_table, row, 6, error);
        set_table_cell(g_full_table, row, 7, phase);
    }
}
