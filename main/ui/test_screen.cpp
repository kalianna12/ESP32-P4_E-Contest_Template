#include "test_screen.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "dds_direct_link.h"
#include "esp_recon.h"
#include "heavyfit.h"
#include "lvgl.h"
#include "spilink.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

#ifndef ENABLE_CAP_VERBOSE_LOG
#define ENABLE_CAP_VERBOSE_LOG 0
#endif

#ifndef ENABLE_ADV_FPGA_RECON_BUTTONS
#define ENABLE_ADV_FPGA_RECON_BUTTONS 0
#endif

#ifndef ENABLE_ADV_SPI_ECHO_BUTTON
#define ENABLE_ADV_SPI_ECHO_BUTTON 0
#endif

#ifndef ENABLE_ADV_CAP_AUTO_SQUARE
#define ENABLE_ADV_CAP_AUTO_SQUARE 0
#endif

#ifndef ENABLE_BASIC_DDS_FREQ_LOG
#define ENABLE_BASIC_DDS_FREQ_LOG 0
#endif

#ifndef FULL_TABLE_PAGE_ROWS
#define FULL_TABLE_PAGE_ROWS 60U
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

static const char *TAG_UI = "TestScreen";

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
static lv_obj_t *g_full_table_page_label = nullptr;
static bool g_full_table_dirty = false;
static uint32_t g_full_table_page = 0;
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
static constexpr uint32_t ADV_HARMONIC_MAIN_ROWS = 12U;
static lv_obj_t *g_adv_harmonic_rows[ADV_HARMONIC_MAIN_ROWS + 1U] = {};
static lv_obj_t *g_adv_harmonic_table = nullptr;
static adv_harmonic_t g_adv_harmonics[ADV_HARMONIC_MAX] = {};
static bool g_adv_harmonic_valid[ADV_HARMONIC_MAX] = {};
static uint32_t g_adv_harmonic_count = 0;
static uint32_t g_last_adv_capture_done_count = 0;
static uint32_t g_last_adv_recon_done_count = 0;
static bool g_adv_capture_pending = false;
static bool g_adv_recon_pending = false;
static bool g_adv_dds_pending = false;
static uint32_t g_latest_adv_capture_count = 0;
static uint32_t g_latest_adv_recon_count = 0;
static uint32_t g_adv_capture_req_base = 0;
static uint32_t g_adv_recon_req_base = 0;

static uint32_t g_start_freq_hz = DEFAULT_START_FREQ_HZ;
static uint32_t g_stop_freq_hz = DEFAULT_STOP_FREQ_HZ;
static uint32_t g_step_freq_hz = DEFAULT_STEP_FREQ_HZ;
static uint32_t g_single_freq_hz = DEFAULT_SINGLE_FREQ_HZ;
static uint8_t g_mode = MODE_SWEEP;

static freq_point_t *g_table = nullptr;
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
static constexpr uint32_t kCapSampleCount = ESP_RECON_SAMPLE_COUNT;
static int16_t *g_cap_samples = nullptr;
static uint8_t *g_cap_valid = nullptr;
static uint32_t g_cap_received_count = 0;
static uint32_t g_cap_expected_chunks = 0;
static bool g_cap_complete = false;
static bool g_cap_send_square_after_complete = false;
static TaskHandle_t g_dds_direct_task = nullptr;
static volatile bool g_dds_direct_busy = false;
static volatile uint32_t g_dds_direct_result = 0;
static TaskHandle_t g_basic_dds_task = nullptr;
static volatile bool g_basic_dds_task_running = false;
static volatile uint32_t g_basic_dds_pending_freq = 0;
static uint32_t g_basic_dds_last_requested_freq = 0;
static bool g_model_saved_for_current_sweep = false;
static bool g_fit_done_for_current_sweep = false;
static bool g_fit_pending = false;
static bool g_fit_inflight = false;
static uint32_t g_fit_pending_tick = 0;
static uint32_t g_last_render_tick = 0;
static uint32_t g_last_chart_tick = 0;
static bool g_chart_point_pending = false;
static int32_t g_pending_chart_gain = 0;
static freqresp_ui_status_t g_fit_pending_status = {};
static heavyfit_input_t *g_fit_input_work_ptr = nullptr;
static heavyfit_output_t *g_fit_output_work_ptr = nullptr;
static lv_timer_t *g_spi_ui_pump_timer = nullptr;

#define g_fit_input_work (*g_fit_input_work_ptr)
#define g_fit_output_work (*g_fit_output_work_ptr)

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

static void update_adv_model_line(void);
static void render_basic_status(const freqresp_ui_status_t *s);
static void process_pending_fit_request(void);
static void process_heavyfit_result(void);
static void process_dds_direct_result(void);
static bool chart_point_visible(uint32_t index);
static void create_full_table_page(void);
static void render_full_table_page(void);
static void update_full_table_page_label(void);
static void capture_buffer_store_chunk(const adc_waveform_chunk_t *chunk);
static void request_basic_dds_freq(uint32_t freq_hz);
static void basic_dds_follow_status(const freqresp_ui_status_t *s);

static constexpr uint32_t kLabelRenderIntervalMs = 50U;
static constexpr uint32_t kChartRenderIntervalMs = 50U;
static constexpr uint32_t kFitDelayMs = 300U;
static constexpr uint32_t kPhaseValidFlag = 0x00000200U;

#ifndef ENABLE_BASIC_ESP_DDS_CONTROL
#define ENABLE_BASIC_ESP_DDS_CONTROL 1
#endif

#ifndef BASIC_DDS_FOLLOW_IDLE_MS
#define BASIC_DDS_FOLLOW_IDLE_MS 100U
#endif

static bool ensure_ui_work_buffers(void)
{
    if (g_table == nullptr) {
        g_table = static_cast<freq_point_t *>(
            heap_caps_calloc(MAX_POINTS, sizeof(freq_point_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_fit_input_work_ptr == nullptr) {
        g_fit_input_work_ptr = static_cast<heavyfit_input_t *>(
            heap_caps_calloc(1, sizeof(heavyfit_input_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_fit_output_work_ptr == nullptr) {
        g_fit_output_work_ptr = static_cast<heavyfit_output_t *>(
            heap_caps_calloc(1, sizeof(heavyfit_output_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    }
    if (g_cap_samples == nullptr) {
        g_cap_samples = static_cast<int16_t *>(
            heap_caps_calloc(kCapSampleCount, sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
        if (g_cap_samples == nullptr) {
            g_cap_samples = static_cast<int16_t *>(
                heap_caps_calloc(kCapSampleCount, sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
            );
        }
    }
    if (g_cap_valid == nullptr) {
        g_cap_valid = static_cast<uint8_t *>(
            heap_caps_calloc(kCapSampleCount, sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
        if (g_cap_valid == nullptr) {
            g_cap_valid = static_cast<uint8_t *>(
                heap_caps_calloc(kCapSampleCount, sizeof(uint8_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
            );
        }
    }

    if (g_table == nullptr ||
        g_fit_input_work_ptr == nullptr ||
        g_fit_output_work_ptr == nullptr ||
        g_cap_samples == nullptr ||
        g_cap_valid == nullptr) {
        ESP_LOGE(TAG_UI,
                 "UI buffer alloc failed: table=%p fit_in=%p fit_out=%p cap=%p valid=%p psram=%u internal=%u dma=%u",
                 g_table,
                 g_fit_input_work_ptr,
                 g_fit_output_work_ptr,
                 g_cap_samples,
                 g_cap_valid,
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 heap_caps_get_free_size(MALLOC_CAP_DMA));
        return false;
    }

    return true;
}

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
    default: return "LP1";
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
    default: return "LP1";
    }
}

static uint8_t model_filter_type_ui(uint8_t type)
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

static uint32_t fit_display_freq_hz(const filter_fit_result_t *fit)
{
    if (fit == nullptr || !fit->valid) {
        return 0U;
    }
    return (fit->f0_hz != 0U) ? fit->f0_hz : fit->fc_hz;
}

static void apply_last_model_to_status(freqresp_ui_status_t *view)
{
    if (view == nullptr) {
        return;
    }

    if (g_last_fit.valid) {
        view->filter_type = model_filter_type_ui(g_last_fit.model_type);
        view->cutoff_freq_hz = fit_display_freq_hz(&g_last_fit);
    } else if (g_circuit_model.valid) {
        view->filter_type = g_circuit_model.filter_type;
        view->cutoff_freq_hz = g_circuit_model.cutoff_freq_hz;
    }

    if (g_table == nullptr) {
        return;
    }

    for (uint32_t i = g_table_count; i > 0U; --i) {
        const freq_point_t *p = &g_table[i - 1U];
        if ((p->flags & FREQ_POINT_FLAG_MISSING) == 0U) {
            view->theory_gain_x1000 = p->theory_gain_x1000;
            view->error_x10 = p->error_x10;
            break;
        }
    }
}

static bool render_last_basic_status(void)
{
    if (!g_have_last_status) {
        return false;
    }

    freqresp_ui_status_t view = g_last_status;
    apply_last_model_to_status(&view);
    g_last_status = view;
    render_basic_status(&g_last_status);
    return true;
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
    lv_obj_set_style_border_color(ta, lv_color_hex(COLOR_BLUE), LV_STATE_FOCUSED);
    lv_obj_set_style_bg_color(ta, lv_color_hex(COLOR_TEXT), LV_PART_CURSOR);
    lv_obj_set_style_bg_opa(ta, LV_OPA_COVER, LV_PART_CURSOR);
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
    if (g_cap_samples != nullptr) {
        memset(g_cap_samples, 0, kCapSampleCount * sizeof(g_cap_samples[0]));
    }
    if (g_cap_valid != nullptr) {
        memset(g_cap_valid, 0, kCapSampleCount * sizeof(g_cap_valid[0]));
    }
    g_cap_received_count = 0;
    g_cap_expected_chunks = 0;
    g_cap_complete = false;
    g_cap_send_square_after_complete = false;
    g_adv_harmonic_count = 0;
    memset(g_adv_harmonic_valid, 0, sizeof(g_adv_harmonic_valid));
    g_last_adv_capture_done_count = 0;
    g_last_adv_recon_done_count = 0;
    g_adv_capture_pending = false;
    g_adv_recon_pending = false;
    g_adv_capture_req_base = g_latest_adv_capture_count;
    g_adv_recon_req_base = g_latest_adv_recon_count;
    g_model_saved_for_current_sweep = false;
    g_fit_done_for_current_sweep = false;
    g_fit_pending = false;
    g_fit_inflight = false;
    g_fit_pending_tick = 0;
    g_last_render_tick = 0;
    g_last_chart_tick = 0;
    g_chart_point_pending = false;
    g_pending_chart_gain = 0;
    g_fit_pending_status = {};
    HeavyFit_Cancel();
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
        lv_label_set_text(g_type, "Type: LP1");
        lv_label_set_text(g_fc, "fc: ---- Hz");
    }
    if (g_full_table != nullptr) {
        lv_table_set_row_cnt(g_full_table, 2);
        lv_table_set_cell_value(g_full_table, 1, 0, "No data yet");
        for (uint32_t col = 1; col < 9; ++col) {
            lv_table_set_cell_value(g_full_table, 1, col, "");
        }
        update_full_table_page_label();
    }
    g_full_table_dirty = false;
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

static bool append_missing_table_point(uint32_t point_index, uint32_t total_points)
{
    if (!ensure_ui_work_buffers()) {
        return false;
    }

    if (g_table_count >= MAX_POINTS) {
        if (!g_table_full) {
            g_table_full = true;
            set_msg("FULL", COLOR_RED);
        }
        return false;
    }

    freq_point_t *point = &g_table[g_table_count++];
    *point = {};
    point->point_index = point_index;
    point->total_points = total_points;
    point->flags = FREQ_POINT_FLAG_MISSING;
    point->phase_valid = false;
    return true;
}

static double model_gain_no_k_ui(uint8_t model_type, double f_hz, double f0_hz, double q)
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

static void append_table_point(const freqresp_ui_status_t *s)
{
    if (!ensure_ui_work_buffers()) {
        return;
    }

    if (s->current_freq_hz == 0U) {
        return;
    }

    if (s->point_index == 0U) {
        return;
    }

    if (g_last_point_index != UINT32_MAX && s->point_index <= g_last_point_index) {
        return;
    }

    if (g_last_point_index != UINT32_MAX && s->point_index > (g_last_point_index + 1U)) {
        spilink_stats_t stats = {};
        SpiLink_GetStats(&stats);
        ESP_LOGW(TAG_UI,
                 "MISSING gap: missing_start=%lu missing_end=%lu last_point_index=%lu new_point_index=%lu freq=%lu dropped=%lu bad_first=%lu frame_errors=%lu high_water=%lu",
                 static_cast<unsigned long>(g_last_point_index + 1U),
                 static_cast<unsigned long>(s->point_index - 1U),
                 static_cast<unsigned long>(g_last_point_index),
                 static_cast<unsigned long>(s->point_index),
                 static_cast<unsigned long>(s->current_freq_hz),
                 static_cast<unsigned long>(stats.dropped_points),
                 static_cast<unsigned long>(stats.bad_first_bytes),
                 static_cast<unsigned long>(stats.frame_errors),
                 static_cast<unsigned long>(stats.point_queue_high_water));
        for (uint32_t missing = g_last_point_index + 1U; missing < s->point_index; ++missing) {
            if (!append_missing_table_point(missing, s->total_points)) {
                break;
            }
        }
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
    point->point_index = s->point_index;
    point->total_points = s->total_points;
    point->freq_hz = s->current_freq_hz;
    point->vin_mv = s->vin_mv;
    point->vout_mv = s->vout_mv;
    point->gain_x1000 = s->gain_x1000;
    point->theory_gain_x1000 = s->theory_gain_x1000;
    point->error_x10 = s->error_x10;
    point->phase_deg_x10 = s->phase_deg_x10;
    point->flags = s->flags;
    point->phase_valid = s->phase_valid;
    g_full_table_dirty = true;

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
    if ((p->flags & FREQ_POINT_FLAG_MISSING) != 0U) {
        return FIT_REJECT_FREQ;
    }
    if (p->freq_hz == 0U) {
        return FIT_REJECT_FREQ;
    }
    if ((p->flags & FREQ_POINT_FLAG_UNSTABLE) != 0U) {
        return FIT_REJECT_GAIN;
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

static bool chart_point_visible(uint32_t index)
{
    if (index >= g_table_count) {
        return false;
    }

    const freq_point_t *cur = &g_table[index];
    if ((cur->flags & FREQ_POINT_FLAG_MISSING) != 0U ||
        cur->freq_hz == 0U ||
        cur->gain_x1000 <= 0 ||
        cur->gain_x1000 > 3000) {
        return false;
    }

    if (index == 0U) {
        return true;
    }

    for (int32_t i = static_cast<int32_t>(index) - 1; i >= 0; --i) {
        const freq_point_t *prev = &g_table[i];
        if ((prev->flags & FREQ_POINT_FLAG_MISSING) != 0U ||
            prev->freq_hz == 0U ||
            prev->gain_x1000 <= 0 ||
            prev->gain_x1000 > 3000 ||
            prev->freq_hz >= cur->freq_hz) {
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

static bool recompute_theory_columns_from_fit(const filter_fit_result_t *fit, uint32_t point_count)
{
    if (fit == nullptr || !fit->valid || point_count == 0U) {
        return false;
    }

    double q = static_cast<double>(fit->q_x1000) / 1000.0;
    if (q <= 0.0) {
        q = 0.707;
    }
    double k = static_cast<double>(fit->k_x1000) / 1000.0;
    if (k <= 0.0) {
        k = 1.0;
    }
    const double f0 = (fit->f0_hz != 0U) ?
        static_cast<double>(fit->f0_hz) :
        static_cast<double>(fit->fc_hz);
    if (f0 <= 0.0) {
        return false;
    }

    bool wrote_any = false;
    const uint32_t n = (point_count < g_table_count) ? point_count : g_table_count;
    for (uint32_t i = 0; i < n; ++i) {
        if (fit_point_reject_reason_strict(i) != FIT_REJECT_NONE) {
            g_table[i].theory_gain_x1000 = 0;
            g_table[i].error_x10 = 0;
            continue;
        }

        const double h0 = model_gain_no_k_ui(fit->model_type,
                                             static_cast<double>(g_table[i].freq_hz),
                                             f0,
                                             q);
        const double theory = k * h0;
        if (theory <= 0.000001) {
            g_table[i].theory_gain_x1000 = 0;
            g_table[i].error_x10 = 0;
            continue;
        }

        const double measured = static_cast<double>(g_table[i].gain_x1000) / 1000.0;
        g_table[i].theory_gain_x1000 = static_cast<int32_t>(theory * 1000.0 + 0.5);
        g_table[i].error_x10 =
            static_cast<int32_t>((fabs(measured - theory) / theory) * 1000.0 + 0.5);
        wrote_any = true;
    }

    return wrote_any;
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
    g_circuit_model.point_count = 0;
    g_circuit_model.saved_time_ms = lv_tick_get();

    for (uint32_t i = 0; i < g_table_count && g_circuit_model.point_count < MODEL_POINTS_MAX; ++i) {
        if (fit_point_reject_reason_strict(i) != FIT_REJECT_NONE) {
            continue;
        }
        const uint32_t dst = g_circuit_model.point_count++;
        g_circuit_model.points[dst].freq_hz = g_table[i].freq_hz;
        g_circuit_model.points[dst].gain_x1000 = g_table[i].gain_x1000;
        g_circuit_model.points[dst].phase_deg_x10 = g_table[i].phase_deg_x10;
    }

    g_adv_output_captured = false;
    g_adv_reconstruction_ready = false;
    g_model_saved_for_current_sweep = true;
}

static void process_pending_fit_request(void)
{
    if (!ensure_ui_work_buffers()) {
        return;
    }

    if (!g_fit_pending || g_fit_done_for_current_sweep || g_fit_inflight || HeavyFit_IsBusy()) {
        return;
    }

    if (SpiLink_PointQueueWaiting() != 0U) {
        return;
    }

    const uint32_t now = lv_tick_get();
    if ((now - g_fit_pending_tick) < kFitDelayMs) {
        return;
    }

    memset(&g_fit_input_work, 0, sizeof(g_fit_input_work));
    g_fit_input_work.status = g_fit_pending_status;
    g_fit_input_work.point_count = (g_table_count > MAX_POINTS) ? MAX_POINTS : g_table_count;
    for (uint32_t i = 0; i < g_fit_input_work.point_count; ++i) {
        g_fit_input_work.points[i] = g_table[i];
    }

    if (HeavyFit_StartAsync(&g_fit_input_work)) {
        g_fit_pending = false;
        g_fit_inflight = true;
        ESP_LOGI(TAG_UI,
                 "Heavy fit started: points=%lu queue_waiting=%lu",
                 static_cast<unsigned long>(g_fit_input_work.point_count),
                 static_cast<unsigned long>(SpiLink_PointQueueWaiting()));
        set_msg("FITTING...", COLOR_YELLOW);
    }
}

static void process_heavyfit_result(void)
{
    if (!g_fit_inflight) {
        return;
    }

    if (!HeavyFit_PollResult(&g_fit_output_work)) {
        return;
    }

    g_fit_inflight = false;
    if (g_fit_output_work.canceled) {
        ESP_LOGW(TAG_UI, "Heavy fit canceled");
        set_msg("FIT CANCELED", COLOR_YELLOW);
        return;
    }

    if (!g_fit_output_work.valid) {
        g_fit_done_for_current_sweep = true;
        g_last_fit = {};
        g_fit_result_quality = FIT_RESULT_NONE;
        ESP_LOGW(TAG_UI, "Heavy fit failed: output invalid");
        set_msg("FIT FAIL", COLOR_RED);
        return;
    }

    const uint32_t n = (g_fit_output_work.point_count < g_table_count) ?
        g_fit_output_work.point_count : g_table_count;
    bool copied_theory = false;
    for (uint32_t i = 0; i < n; ++i) {
        g_table[i].theory_gain_x1000 = g_fit_output_work.theory_gain_x1000[i];
        g_table[i].error_x10 = g_fit_output_work.error_x10[i];
        if (g_fit_output_work.theory_gain_x1000[i] > 0) {
            copied_theory = true;
        }
    }

    if (!copied_theory && g_fit_output_work.fit.valid) {
        copied_theory = recompute_theory_columns_from_fit(&g_fit_output_work.fit, n);
        ESP_LOGW(TAG_UI,
                 "Heavy fit output had no theory columns; recompute %s",
                 copied_theory ? "ok" : "failed");
    }

    if (n != 0U) {
        g_fit_output_work.status.theory_gain_x1000 = g_table[n - 1U].theory_gain_x1000;
        g_fit_output_work.status.error_x10 = g_table[n - 1U].error_x10;
    }
    g_last_fit = g_fit_output_work.fit;
    g_fit_result_quality = static_cast<fit_result_quality_t>(g_fit_output_work.quality);
    g_fit_done_for_current_sweep = true;
    apply_last_model_to_status(&g_fit_output_work.status);
    g_last_status = g_fit_output_work.status;
    g_have_last_status = true;
    save_circuit_model_from_table(&g_fit_output_work.status);
    update_adv_model_line();
    g_full_table_dirty = true;
    refresh_chart_from_table();
    if (g_full_table != nullptr) {
        create_full_table_page();
    }
    render_basic_status(&g_last_status);
    ESP_LOGI(TAG_UI,
             "Heavy fit done: quality=%u type=%u fc=%lu confidence=%ld",
             static_cast<unsigned>(g_fit_result_quality),
             static_cast<unsigned>(g_last_fit.model_type),
             static_cast<unsigned long>(g_fit_output_work.status.cutoff_freq_hz),
             static_cast<long>(g_last_fit.confidence_x1000));
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
        if (g_circuit_model.fit.valid) {
            if ((g_circuit_model.fit.model_type == MODEL_TYPE_BP2 ||
                 g_circuit_model.fit.model_type == MODEL_TYPE_BS2) &&
                g_circuit_model.fit.fl_hz != 0U &&
                g_circuit_model.fit.fh_hz != 0U) {
                char fl[16];
                char fh[16];
                format_freq(fl, sizeof(fl), g_circuit_model.fit.fl_hz);
                format_freq(fh, sizeof(fh), g_circuit_model.fit.fh_hz);
                snprintf(buf,
                         sizeof(buf),
                         "Circuit Model: %s      f0: %s Hz      fL/fH: %s/%s Hz",
                         model_kind_text(g_circuit_model.fit.model_type),
                         fc,
                         fl,
                         fh);
            } else {
                snprintf(buf,
                         sizeof(buf),
                         "Circuit Model: %s      f0/fc: %s Hz      H(f): Loaded",
                         model_kind_text(g_circuit_model.fit.model_type),
                         fc);
            }
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
                     "Model Range: %s~%s Hz      Points: %lu",
                     start_freq,
                     stop_freq,
                     static_cast<unsigned long>(g_circuit_model.fit.valid_point_count));
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
        snprintf(buf, sizeof(buf), "Circuit Model: LP1      fc: ----- Hz      H(f): Pending");
        lv_obj_set_style_text_color(g_adv_model_line, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN);
        lv_label_set_text(g_adv_model_line, buf);

        lv_label_set_text(g_adv_model_range_line, "Model Range: -----~----- Hz      Points: 0");
        lv_obj_set_style_text_color(g_adv_model_range_line, lv_color_hex(COLOR_SUBTEXT), LV_PART_MAIN);
    }
}

static void set_harmonic_rows_empty(void)
{
    for (uint32_t i = 0; i <= ADV_HARMONIC_MAIN_ROWS; ++i) {
        if (g_adv_harmonic_rows[i] == nullptr) {
            continue;
        }
        if (i == 0U) {
            lv_label_set_text(g_adv_harmonic_rows[i], "No Freq Amp Phase Flags");
        } else {
            lv_label_set_text(g_adv_harmonic_rows[i], "-- ----- -- -- --");
        }
    }
}

static void refresh_harmonic_rows(void)
{
    if (!g_reconstruction_page_active) {
        return;
    }

    if (g_adv_harmonic_count == 0U) {
        set_harmonic_rows_empty();
        return;
    }

    if (g_adv_harmonic_rows[0] != nullptr) {
        lv_label_set_text(g_adv_harmonic_rows[0], "No Freq Amp Phase Flags");
    }

    uint32_t shown = 0;
    for (uint32_t i = 0; i < ADV_HARMONIC_MAX && shown < ADV_HARMONIC_MAIN_ROWS; ++i) {
        if (!g_adv_harmonic_valid[i]) {
            continue;
        }

        char freq[16];
        char phase[20];
        char buf[96];
        format_freq(freq, sizeof(freq), g_adv_harmonics[i].freq_hz);
        format_phase(phase, sizeof(phase), g_adv_harmonics[i].phase_deg_x10);
        snprintf(buf,
                 sizeof(buf),
                 "%lu %sHz %ldmV %s 0x%lX",
                 static_cast<unsigned long>(g_adv_harmonics[i].index),
                 freq,
                 static_cast<long>(g_adv_harmonics[i].amp_mv),
                 phase,
                 static_cast<unsigned long>(g_adv_harmonics[i].flags));

        if (g_adv_harmonic_rows[shown + 1U] != nullptr) {
            lv_label_set_text(g_adv_harmonic_rows[shown + 1U], buf);
        }
        ++shown;
    }

    while (shown < ADV_HARMONIC_MAIN_ROWS) {
        if (g_adv_harmonic_rows[shown + 1U] != nullptr) {
            lv_label_set_text(g_adv_harmonic_rows[shown + 1U], "-- ----- -- -- --");
        }
        ++shown;
    }
}

static void update_top_status(const freqresp_ui_status_t *s)
{
    char buf[192];
    const char *link = "WAIT";
    spilink_stats_t stats = {};
    SpiLink_GetStats(&stats);

    if (s->link_ok != 0U) {
        link = "OK";
    } else if (s->frame_errors != 0U || s->timeouts != 0U) {
        link = "ERR";
    }

    snprintf(buf,
             sizeof(buf),
             "L:%s S:%s P:%lu%% %lu/%lu RX:%lu ERR:%lu BFB:%lu DROP:%lu HDROP:%lu Q:%lu",
             link,
             state_text(s->state),
             static_cast<unsigned long>(s->progress_permille / 10U),
             static_cast<unsigned long>(s->point_index),
             static_cast<unsigned long>(s->total_points),
             static_cast<unsigned long>(stats.rx_ok),
             static_cast<unsigned long>(stats.frame_errors + stats.timeouts),
             static_cast<unsigned long>(stats.bad_first_bytes),
             static_cast<unsigned long>(stats.dropped_points),
             static_cast<unsigned long>(stats.dropped_harmonics),
             static_cast<unsigned long>(stats.point_queue_waiting));
    lv_label_set_text(g_top_status, buf);

    uint32_t color = COLOR_YELLOW;
    if ((stats.dropped_points == 0U && stats.dropped_harmonics == 0U) &&
        (strcmp(link, "OK") == 0 && s->state != FREQRESP_STATE_ERROR)) {
        color = COLOR_GREEN;
    }
    if (stats.dropped_points != 0U ||
        stats.dropped_harmonics != 0U ||
        s->state == FREQRESP_STATE_ERROR ||
        strcmp(link, "ERR") == 0) {
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
    uint8_t display_filter_type = s->filter_type;
    uint32_t display_cutoff_hz = s->cutoff_freq_hz;
    if (g_last_fit.valid) {
        display_filter_type = model_filter_type_ui(g_last_fit.model_type);
        display_cutoff_hz = fit_display_freq_hz(&g_last_fit);
    } else if (g_circuit_model.valid) {
        display_filter_type = g_circuit_model.filter_type;
        display_cutoff_hz = g_circuit_model.cutoff_freq_hz;
    }
    format_cutoff_freq(fc, sizeof(fc), display_filter_type, display_cutoff_hz);
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

    if (g_last_fit.valid) {
        snprintf(buf, sizeof(buf), "Type: %s", model_kind_text(g_last_fit.model_type));
    } else {
        snprintf(buf, sizeof(buf), "Type: LP1");
    }
    lv_label_set_text(g_type, buf);

    if (g_last_fit.valid &&
        (g_last_fit.model_type == MODEL_TYPE_BP2 ||
         g_last_fit.model_type == MODEL_TYPE_BS2) &&
        g_last_fit.fl_hz != 0U &&
        g_last_fit.fh_hz != 0U) {
        char fl[16];
        char fh[16];
        format_freq(fl, sizeof(fl), g_last_fit.fl_hz);
        format_freq(fh, sizeof(fh), g_last_fit.fh_hz);
        snprintf(buf, sizeof(buf), "%s/%s Hz", fl, fh);
    } else if (g_last_fit.valid &&
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

    spilink_stats_t stats = {};
    SpiLink_GetStats(&stats);
    if (stats.dropped_points != 0U || stats.dropped_harmonics != 0U) {
        set_msg("QUEUE DROP", COLOR_RED);
    } else if (g_table_full) {
        set_msg("FULL", COLOR_RED);
    }
}

static void basic_dds_task_entry(void *arg)
{
    (void)arg;

    uint32_t idle_ms = 0;
    while (idle_ms < BASIC_DDS_FOLLOW_IDLE_MS) {
        const uint32_t freq_hz = g_basic_dds_pending_freq;
        if (freq_hz != 0U) {
            g_basic_dds_pending_freq = 0;
            idle_ms = 0;
            const bool ok = DdsDirect_SetBasicFreq(freq_hz);
            if (!ok) {
                g_basic_dds_last_requested_freq = 0;
            }
#if ENABLE_BASIC_DDS_FREQ_LOG
            ESP_LOGW(TAG_UI, "Basic DDS direct freq %s: %lu Hz",
                     ok ? "ok" : "failed",
                     static_cast<unsigned long>(freq_hz));
#endif
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            idle_ms += 5U;
        }
    }

    g_basic_dds_task_running = false;
    g_basic_dds_task = nullptr;
    vTaskDelete(nullptr);
}

static void request_basic_dds_freq(uint32_t freq_hz)
{
#if ENABLE_BASIC_ESP_DDS_CONTROL
    if (freq_hz == 0U || freq_hz == g_basic_dds_last_requested_freq) {
        return;
    }

    g_basic_dds_last_requested_freq = freq_hz;
    g_basic_dds_pending_freq = freq_hz;

    if (!g_basic_dds_task_running) {
        g_basic_dds_task_running = true;
        const BaseType_t created = xTaskCreatePinnedToCore(
            basic_dds_task_entry,
            "basic_dds_freq",
            4096,
            nullptr,
            2,
            &g_basic_dds_task,
            1);
        if (created != pdPASS) {
            g_basic_dds_task_running = false;
            g_basic_dds_task = nullptr;
            g_basic_dds_last_requested_freq = 0;
            ESP_LOGE(TAG_UI, "Create basic DDS task failed");
        }
    }
#else
    (void)freq_hz;
#endif
}

static void basic_dds_follow_status(const freqresp_ui_status_t *s)
{
#if ENABLE_BASIC_ESP_DDS_CONTROL
    if (s == nullptr || s->state != FREQRESP_STATE_SCANNING || s->current_freq_hz == 0U) {
        return;
    }
    request_basic_dds_freq(s->current_freq_hz);
#else
    (void)s;
#endif
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
#if ENABLE_BASIC_ESP_DDS_CONTROL
    g_basic_dds_last_requested_freq = 0;
    request_basic_dds_freq((g_mode == MODE_SWEEP) ? g_start_freq_hz : g_single_freq_hz);
    SpiLink_SetPendingCommand(CMD_SET_ESP_DDS_MODE, 1U, 0U);
#else
    SpiLink_SetPendingCommand(CMD_SET_ESP_DDS_MODE, 0U, 0U);
#endif
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
        set_msg("STOP MEAS", COLOR_YELLOW);
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
static void create_harmonic_table_page(void);
static void set_table_cell(lv_obj_t *table, uint32_t row, uint32_t col, const char *text);

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

static void open_harmonic_table_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_harmonic_table_page();
    }
}

static void back_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_main_page();
        render_last_basic_status();
    }
}

static void adv_capture_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    g_adv_capture_pending = true;
    g_adv_recon_pending = false;
    g_cap_send_square_after_complete = false;
    g_adv_capture_req_base = g_latest_adv_capture_count;
    g_adv_output_captured = false;
    g_adv_reconstruction_ready = false;
    g_adv_harmonic_count = 0;
    memset(g_adv_harmonic_valid, 0, sizeof(g_adv_harmonic_valid));
    refresh_harmonic_rows();
    SpiLink_SetPendingCommand(CMD_ADV_CAPTURE, 0U, 0U);
    set_adv_result("CAP REQ / WAIT", COLOR_YELLOW);
}

static void adv_capture_direct_square_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    g_adv_capture_pending = true;
    g_adv_recon_pending = false;
    g_cap_send_square_after_complete = true;
    g_adv_capture_req_base = g_latest_adv_capture_count;
    g_adv_output_captured = false;
    g_adv_reconstruction_ready = false;
    g_adv_harmonic_count = 0;
    memset(g_adv_harmonic_valid, 0, sizeof(g_adv_harmonic_valid));
    refresh_harmonic_rows();
    SpiLink_SetPendingCommand(CMD_ADV_CAPTURE, 0U, 0U);
    set_adv_result("CAP REQ / DDS DIRECT SQ", COLOR_YELLOW);
}

static void adv_reconstruct_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!g_circuit_model.valid) {
        set_adv_result("MODEL FIRST", COLOR_RED);
        return;
    }

    if (!g_adv_output_captured) {
        set_adv_result("CAP FIRST", COLOR_RED);
        return;
    }

    g_adv_recon_pending = true;
    g_adv_dds_pending = true;
    g_adv_recon_req_base = g_latest_adv_recon_count;
    g_adv_reconstruction_ready = false;
    g_adv_harmonic_count = 0;
    memset(g_adv_harmonic_valid, 0, sizeof(g_adv_harmonic_valid));
    refresh_harmonic_rows();
    SpiLink_SetPendingCommand(CMD_ADV_RECONSTRUCT, 0U, 0U);
    set_adv_result("RECON+DDS / WAIT", COLOR_YELLOW);
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
    g_adv_dds_pending = true;
    set_adv_result("DDS REQ / WAIT", COLOR_YELLOW);
}

typedef enum {
    DDS_DIRECT_JOB_SQUARE = 1,
    DDS_DIRECT_JOB_TRIANGLE = 2,
    DDS_DIRECT_JOB_ECHO = 3,
    DDS_DIRECT_JOB_RECON = 4,
} dds_direct_job_t;

static void dds_direct_task_entry(void *arg)
{
    const uint32_t job = reinterpret_cast<uintptr_t>(arg);
    bool ok = false;

    if (job == DDS_DIRECT_JOB_SQUARE) {
        ok = DdsDirect_SendSquareTest();
    } else if (job == DDS_DIRECT_JOB_TRIANGLE) {
        ok = DdsDirect_SendTriangleTest();
    } else if (job == DDS_DIRECT_JOB_ECHO) {
        ok = DdsDirect_SpiEchoTest();
    } else if (job == DDS_DIRECT_JOB_RECON) {
        ok = EspRecon_SendFromCapture(g_cap_samples,
                                      kCapSampleCount,
                                      100000U,
                                      g_circuit_model.valid ? &g_circuit_model : nullptr);
    }

    g_dds_direct_result = ok ? job : (job | 0x80000000U);
    g_dds_direct_busy = false;
    g_dds_direct_task = nullptr;
    vTaskDelete(nullptr);
}

static bool start_dds_direct_job(dds_direct_job_t job, const char *busy_text)
{
    if (g_dds_direct_busy) {
        set_adv_result("DDS DIRECT BUSY", COLOR_YELLOW);
        return false;
    }

    g_dds_direct_busy = true;
    g_dds_direct_result = 0;
    set_adv_result(busy_text, COLOR_YELLOW);

    const BaseType_t created = xTaskCreatePinnedToCore(
        dds_direct_task_entry,
        "dds_direct_tx",
        8192,
        reinterpret_cast<void *>(static_cast<uintptr_t>(job)),
        2,
        &g_dds_direct_task,
        1);
    if (created != pdPASS) {
        g_dds_direct_busy = false;
        g_dds_direct_task = nullptr;
        set_adv_result("DDS DIRECT TASK FAIL", COLOR_RED);
        return false;
    }
    return true;
}

static void process_dds_direct_result(void)
{
    const uint32_t result = g_dds_direct_result;
    if (result == 0U) {
        return;
    }
    g_dds_direct_result = 0;

    const bool ok = (result & 0x80000000U) == 0U;
    const uint32_t job = result & 0x7FFFFFFFU;
    if (job == DDS_DIRECT_JOB_SQUARE) {
        set_adv_result(ok ? "DDS DIRECT SQ DONE" : "DDS DIRECT SQ FAIL",
                       ok ? COLOR_GREEN : COLOR_RED);
    } else if (job == DDS_DIRECT_JOB_TRIANGLE) {
        set_adv_result(ok ? "DDS DIRECT TRI DONE" : "DDS DIRECT TRI FAIL",
                       ok ? COLOR_GREEN : COLOR_RED);
    } else if (job == DDS_DIRECT_JOB_ECHO) {
        set_adv_result(ok ? "DDS SPI ECHO DONE" : "DDS SPI ECHO FAIL",
                       ok ? COLOR_GREEN : COLOR_RED);
    } else if (job == DDS_DIRECT_JOB_RECON) {
        if (ok) {
            esp_recon_harmonic_t harmonics[ESP_RECON_HARMONIC_MAX] = {};
            const uint32_t count = EspRecon_GetLastHarmonics(harmonics, ESP_RECON_HARMONIC_MAX);
            memset(g_adv_harmonic_valid, 0, sizeof(g_adv_harmonic_valid));
            memset(g_adv_harmonics, 0, sizeof(g_adv_harmonics));
            g_adv_harmonic_count = 0U;
            for (uint32_t i = 0; i < count; ++i) {
                if (harmonics[i].index == 0U || harmonics[i].index > ADV_HARMONIC_MAX) {
                    continue;
                }
                const uint32_t slot = harmonics[i].index - 1U;
                g_adv_harmonics[slot].seq = 0U;
                g_adv_harmonics[slot].index = harmonics[i].index;
                g_adv_harmonics[slot].freq_hz = harmonics[i].freq_hz;
                g_adv_harmonics[slot].amp_mv = harmonics[i].amp_mv;
                g_adv_harmonics[slot].phase_deg_x10 = harmonics[i].phase_deg_x10;
                g_adv_harmonics[slot].flags = harmonics[i].flags;
                g_adv_harmonic_valid[slot] = true;
                if (g_adv_harmonic_count < harmonics[i].index) {
                    g_adv_harmonic_count = harmonics[i].index;
                }
            }
            refresh_harmonic_rows();
        }
        set_adv_result(ok ? "ESP RECON DDS DONE" : "ESP RECON DDS FAIL",
                       ok ? COLOR_GREEN : COLOR_RED);
    }
}

static void adv_direct_square_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    start_dds_direct_job(DDS_DIRECT_JOB_SQUARE, "DDS DIRECT SQ TX");
}

static void adv_direct_triangle_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    start_dds_direct_job(DDS_DIRECT_JOB_TRIANGLE, "DDS DIRECT TRI TX");
}

static void adv_esp_recon_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!g_cap_complete || g_cap_samples == nullptr) {
        set_adv_result("CAP FIRST", COLOR_RED);
        return;
    }

    if (!g_circuit_model.valid && ESP_RECON_STAGE != 0) {
        set_adv_result("MODEL FIRST", COLOR_RED);
        return;
    }

    start_dds_direct_job(DDS_DIRECT_JOB_RECON, "ESP RECON DDS TX");
}

static void adv_spi_echo_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    start_dds_direct_job(DDS_DIRECT_JOB_ECHO, "DDS SPI ECHO TX");
}

static void adv_back_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_main_page();
        render_last_basic_status();
    }
}

static void harmonic_back_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        create_reconstruction_page();
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
        render_last_basic_status();
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
    g_full_table_page_label = nullptr;
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
    for (uint32_t i = 0; i <= ADV_HARMONIC_MAIN_ROWS; ++i) {
        g_adv_harmonic_rows[i] = nullptr;
    }
    g_adv_harmonic_table = nullptr;

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
    g_full_table_page_label = nullptr;
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
    g_keyboard = nullptr;
    g_adv_output_chart = nullptr;
    g_adv_output_series = nullptr;
    g_adv_recon_chart = nullptr;
    g_adv_recon_series = nullptr;
    g_adv_harmonic_table = nullptr;
    memset(g_adv_harmonic_rows, 0, sizeof(g_adv_harmonic_rows));
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
    g_adv_status = create_label(screen, "ADV WAIT", 610, 14, 400, &lv_font_montserrat_14, COLOR_GREEN);
    create_hline(screen, 55);

    g_adv_model_line = create_label(screen,
                                    "Circuit Model: LP1      fc: ----- Hz      H(f): Pending",
                                    24,
                                    70,
                                    590,
                                    &lv_font_montserrat_16,
                                    COLOR_YELLOW);

    g_adv_model_range_line = create_label(screen,
                                          "Model Range: -----~----- Hz      Points: 0",
                                          24,
                                          96,
                                          590,
                                          &lv_font_montserrat_16,
                                          COLOR_SUBTEXT);

    create_label(screen,
                 "SRC ADC CH2   FS 100 kSPS",
                 24,
                 122,
                 590,
                 &lv_font_montserrat_16,
                 COLOR_SUBTEXT);
    update_adv_model_line();

    create_label(screen,
                 "CAP / ESP RECON / DDS",
                 24,
                 150,
                 590,
                 &lv_font_montserrat_16,
                 COLOR_SUBTEXT);

    lv_obj_t *btn_capture = create_button(screen, "CAP", 24, 178, 120);
    lv_obj_t *btn_esp_recon = create_button(screen, "ESP RECON", 164, 178, 150);
    lv_obj_t *btn_direct_square = create_button(screen, "DIR SQ", 334, 178, 95);
    lv_obj_t *btn_direct_triangle = create_button(screen, "DIR TRI", 449, 178, 95);
    lv_obj_add_event_cb(btn_capture, adv_capture_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_esp_recon, adv_esp_recon_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_direct_square, adv_direct_square_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_direct_triangle, adv_direct_triangle_event_cb, LV_EVENT_CLICKED, nullptr);

#if ENABLE_ADV_FPGA_RECON_BUTTONS
    lv_obj_t *btn_reconstruct = create_button(screen, "FPGA RECON", 564, 178, 150);
    lv_obj_t *btn_send = create_button(screen, "DDS RETRY", 734, 178, 110);
    lv_obj_add_event_cb(btn_reconstruct, adv_reconstruct_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_send, adv_send_event_cb, LV_EVENT_CLICKED, nullptr);
#endif

#if ENABLE_ADV_CAP_AUTO_SQUARE
    lv_obj_t *btn_cap_direct_square = create_button(screen, "CAP+SQ", 864, 178, 115);
    lv_obj_add_event_cb(btn_cap_direct_square, adv_capture_direct_square_event_cb, LV_EVENT_CLICKED, nullptr);
#endif

#if ENABLE_ADV_SPI_ECHO_BUTTON
    lv_obj_t *btn_spi_echo = create_button(screen, "SPI ECHO", 564, 214, 120);
    lv_obj_add_event_cb(btn_spi_echo, adv_spi_echo_event_cb, LV_EVENT_CLICKED, nullptr);
#endif

    lv_obj_t *btn_harmonic = create_button(screen, "Harmonics", 564, 178, 160);
    lv_obj_add_event_cb(btn_harmonic, open_harmonic_table_event_cb, LV_EVENT_CLICKED, nullptr);

    create_hline(screen, 265);

    create_label(screen, "Harmonics", 24, 280, 220, &lv_font_montserrat_20, COLOR_BLUE);
    for (uint32_t i = 0; i <= ADV_HARMONIC_MAIN_ROWS; ++i) {
        g_adv_harmonic_rows[i] = create_label(screen,
                                              "",
                                              24,
                                              312 + static_cast<int32_t>(i * 18U),
                                              560,
                                              &lv_font_montserrat_14,
                                              i == 0U ? COLOR_SUBTEXT : COLOR_TEXT);
    }
    refresh_harmonic_rows();

    create_hline(screen, 535);

    g_adv_result = create_label(screen, "WAIT", 24, 552, 850, &lv_font_montserrat_16, COLOR_YELLOW);
    lv_obj_t *btn_back = create_button(screen, "Back", 900, 552, 100);
    lv_obj_add_event_cb(btn_back, adv_back_event_cb, LV_EVENT_CLICKED, nullptr);

    if (g_adv_reconstruction_ready) {
        set_adv_result("RECON DONE", COLOR_GREEN);
    } else if (g_adv_output_captured) {
        set_adv_result("CAP DONE", COLOR_GREEN);
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
    g_full_table_page_label = nullptr;
    g_adv_status = nullptr;
    g_adv_model_line = nullptr;
    g_adv_model_range_line = nullptr;
    g_adv_result = nullptr;
    g_adv_output_chart = nullptr;
    g_adv_output_series = nullptr;
    g_adv_recon_chart = nullptr;
    g_adv_recon_series = nullptr;
    for (uint32_t i = 0; i <= ADV_HARMONIC_MAIN_ROWS; ++i) {
        g_adv_harmonic_rows[i] = nullptr;
    }
    g_adv_harmonic_table = nullptr;
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
    g_top_status = create_label(screen, "L:WAIT S:Idle RX:0 ERR:0 DROP:0 Q:0", 455, 20, 545, &lv_font_montserrat_14, COLOR_YELLOW);
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
    g_type = create_label(screen, "Type: LP1", 24, 435, 180, &lv_font_montserrat_18, COLOR_TEXT);
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
    if (!ensure_ui_work_buffers()) {
        return;
    }

#if ENABLE_SPI_TEST_WINDOW
    create_spi_test_page();
#else
    create_main_page();
#endif

    if (g_spi_ui_pump_timer == nullptr) {
        g_spi_ui_pump_timer = lv_timer_create([](lv_timer_t *timer) {
            (void)timer;
            SpiLink_UiPump();
            process_pending_fit_request();
            process_heavyfit_result();
            process_dds_direct_result();
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
    basic_dds_follow_status(&view);

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
        g_last_status.flags = view.flags;
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
        flush_pending_chart_point(true);
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

static void capture_buffer_print_summary(uint32_t total_count)
{
    if (g_cap_samples == nullptr || g_cap_valid == nullptr) {
        return;
    }

    int16_t cap_min = 32767;
    int16_t cap_max = -32768;
    int64_t sum = 0;
    uint32_t zero_count = 0;
    const uint32_t n = (total_count > kCapSampleCount) ? kCapSampleCount : total_count;

    for (uint32_t i = 0; i < n; ++i) {
        if (!g_cap_valid[i]) {
            continue;
        }
        const int16_t s = g_cap_samples[i];
        if (s < cap_min) {
            cap_min = s;
        }
        if (s > cap_max) {
            cap_max = s;
        }
        if (s == 0) {
            ++zero_count;
        }
        sum += s;
    }

    const int32_t vpp = static_cast<int32_t>(cap_max) - static_cast<int32_t>(cap_min);
    const int32_t mean = (g_cap_received_count == 0U) ? 0 :
        static_cast<int32_t>(sum / static_cast<int64_t>(g_cap_received_count));

    ESP_LOGW(TAG_UI,
             "CAP COMPLETE min=%d max=%d vpp=%ld mean=%ld zero_count=%lu received=%lu/%lu",
             static_cast<int>(cap_min),
             static_cast<int>(cap_max),
             static_cast<long>(vpp),
             static_cast<long>(mean),
             static_cast<unsigned long>(zero_count),
             static_cast<unsigned long>(g_cap_received_count),
             static_cast<unsigned long>(n));

    char first_line[160];
    int used = snprintf(first_line, sizeof(first_line), "CAP first16:");
    for (uint32_t i = 0; i < 16U && i < n; ++i) {
        used += snprintf(first_line + used, sizeof(first_line) - used, " %d",
                         static_cast<int>(g_cap_samples[i]));
        if (used >= static_cast<int>(sizeof(first_line))) {
            break;
        }
    }
    ESP_LOGW(TAG_UI, "%s", first_line);

    for (uint32_t i = 0; i < n; i += 64U) {
        ESP_LOGW(TAG_UI, "CAP[%04lu]=%d",
                 static_cast<unsigned long>(i),
                 static_cast<int>(g_cap_samples[i]));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void capture_buffer_store_chunk(const adc_waveform_chunk_t *chunk)
{
    if (chunk == nullptr || chunk->wave_type != 0U) {
        return;
    }
    if (!ensure_ui_work_buffers()) {
        return;
    }

    if (chunk->chunk_index == 0U) {
        memset(g_cap_samples, 0, kCapSampleCount * sizeof(g_cap_samples[0]));
        memset(g_cap_valid, 0, kCapSampleCount * sizeof(g_cap_valid[0]));
        g_cap_received_count = 0;
        g_cap_expected_chunks = chunk->chunk_count;
        g_cap_complete = false;
    }

    const uint32_t total = (chunk->total_sample_count > kCapSampleCount) ? kCapSampleCount : chunk->total_sample_count;
#if ENABLE_CAP_VERBOSE_LOG
    ESP_LOGI(TAG_UI,
             "CAP chunk idx=%lu/%lu start=%lu count<=30 flags=0x%08lX total=%lu",
             static_cast<unsigned long>(chunk->chunk_index),
             static_cast<unsigned long>(chunk->chunk_count),
             static_cast<unsigned long>(chunk->start_sample_index),
             static_cast<unsigned long>(chunk->flags),
             static_cast<unsigned long>(chunk->total_sample_count));
#endif

    for (uint32_t i = 0; i < 30U; ++i) {
        const uint32_t sample_index = chunk->start_sample_index + i;
        if (sample_index >= total) {
            break;
        }
        g_cap_samples[sample_index] = chunk->samples[i];
        if (!g_cap_valid[sample_index]) {
            g_cap_valid[sample_index] = true;
            ++g_cap_received_count;
        }
    }

    const bool is_done = (chunk->flags & 0x00000002U) != 0U ||
                         (chunk->chunk_index + 1U >= chunk->chunk_count);
    if (!g_cap_complete && total != 0U && (g_cap_received_count >= total || is_done)) {
        g_cap_complete = (g_cap_received_count >= total);
        if (g_cap_complete) {
            uint32_t missing_chunks = 0;
            const uint32_t expected_chunks =
                (g_cap_expected_chunks == 0U) ? chunk->chunk_count : g_cap_expected_chunks;
            for (uint32_t i = 0; i < expected_chunks; ++i) {
                const uint32_t chunk_start = i * 30U;
                bool chunk_seen = false;
                for (uint32_t j = 0; j < 30U && (chunk_start + j) < total; ++j) {
                    if (g_cap_valid[chunk_start + j]) {
                        chunk_seen = true;
                        break;
                    }
                }
                if (!chunk_seen) {
                    ++missing_chunks;
                }
            }
            ESP_LOGW(TAG_UI,
                     "CAP chunks complete: chunks=%lu missing=%lu samples=%lu/%lu",
                     static_cast<unsigned long>(expected_chunks),
                     static_cast<unsigned long>(missing_chunks),
                     static_cast<unsigned long>(g_cap_received_count),
                     static_cast<unsigned long>(total));
            ESP_LOGW(TAG_UI,
                     "CAP frame meta: min=%ld max=%ld vpp=%ld mean=%ld flags=0x%08lX",
                     static_cast<long>(chunk->min_mv),
                     static_cast<long>(chunk->max_mv),
                     static_cast<long>(chunk->vpp_mv),
                     static_cast<long>(chunk->mean_mv),
                     static_cast<unsigned long>(chunk->flags));
            capture_buffer_print_summary(total);
            if (g_cap_send_square_after_complete) {
                g_cap_send_square_after_complete = false;
                set_adv_result("CAP OK / DDS DIRECT TX", COLOR_YELLOW);
                start_dds_direct_job(DDS_DIRECT_JOB_SQUARE, "CAP OK / DDS DIRECT TX");
            }
        } else {
            g_cap_send_square_after_complete = false;
            ESP_LOGW(TAG_UI,
                     "CAP incomplete at done: received=%lu/%lu idx=%lu/%lu start=%lu flags=0x%08lX s0=%d s1=%d",
                     static_cast<unsigned long>(g_cap_received_count),
                     static_cast<unsigned long>(total),
                     static_cast<unsigned long>(chunk->chunk_index),
                     static_cast<unsigned long>(chunk->chunk_count),
                     static_cast<unsigned long>(chunk->start_sample_index),
                     static_cast<unsigned long>(chunk->flags),
                     static_cast<int>(chunk->samples[0]),
                     static_cast<int>(chunk->samples[1]));
        }
    }
}

void test_screen_update_adc_waveform_chunk(const adc_waveform_chunk_t *chunk)
{
    if (chunk == nullptr) {
        return;
    }

    capture_buffer_store_chunk(chunk);

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
            g_adv_recon_pending = false;
            g_adv_reconstruction_ready = true;
            snprintf(buf,
                     sizeof(buf),
                     "RECON DONE FS=%lu N=%lu",
                     static_cast<unsigned long>(chunk->sample_rate_hz),
                     static_cast<unsigned long>(chunk->total_sample_count));
            set_adv_result(buf, COLOR_GREEN);
            set_harmonic_rows_empty();
        } else {
            g_adv_capture_pending = false;
            g_adv_output_captured = true;
            snprintf(buf,
                     sizeof(buf),
                     "CAP DONE FS=%lu N=%lu",
                     static_cast<unsigned long>(chunk->sample_rate_hz),
                     static_cast<unsigned long>(chunk->total_sample_count));
            set_adv_result(buf, COLOR_GREEN);
        }
    } else if (g_reconstruction_page_active) {
        snprintf(buf,
                 sizeof(buf),
                 "%s RX %lu/%lu",
                 is_recon ? "RECON" : "CAP",
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

    g_latest_adv_capture_count = status->capture_done_count;
    g_latest_adv_recon_count = status->recon_done_count;
    g_last_adv_capture_done_count = status->capture_done_count;
    g_last_adv_recon_done_count = status->recon_done_count;

    if (status->error_code != 0U) {
        g_adv_capture_pending = false;
        g_adv_recon_pending = false;

        const char *msg = "ADV ERR";
        if (status->error_code == 1U) {
            msg = "BUSY";
        } else if (status->error_code == 2U) {
            msg = "CAP?";
        } else if (status->error_code == 3U) {
            msg = "DDS N/I";
        } else if (status->error_code == 4U) {
            msg = "RECON TIMEOUT";
        } else if (status->error_code == 5U) {
            msg = "DDS TIMEOUT";
        }

        g_adv_dds_pending = false;
        ESP_LOGW(TAG_UI,
                 "ADV st=%lu err=%lu stage=%lu sub=%lu cmd=%lu seq=%lu acc=%lu rej=%lu cap=%lu recon=%lu y=%ld x=%ld g=%lu auto=%lu ifft=%lu flags=0x%08lX d4=[%ld,%ld] vpp=%lu z=%lu first=%ld second=%ld mode=0x%08lX",
                 static_cast<unsigned long>(status->adv_state),
                 static_cast<unsigned long>(status->error_code),
                 static_cast<unsigned long>(status->debug_stage),
                 static_cast<unsigned long>(status->debug_substage),
                 static_cast<unsigned long>(status->last_cmd_seen),
                 static_cast<unsigned long>(status->last_cmd_seq),
                 static_cast<unsigned long>(status->last_cmd_accepted),
                 static_cast<unsigned long>(status->last_cmd_reject_reason),
                 static_cast<unsigned long>(status->capture_done_count),
                 static_cast<unsigned long>(status->recon_done_count),
                 static_cast<long>(status->y_vpp),
                 static_cast<long>(status->x_vpp),
                 static_cast<unsigned long>(status->dds_gain_shift),
                 static_cast<unsigned long>(status->dds_auto_scale),
                 static_cast<unsigned long>(status->ifft_unscaled),
                 static_cast<unsigned long>(status->core_dbg_flags),
                 static_cast<long>(status->d4_src_min),
                 static_cast<long>(status->d4_src_max),
                 static_cast<unsigned long>(status->d4_src_vpp),
                 static_cast<unsigned long>(status->d4_src_zero_count),
                 static_cast<long>(status->d4_src_first),
                 static_cast<long>(status->d4_src_second),
                 static_cast<unsigned long>(status->d4_src_mode));
        set_adv_result(msg, status->error_code == 3U ? COLOR_YELLOW : COLOR_RED);
        return;
    }

    if (g_adv_capture_pending &&
        status->capture_done_count > g_adv_capture_req_base) {
        g_adv_capture_pending = false;
        g_adv_output_captured = true;
        if (!g_adv_reconstruction_ready) {
            set_adv_result("CAP DONE", COLOR_GREEN);
        }
    }

    if (g_adv_recon_pending &&
        status->recon_done_count > g_adv_recon_req_base) {
        g_adv_recon_pending = false;
        g_adv_reconstruction_ready = true;
        if (g_adv_dds_pending) {
            set_adv_result("DDS TX / WAIT", COLOR_YELLOW);
        } else {
            set_adv_result("RECON DONE", COLOR_GREEN);
        }
    }

    constexpr uint32_t kAdvStatusFlagDdsPlaying = 0x00010000U;
    if (g_adv_dds_pending && ((status->flags & kAdvStatusFlagDdsPlaying) != 0U)) {
        g_adv_dds_pending = false;
        set_adv_result("DDS PLAYING", COLOR_GREEN);
    } else if (g_adv_dds_pending && status->debug_stage != 0U) {
        char buf[64];
        snprintf(buf,
                 sizeof(buf),
                 "DDS TX %lu/35",
                 static_cast<unsigned long>(status->debug_substage + 1U));
        set_adv_result(buf, COLOR_YELLOW);
    }

    ESP_LOGW(TAG_UI,
             "ADV st=%lu err=%lu stage=%lu sub=%lu cmd=%lu seq=%lu acc=%lu rej=%lu cap=%lu cap_base=%lu cap_pend=%d recon=%lu recon_base=%lu recon_pend=%d y=%ld x=%ld g=%lu auto=%lu ifft=%lu flags=0x%08lX d4=[%ld,%ld] vpp=%lu z=%lu first=%ld second=%ld mode=0x%08lX",
             static_cast<unsigned long>(status->adv_state),
             static_cast<unsigned long>(status->error_code),
             static_cast<unsigned long>(status->debug_stage),
             static_cast<unsigned long>(status->debug_substage),
             static_cast<unsigned long>(status->last_cmd_seen),
             static_cast<unsigned long>(status->last_cmd_seq),
             static_cast<unsigned long>(status->last_cmd_accepted),
             static_cast<unsigned long>(status->last_cmd_reject_reason),
             static_cast<unsigned long>(status->capture_done_count),
             static_cast<unsigned long>(g_adv_capture_req_base),
             g_adv_capture_pending ? 1 : 0,
             static_cast<unsigned long>(status->recon_done_count),
             static_cast<unsigned long>(status->recon_count_base),
             g_adv_recon_pending ? 1 : 0,
             static_cast<long>(status->y_vpp),
             static_cast<long>(status->x_vpp),
             static_cast<unsigned long>(status->dds_gain_shift),
             static_cast<unsigned long>(status->dds_auto_scale),
             static_cast<unsigned long>(status->ifft_unscaled),
             static_cast<unsigned long>(status->core_dbg_flags),
             static_cast<long>(status->d4_src_min),
             static_cast<long>(status->d4_src_max),
             static_cast<unsigned long>(status->d4_src_vpp),
             static_cast<unsigned long>(status->d4_src_zero_count),
             static_cast<long>(status->d4_src_first),
             static_cast<long>(status->d4_src_second),
             static_cast<unsigned long>(status->d4_src_mode));

    if (g_adv_status != nullptr) {
        char buf[320];
        const char recon_mode =
            (status->recon_debug_mode == 0U) ? 'C' :
            (status->recon_debug_mode == 2U) ? 'S' :
            (status->recon_debug_mode == 3U) ? 'T' : 'F';
        snprintf(buf,
                 sizeof(buf),
                 "ADV:%lu E:%lu F:%lu/%lu Y:%ld X:%ld R:%c I:%s\nG:%lu %s M:%lu A:%lu D4:%ld/%ld V:%lu Z:%lu\nS:%lu/%lu F:%ld S:%ld MD:%02lX C:%02lX",
                 static_cast<unsigned long>(status->adv_state),
                 static_cast<unsigned long>(status->error_code),
                 static_cast<unsigned long>(status->fft_overflow_count),
                 static_cast<unsigned long>(status->ifft_overflow_count),
                 static_cast<long>(status->y_vpp),
                 static_cast<long>(status->x_vpp),
                 recon_mode,
                 status->ifft_unscaled ? "U" : "S",
                 static_cast<unsigned long>(status->dds_gain_shift),
                 status->dds_auto_scale ? "AUTO" : "MAN",
                 static_cast<unsigned long>(status->dds_manual_gain_shift),
                 static_cast<unsigned long>(status->dds_auto_gain_shift),
                 static_cast<long>(status->d4_src_min),
                 static_cast<long>(status->d4_src_max),
                 static_cast<unsigned long>(status->d4_src_vpp),
                 static_cast<unsigned long>(status->d4_src_zero_count),
                 static_cast<unsigned long>(status->debug_stage),
                 static_cast<unsigned long>(status->debug_substage),
                 static_cast<long>(status->d4_src_first),
                 static_cast<long>(status->d4_src_second),
                 static_cast<unsigned long>(status->d4_src_mode & 0xFFU),
                 static_cast<unsigned long>(status->core_dbg_flags & 0xFFU));
        lv_label_set_text(g_adv_status, buf);
        lv_obj_set_style_text_color(g_adv_status,
                                    lv_color_hex(COLOR_GREEN),
                                    LV_PART_MAIN);
    }
}

void test_screen_update_adv_harmonic(const adv_harmonic_t *harmonic)
{
    if (harmonic == nullptr || harmonic->index == 0U || harmonic->index > ADV_HARMONIC_MAX) {
        return;
    }

    const uint32_t slot = harmonic->index - 1U;
    g_adv_harmonics[slot] = *harmonic;
    g_adv_harmonic_valid[slot] = true;
    if (g_adv_harmonic_count < harmonic->index) {
        g_adv_harmonic_count = harmonic->index;
    }
    refresh_harmonic_rows();

    if (g_adv_harmonic_table != nullptr) {
        const uint32_t row = harmonic->index;
        char no[12];
        char freq[24];
        char amp[24];
        char phase[24];
        char flags[24];
        char freq_num[16];
        snprintf(no, sizeof(no), "%lu", static_cast<unsigned long>(harmonic->index));
        format_freq(freq_num, sizeof(freq_num), harmonic->freq_hz);
        snprintf(freq, sizeof(freq), "%s Hz", freq_num);
        snprintf(amp, sizeof(amp), "%ld mV", static_cast<long>(harmonic->amp_mv));
        format_phase(phase, sizeof(phase), harmonic->phase_deg_x10);
        snprintf(flags, sizeof(flags), "0x%lX", static_cast<unsigned long>(harmonic->flags));
        set_table_cell(g_adv_harmonic_table, row, 0, no);
        set_table_cell(g_adv_harmonic_table, row, 1, freq);
        set_table_cell(g_adv_harmonic_table, row, 2, amp);
        set_table_cell(g_adv_harmonic_table, row, 3, phase);
        set_table_cell(g_adv_harmonic_table, row, 4, flags);
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

static uint32_t full_table_page_count(void)
{
    if (g_table_count == 0U) {
        return 1U;
    }
    return (g_table_count + FULL_TABLE_PAGE_ROWS - 1U) / FULL_TABLE_PAGE_ROWS;
}

static void update_full_table_page_label(void)
{
    if (g_full_table_page_label == nullptr) {
        return;
    }

    char buf[96];
    const uint32_t pages = full_table_page_count();
    const uint32_t page = (g_full_table_page < pages) ? g_full_table_page : (pages - 1U);
    const uint32_t start = (g_table_count == 0U) ? 0U : (page * FULL_TABLE_PAGE_ROWS + 1U);
    uint32_t end = (page + 1U) * FULL_TABLE_PAGE_ROWS;
    if (end > g_table_count) {
        end = g_table_count;
    }
    snprintf(buf, sizeof(buf), "Page %lu/%lu  Rows %lu-%lu",
             static_cast<unsigned long>(page + 1U),
             static_cast<unsigned long>(pages),
             static_cast<unsigned long>(start),
             static_cast<unsigned long>(end));
    lv_label_set_text(g_full_table_page_label, buf);
}

static void render_full_table_page(void)
{
    if (g_full_table == nullptr) {
        return;
    }

    const uint32_t pages = full_table_page_count();
    if (g_full_table_page >= pages) {
        g_full_table_page = pages - 1U;
    }

    const uint32_t start_index = g_full_table_page * FULL_TABLE_PAGE_ROWS;
    uint32_t rows_on_page = 0;
    if (g_table_count > start_index) {
        rows_on_page = g_table_count - start_index;
        if (rows_on_page > FULL_TABLE_PAGE_ROWS) {
            rows_on_page = FULL_TABLE_PAGE_ROWS;
        }
    }

    lv_table_set_col_cnt(g_full_table, 9);
    lv_table_set_row_cnt(g_full_table, (rows_on_page == 0U) ? 2U : (rows_on_page + 1U));
    lv_table_set_col_width(g_full_table, 0, 70);
    lv_table_set_col_width(g_full_table, 1, 120);
    lv_table_set_col_width(g_full_table, 2, 100);
    lv_table_set_col_width(g_full_table, 3, 100);
    lv_table_set_col_width(g_full_table, 4, 90);
    lv_table_set_col_width(g_full_table, 5, 95);
    lv_table_set_col_width(g_full_table, 6, 90);
    lv_table_set_col_width(g_full_table, 7, 100);
    lv_table_set_col_width(g_full_table, 8, 130);

    set_table_cell(g_full_table, 0, 0, "No.");
    set_table_cell(g_full_table, 0, 1, "Freq");
    set_table_cell(g_full_table, 0, 2, "Vin");
    set_table_cell(g_full_table, 0, 3, "Vout");
    set_table_cell(g_full_table, 0, 4, "Gain");
    set_table_cell(g_full_table, 0, 5, "Theory");
    set_table_cell(g_full_table, 0, 6, "Error");
    set_table_cell(g_full_table, 0, 7, "Phase");
    set_table_cell(g_full_table, 0, 8, "Flags");

    if (rows_on_page == 0U) {
        set_table_cell(g_full_table, 1, 0, "No data yet");
        for (uint32_t col = 1; col < 9; ++col) {
            set_table_cell(g_full_table, 1, col, "");
        }
        update_full_table_page_label();
        g_full_table_dirty = false;
        return;
    }

    for (uint32_t n = 0; n < rows_on_page; ++n) {
        const uint32_t i = start_index + n;
        const uint32_t row = n + 1U;
        char no[12];
        char freq[24];
        char vin[24];
        char vout[24];
        char gain[24];
        char theory[24];
        char error[24];
        char phase[24];
        char flags[32];
        char freq_num[16];

        snprintf(no, sizeof(no), "%lu", static_cast<unsigned long>(g_table[i].point_index));
        format_point_flags(flags, sizeof(flags), g_table[i].flags);
        if ((g_table[i].flags & FREQ_POINT_FLAG_MISSING) != 0U) {
            set_table_cell(g_full_table, row, 0, no);
            set_table_cell(g_full_table, row, 1, "--");
            set_table_cell(g_full_table, row, 2, "--");
            set_table_cell(g_full_table, row, 3, "--");
            set_table_cell(g_full_table, row, 4, "--");
            set_table_cell(g_full_table, row, 5, "--");
            set_table_cell(g_full_table, row, 6, "--");
            set_table_cell(g_full_table, row, 7, "--");
            set_table_cell(g_full_table, row, 8, flags);
            continue;
        }

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

        set_table_cell(g_full_table, row, 0, no);
        set_table_cell(g_full_table, row, 1, freq);
        set_table_cell(g_full_table, row, 2, vin);
        set_table_cell(g_full_table, row, 3, vout);
        set_table_cell(g_full_table, row, 4, gain);
        set_table_cell(g_full_table, row, 5, theory);
        set_table_cell(g_full_table, row, 6, error);
        set_table_cell(g_full_table, row, 7, phase);
        set_table_cell(g_full_table, row, 8, flags);
    }

    update_full_table_page_label();
    g_full_table_dirty = false;
}

static void full_table_prev_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }
    if (g_full_table_page > 0U) {
        --g_full_table_page;
        render_full_table_page();
    }
}

static void full_table_next_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }
    if ((g_full_table_page + 1U) < full_table_page_count()) {
        ++g_full_table_page;
        render_full_table_page();
    }
}

static void create_harmonic_table_page(void)
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
    g_full_table_page_label = nullptr;
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
    g_adv_harmonic_table = nullptr;
    g_adv_status = nullptr;
    g_adv_model_line = nullptr;
    g_adv_model_range_line = nullptr;
    g_adv_result = nullptr;
    g_adv_output_chart = nullptr;
    g_adv_output_series = nullptr;
    g_adv_recon_chart = nullptr;
    g_adv_recon_series = nullptr;
    for (uint32_t i = 0; i <= ADV_HARMONIC_MAIN_ROWS; ++i) {
        g_adv_harmonic_rows[i] = nullptr;
    }

    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    create_label(screen, "Harmonic Table", 24, 14, 320, &lv_font_montserrat_24, COLOR_TEXT);
    lv_obj_t *back = create_button(screen, "Back", 900, 12, 100);
    lv_obj_add_event_cb(back, harmonic_back_event_cb, LV_EVENT_CLICKED, nullptr);
    create_hline(screen, 55);

    g_adv_harmonic_table = lv_table_create(screen);
    lv_obj_set_pos(g_adv_harmonic_table, 20, 70);
    lv_obj_set_size(g_adv_harmonic_table, 984, 510);
    lv_obj_set_scrollbar_mode(g_adv_harmonic_table, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_text_font(g_adv_harmonic_table, &lv_font_montserrat_14, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_adv_harmonic_table, lv_color_hex(COLOR_TEXT), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_adv_harmonic_table, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_adv_harmonic_table, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(g_adv_harmonic_table, lv_color_hex(COLOR_LINE), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_adv_harmonic_table, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_left(g_adv_harmonic_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_adv_harmonic_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_adv_harmonic_table, 5, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_adv_harmonic_table, 5, LV_PART_ITEMS);

    lv_table_set_col_cnt(g_adv_harmonic_table, 5);
    lv_table_set_row_cnt(g_adv_harmonic_table, ADV_HARMONIC_MAX + 1U);
    lv_table_set_col_width(g_adv_harmonic_table, 0, 80);
    lv_table_set_col_width(g_adv_harmonic_table, 1, 180);
    lv_table_set_col_width(g_adv_harmonic_table, 2, 160);
    lv_table_set_col_width(g_adv_harmonic_table, 3, 180);
    lv_table_set_col_width(g_adv_harmonic_table, 4, 180);

    set_table_cell(g_adv_harmonic_table, 0, 0, "No");
    set_table_cell(g_adv_harmonic_table, 0, 1, "Freq");
    set_table_cell(g_adv_harmonic_table, 0, 2, "Amp");
    set_table_cell(g_adv_harmonic_table, 0, 3, "Phase");
    set_table_cell(g_adv_harmonic_table, 0, 4, "Flags");

    for (uint32_t i = 0; i < ADV_HARMONIC_MAX; ++i) {
        const uint32_t row = i + 1U;
        char no[12];
        char freq[24];
        char amp[24];
        char phase[24];
        char flags[24];
        snprintf(no, sizeof(no), "%lu", static_cast<unsigned long>(i + 1U));
        if (!g_adv_harmonic_valid[i]) {
            set_table_cell(g_adv_harmonic_table, row, 0, no);
            set_table_cell(g_adv_harmonic_table, row, 1, "--");
            set_table_cell(g_adv_harmonic_table, row, 2, "--");
            set_table_cell(g_adv_harmonic_table, row, 3, "--");
            set_table_cell(g_adv_harmonic_table, row, 4, "--");
            continue;
        }

        char freq_num[16];
        format_freq(freq_num, sizeof(freq_num), g_adv_harmonics[i].freq_hz);
        snprintf(freq, sizeof(freq), "%s Hz", freq_num);
        snprintf(amp, sizeof(amp), "%ld mV", static_cast<long>(g_adv_harmonics[i].amp_mv));
        format_phase(phase, sizeof(phase), g_adv_harmonics[i].phase_deg_x10);
        snprintf(flags, sizeof(flags), "0x%lX", static_cast<unsigned long>(g_adv_harmonics[i].flags));
        set_table_cell(g_adv_harmonic_table, row, 0, no);
        set_table_cell(g_adv_harmonic_table, row, 1, freq);
        set_table_cell(g_adv_harmonic_table, row, 2, amp);
        set_table_cell(g_adv_harmonic_table, row, 3, phase);
        set_table_cell(g_adv_harmonic_table, row, 4, flags);
    }
}

static void format_point_flags(char *buf, size_t len, uint32_t flags)
{
    if ((flags & FREQ_POINT_FLAG_MISSING) != 0U) {
        snprintf(buf, len, "MISSING");
    } else if ((flags & FREQ_POINT_FLAG_UNSTABLE) != 0U) {
        snprintf(buf, len, "UNSTABLE");
    } else if ((flags & kPhaseValidFlag) == 0U) {
        snprintf(buf, len, "NO_PHASE");
    } else if (flags != 0U) {
        snprintf(buf, len, "0x%08lX", static_cast<unsigned long>(flags));
    } else {
        snprintf(buf, len, "OK");
    }
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
    g_full_table_page_label = nullptr;
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
    g_full_table_page_label = create_label(screen, "Page --", 570, 20, 160, &lv_font_montserrat_14, COLOR_YELLOW);
    lv_obj_t *prev = create_button(screen, "Prev", 735, 12, 70);
    lv_obj_add_event_cb(prev, full_table_prev_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *next = create_button(screen, "Next", 815, 12, 70);
    lv_obj_add_event_cb(next, full_table_next_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *back = create_button(screen, "Back", 900, 12, 100);
    lv_obj_add_event_cb(back, back_button_event_cb, LV_EVENT_CLICKED, nullptr);
    create_hline(screen, 55);

    g_full_table = lv_table_create(screen);
    lv_obj_set_pos(g_full_table, 20, 70);
    lv_obj_set_size(g_full_table, 984, 510);
    lv_obj_set_scrollbar_mode(g_full_table, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_text_font(g_full_table, &lv_font_montserrat_14, LV_PART_ITEMS);
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

    render_full_table_page();
}
