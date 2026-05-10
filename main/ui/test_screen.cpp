#include "test_screen.h"

#include "lvgl.h"
#include "spilink.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef ENABLE_FAKE_DATA_TEST
#define ENABLE_FAKE_DATA_TEST 1
#endif

#define TEST_SCREEN_W 1024
#define TEST_SCREEN_H 600

#define DEFAULT_START_FREQ_HZ 100U
#define DEFAULT_STOP_FREQ_HZ 100000U
#define DEFAULT_STEP_FREQ_HZ 10000U
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

static void clear_table_and_curve(void)
{
    g_table_count = 0;
    g_table_full = false;
    g_last_point_index = UINT32_MAX;
    g_last_point_freq = 0;

    if (g_main_page_active && g_chart != nullptr && g_chart_gain != nullptr) {
        lv_chart_set_all_value(g_chart, g_chart_gain, LV_CHART_POINT_NONE);
        lv_chart_refresh(g_chart);
    }

    if (g_main_page_active && g_latest != nullptr) {
        lv_label_set_text(g_latest, "Latest: --");
    }

    set_msg("CLEARED", COLOR_GREEN);
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
        g_table_full = true;
        set_msg("FULL", COLOR_RED);
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

    if (g_main_page_active && g_chart != nullptr && g_chart_gain != nullptr) {
        int32_t chart_gain = s->gain_x1000;
        if (chart_gain < 0) {
            chart_gain = 0;
        } else if (chart_gain > 1000) {
            chart_gain = 1000;
        }
        lv_chart_set_next_value(g_chart, g_chart_gain, chart_gain);
    }
}

static void refresh_chart_from_table(void)
{
    if (!g_main_page_active || g_chart == nullptr || g_chart_gain == nullptr) {
        return;
    }

    lv_chart_set_all_value(g_chart, g_chart_gain, LV_CHART_POINT_NONE);
    for (uint32_t i = 0; i < g_table_count; ++i) {
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

static void update_top_status(const freqresp_ui_status_t *s)
{
    char buf[128];
    const char *link = "WAIT";

    if (s->link_ok != 0U) {
        link = "OK";
    } else if (s->frame_errors != 0U || s->timeouts != 0U) {
        link = "ERR";
    }

    snprintf(buf, sizeof(buf), "Link: %s    State: %s", link, state_text(s->state));
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
    format_freq(fc, sizeof(fc), s->cutoff_freq_hz);

    snprintf(buf, sizeof(buf), "Freq: %s Hz", freq);
    lv_label_set_text(g_freq, buf);

    snprintf(buf, sizeof(buf), "Vin : %s", vin);
    lv_label_set_text(g_vin, buf);

    snprintf(buf, sizeof(buf), "Vout: %s", vout);
    lv_label_set_text(g_vout, buf);

    snprintf(buf,
             sizeof(buf),
             "Gain : %s   Theory: %s",
             gain,
             theory);
    lv_label_set_text(g_gain_line, buf);

    snprintf(buf,
             sizeof(buf),
             "Error: %s    %s",
             err,
             s->error_x10 <= 50 ? "PASS" : "FAIL");
    lv_label_set_text(g_error_line, buf);
    lv_obj_set_style_text_color(g_gain_line,
                                lv_color_hex(s->error_x10 <= 50 ? COLOR_GREEN : COLOR_RED),
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(g_error_line,
                                lv_color_hex(s->error_x10 <= 50 ? COLOR_GREEN : COLOR_RED),
                                LV_PART_MAIN);

    snprintf(buf, sizeof(buf), "Phase: %s", phase);
    lv_label_set_text(g_phase, buf);

    snprintf(buf, sizeof(buf), "Type : %s", filter_type_text(s->filter_type));
    lv_label_set_text(g_type, buf);

    snprintf(buf, sizeof(buf), "fc   : %s Hz", fc);
    lv_label_set_text(g_fc, buf);
}

static void start_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!read_and_validate_inputs()) {
        return;
    }

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
    clear_table_and_curve();
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
        clear_table_and_curve();
        SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
    }
}

static void theory_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        set_msg("TODO", COLOR_YELLOW);
    }
}

static void create_main_page(void);
static void create_full_table_page(void);

static void keyboard_event_cb(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        read_and_validate_inputs();
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
            test_screen_update_measurement(&g_last_status);
        }
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
    lv_obj_t *btn_theory = create_button(screen, "Theory", 750, 137, 100);
    lv_obj_add_event_cb(btn_start, start_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_stop, stop_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_clear, clear_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn_theory, theory_button_event_cb, LV_EVENT_CLICKED, nullptr);

    g_msg = create_label(screen, "Ready", 870, 145, 130, &lv_font_montserrat_18, COLOR_GREEN);

    create_hline(screen, 190);

    create_vline(screen, 430, 195, 320);

    create_label(screen, "Current Point", 24, 205, 250, &lv_font_montserrat_20, COLOR_BLUE);
    g_freq = create_label(screen, "Freq: ----- Hz", 24, 245, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_vin = create_label(screen, "Vin : -.--- V", 24, 275, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_vout = create_label(screen, "Vout: -.--- V", 24, 305, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_gain_line = create_label(screen, "Gain : -.---   Theory: -.---", 24, 335, 380, &lv_font_montserrat_18, COLOR_GREEN);
    g_error_line = create_label(screen, "Error: -.-%    PASS", 24, 365, 380, &lv_font_montserrat_18, COLOR_GREEN);
    g_phase = create_label(screen, "Phase: -.- deg", 24, 395, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_type = create_label(screen, "Type : Unknown", 24, 425, 380, &lv_font_montserrat_18, COLOR_TEXT);
    g_fc = create_label(screen, "fc   : ----- Hz", 24, 455, 380, &lv_font_montserrat_18, COLOR_TEXT);

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
    create_main_page();

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

    append_table_point(s);

    g_last_status = *s;
    g_have_last_status = true;

    if (!g_main_page_active || g_top_status == nullptr) {
        return;
    }

    update_top_status(s);
    update_current_point(s);
    update_latest_line(s);

    if (g_table_full) {
        set_msg("FULL", COLOR_RED);
    }
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
    g_top_status = nullptr;
    g_msg = nullptr;
    g_chart = nullptr;
    g_chart_gain = nullptr;
    g_latest = nullptr;
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
