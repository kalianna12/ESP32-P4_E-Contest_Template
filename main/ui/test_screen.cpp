#include "test_screen.h"

#include "lvgl.h"
#include "spilink.h"

#include <stdint.h>
#include <stdio.h>

#define TEST_SCREEN_W 1024
#define TEST_SCREEN_H 600

#define COLOR_BG        0x0B1020
#define COLOR_LINE      0x334155
#define COLOR_TEXT      0xF8FAFC
#define COLOR_SUBTEXT   0xCBD5E1
#define COLOR_BLUE      0x93C5FD
#define COLOR_GREEN     0x86EFAC
#define COLOR_YELLOW    0xFBBF24
#define COLOR_RED       0xFCA5A5

#define DYNAMIC_INVALID INT32_MIN

#define CMD_START      1U
#define CMD_STOP       2U
#define CMD_SET_MODE   4U

#define ADC_TEST_MODE_STATIC_8BIT  0U
#define ADC_TEST_MODE_STATIC_12BIT 1U

static lv_obj_t *g_title = nullptr;
static lv_obj_t *g_status = nullptr;

static lv_obj_t *g_btn_start = nullptr;
static lv_obj_t *g_btn_stop = nullptr;
static lv_obj_t *g_btn_basic = nullptr;
static lv_obj_t *g_btn_adv = nullptr;
static lv_obj_t *g_source = nullptr;
static lv_obj_t *g_link_state = nullptr;

static lv_obj_t *g_dut_code = nullptr;
static lv_obj_t *g_dut_avg = nullptr;
static lv_obj_t *g_dut_conv = nullptr;
static lv_obj_t *g_dut_step = nullptr;

static lv_obj_t *g_mon_voltage = nullptr;
static lv_obj_t *g_mon_raw12 = nullptr;
static lv_obj_t *g_mon_status = nullptr;
static lv_obj_t *g_mon_source = nullptr;

static lv_obj_t *g_static_offset = nullptr;
static lv_obj_t *g_static_gain = nullptr;
static lv_obj_t *g_static_dnl = nullptr;
static lv_obj_t *g_static_inl = nullptr;
static lv_obj_t *g_static_missing = nullptr;
static lv_obj_t *g_static_elapsed = nullptr;

static lv_obj_t *g_dynamic_line1 = nullptr;
static lv_obj_t *g_dynamic_line2 = nullptr;

static lv_obj_t *g_link = nullptr;
static lv_obj_t *g_hint = nullptr;
static lv_obj_t *g_bar = nullptr;

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

static lv_obj_t *create_button(lv_obj_t *parent, const char *text, int32_t x, int32_t y)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, 112, 42);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_center(label);

    return btn;
}

static const char *state_text(uint8_t state)
{
    switch (state) {
    case 0: return "Idle";
    case 1: return "Scanning";
    case 2: return "Calculating";
    case 3: return "Done";
    case 4: return "Error";
    case 5: return "Stopped";
    default: return "Unknown";
    }
}

static void start_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        SpiLink_SetPendingCommand(CMD_START, 0U, 0U);
    }
}

static void stop_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        SpiLink_SetPendingCommand(CMD_STOP, 0U, 0U);
    }
}

static void basic_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        SpiLink_SetPendingCommand(CMD_SET_MODE, ADC_TEST_MODE_STATIC_8BIT, 0U);
    }
}

static void adv_button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        SpiLink_SetPendingCommand(CMD_SET_MODE, ADC_TEST_MODE_STATIC_12BIT, 0U);
    }
}

static const char *mode_text(uint8_t mode)
{
    switch (mode) {
    case 0: return "Static 8-bit";
    case 1: return "Static 12-bit";
    case 2: return "Dynamic";
    default: return "Unknown";
    }
}

static const char *source_text(uint8_t source)
{
    switch (source) {
    case 0: return "AD9767";
    case 1: return "STM32_DAC";
    default: return "External";
    }
}

static uint32_t max_code_from_bits(uint32_t bits)
{
    if (bits == 0U) {
        bits = 8U;
    }

    if (bits >= 31U) {
        return 0xFFFFFFFFU;
    }

    return (1UL << bits) - 1U;
}

static void format_voltage(char *buf, size_t len, uint32_t mv)
{
    snprintf(buf,
             len,
             "%lu.%03lu V",
             static_cast<unsigned long>(mv / 1000U),
             static_cast<unsigned long>(mv % 1000U));
}

static void format_signed_fixed(char *buf, size_t len, int32_t value, uint32_t scale)
{
    const char *sign = "+";
    int32_t v = value;

    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%03ld",
             sign,
             static_cast<long>(v / static_cast<int32_t>(scale)),
             static_cast<long>(v % static_cast<int32_t>(scale)));
}

static void format_lsb_x1000(char *buf, size_t len, int32_t value)
{
    char value_buf[32];
    format_signed_fixed(value_buf, sizeof(value_buf), value, 1000U);
    snprintf(buf, len, "%s LSB", value_buf);
}

static void format_time_ns(char *buf, size_t len, uint32_t ns)
{
    if (ns >= 1000U) {
        snprintf(buf,
                 len,
                 "%lu.%03lu us",
                 static_cast<unsigned long>(ns / 1000U),
                 static_cast<unsigned long>(ns % 1000U));
        return;
    }

    snprintf(buf, len, "%lu ns", static_cast<unsigned long>(ns));
}

static void format_elapsed(char *buf, size_t len, uint32_t elapsed_ms)
{
    snprintf(buf,
             len,
             "%lu.%lu s",
             static_cast<unsigned long>(elapsed_ms / 1000U),
             static_cast<unsigned long>((elapsed_ms % 1000U) / 100U));
}

static void format_db_x100(char *buf, size_t len, const char *name, int32_t value)
{
    if (value == DYNAMIC_INVALID) {
        snprintf(buf, len, "%s: -- dB", name);
        return;
    }

    const char *sign = "";
    int32_t v = value;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s: %s%ld.%02ld dB",
             name,
             sign,
             static_cast<long>(v / 100),
             static_cast<long>(v % 100));
}

static void format_enob_x100(char *buf, size_t len, int32_t value)
{
    if (value == DYNAMIC_INVALID) {
        snprintf(buf, len, "ENOB: -- bit");
        return;
    }

    const char *sign = "";
    int32_t v = value;
    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "ENOB: %s%ld.%02ld bit",
             sign,
             static_cast<long>(v / 100),
             static_cast<long>(v % 100));
}

void test_screen_create(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(screen, TEST_SCREEN_W, TEST_SCREEN_H);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    g_title = create_label(screen,
                           "ADC Characteristic Test",
                           24,
                           16,
                           420,
                           &lv_font_montserrat_24,
                           COLOR_TEXT);

    g_status = create_label(screen,
                            "Mode: Static 8-bit  Idle",
                            500,
                            18,
                            500,
                            &lv_font_montserrat_20,
                            COLOR_BLUE);

    create_hline(screen, 64);

    g_btn_start = create_button(screen, "Start", 24, 78);
    g_btn_stop  = create_button(screen, "Stop", 160, 78);
    g_btn_basic = create_button(screen, "8-bit", 296, 78);
    g_btn_adv   = create_button(screen, "12-bit", 432, 78);

    lv_obj_add_event_cb(g_btn_start, start_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(g_btn_stop, stop_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(g_btn_basic, basic_button_event_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(g_btn_adv, adv_button_event_cb, LV_EVENT_CLICKED, nullptr);

    g_source = create_label(screen,
                            "Source: AD9767",
                            590,
                            84,
                            180,
                            &lv_font_montserrat_18,
                            COLOR_YELLOW);

    g_link_state = create_label(screen,
                                "Link: WAIT",
                                790,
                                84,
                                210,
                                &lv_font_montserrat_18,
                                COLOR_YELLOW);

    create_hline(screen, 136);

    create_label(screen,
                 "Live DUT ADC0832",
                 24,
                 150,
                 470,
                 &lv_font_montserrat_20,
                 COLOR_BLUE);

    create_label(screen,
                 "Live STM32 Monitor ADC",
                 540,
                 150,
                 460,
                 &lv_font_montserrat_20,
                 COLOR_BLUE);

    create_vline(screen, 512, 145, 140);

    g_dut_code = create_label(screen, "Code: --- / ---", 24, 184, 470, &lv_font_montserrat_18, COLOR_TEXT);
    g_dut_avg  = create_label(screen, "Avg: ---.---", 24, 212, 470, &lv_font_montserrat_18, COLOR_TEXT);
    g_dut_conv = create_label(screen, "Conv: --", 24, 240, 470, &lv_font_montserrat_18, COLOR_TEXT);
    g_dut_step = create_label(screen, "Step: 0 / 0", 24, 268, 470, &lv_font_montserrat_18, COLOR_SUBTEXT);

    g_mon_voltage = create_label(screen, "Voltage: ---.--- V", 540, 184, 460, &lv_font_montserrat_18, COLOR_GREEN);
    g_mon_raw12   = create_label(screen, "Raw12: ---- / 4095", 540, 212, 460, &lv_font_montserrat_18, COLOR_TEXT);
    g_mon_status  = create_label(screen, "Monitor: WARN", 540, 240, 460, &lv_font_montserrat_18, COLOR_YELLOW);
    g_mon_source  = create_label(screen, "Input: ---.--- V", 540, 268, 460, &lv_font_montserrat_18, COLOR_SUBTEXT);

    create_hline(screen, 292);

    create_label(screen, "Static Parameters", 24, 306, 960, &lv_font_montserrat_20, COLOR_BLUE);
    g_static_offset  = create_label(screen, "Offset: -- LSB", 24, 340, 450, &lv_font_montserrat_18, COLOR_TEXT);
    g_static_gain    = create_label(screen, "Gain: -- LSB / -- ppm", 520, 340, 480, &lv_font_montserrat_18, COLOR_TEXT);
    g_static_dnl     = create_label(screen, "DNL: -- / --", 24, 376, 450, &lv_font_montserrat_18, COLOR_TEXT);
    g_static_inl     = create_label(screen, "INL: -- / --", 520, 376, 480, &lv_font_montserrat_18, COLOR_TEXT);
    g_static_missing = create_label(screen, "Missing: --", 24, 412, 450, &lv_font_montserrat_18, COLOR_TEXT);
    g_static_elapsed = create_label(screen, "Elapsed: 0.0 s", 520, 412, 480, &lv_font_montserrat_18, COLOR_TEXT);

    g_bar = lv_bar_create(screen);
    lv_obj_set_pos(g_bar, 24, 438);
    lv_obj_set_size(g_bar, 976, 12);
    lv_bar_set_range(g_bar, 0, 1000);
    lv_bar_set_value(g_bar, 0, LV_ANIM_OFF);

    create_hline(screen, 455);

    create_label(screen, "Dynamic Parameters", 24, 468, 960, &lv_font_montserrat_20, COLOR_BLUE);
    g_dynamic_line1 = create_label(screen,
                                   "SNR: -- dB  SINAD: -- dB  ENOB: -- bit",
                                   24,
                                   500,
                                   960,
                                   &lv_font_montserrat_18,
                                   COLOR_YELLOW);
    g_dynamic_line2 = create_label(screen,
                                   "SFDR: -- dB  THD: -- dB",
                                   24,
                                   526,
                                   960,
                                   &lv_font_montserrat_18,
                                   COLOR_YELLOW);

    create_hline(screen, 545);

    g_link = create_label(screen,
                          "SPI packets=0 err=0 timeout=0",
                          24,
                          558,
                          500,
                          &lv_font_montserrat_18,
                          COLOR_BLUE);

    g_hint = create_label(screen,
                          "Hint: waiting for ADC0832 static test data",
                          540,
                          558,
                          460,
                          &lv_font_montserrat_18,
                          COLOR_YELLOW);
}

void test_screen_update_measurement(const adc_ui_status_t *s)
{
    if (s == nullptr) {
        return;
    }

    char buf[160];
    char a[48];
    char b[48];
    char c[48];

    snprintf(buf,
             sizeof(buf),
             "Mode: %s  %s",
             mode_text(s->mode),
             state_text(s->state));
    lv_label_set_text(g_status, buf);
    lv_obj_set_style_text_color(g_status,
                                lv_color_hex(s->state == 4U ? COLOR_RED : COLOR_BLUE),
                                LV_PART_MAIN);

    snprintf(buf, sizeof(buf), "Source: %s", source_text(s->source));
    lv_label_set_text(g_source, buf);

    const bool link_ok = (s->frame_errors == 0U && s->timeouts == 0U && s->packets > 0U);
    snprintf(buf, sizeof(buf), "Link: %s", link_ok ? "OK" : "WARN");
    lv_label_set_text(g_link_state, buf);
    lv_obj_set_style_text_color(g_link_state,
                                lv_color_hex(link_ok ? COLOR_GREEN : COLOR_YELLOW),
                                LV_PART_MAIN);

    const uint32_t max_code = max_code_from_bits(s->dut_adc_bits);
    snprintf(buf,
             sizeof(buf),
             "Code: %lu / %lu",
             static_cast<unsigned long>(s->dut_adc_code),
             static_cast<unsigned long>(max_code));
    lv_label_set_text(g_dut_code, buf);

    snprintf(buf,
             sizeof(buf),
             "Avg: %lu.%03lu",
             static_cast<unsigned long>(s->dut_adc_avg_x1000 / 1000U),
             static_cast<unsigned long>(s->dut_adc_avg_x1000 % 1000U));
    lv_label_set_text(g_dut_avg, buf);

    format_time_ns(a, sizeof(a), s->dut_conversion_time_ns);
    snprintf(buf, sizeof(buf), "Conv: %s", a);
    lv_label_set_text(g_dut_conv, buf);

    snprintf(buf,
             sizeof(buf),
             "Step: %lu / %lu  Progress: %lu.%lu%%",
             static_cast<unsigned long>(s->sample_index),
             static_cast<unsigned long>(s->total_samples),
             static_cast<unsigned long>(s->progress_permille / 10U),
             static_cast<unsigned long>(s->progress_permille % 10U));
    lv_label_set_text(g_dut_step, buf);

    format_voltage(a, sizeof(a), s->stm32_adc_mv);
    snprintf(buf, sizeof(buf), "Voltage: %s", a);
    lv_label_set_text(g_mon_voltage, buf);

    snprintf(buf,
             sizeof(buf),
             "Raw12: %lu / 4095",
             static_cast<unsigned long>(s->stm32_adc_raw12));
    lv_label_set_text(g_mon_raw12, buf);

    snprintf(buf, sizeof(buf), "Monitor: %s", s->monitor_ok ? "OK" : "WARN");
    lv_label_set_text(g_mon_status, buf);
    lv_obj_set_style_text_color(g_mon_status,
                                lv_color_hex(s->monitor_ok ? COLOR_GREEN : COLOR_YELLOW),
                                LV_PART_MAIN);

    format_voltage(a, sizeof(a), s->input_mv);
    snprintf(buf, sizeof(buf), "Input: %s", a);
    lv_label_set_text(g_mon_source, buf);

    format_lsb_x1000(a, sizeof(a), s->offset_error_lsb_x1000);
    snprintf(buf, sizeof(buf), "Offset: %s", a);
    lv_label_set_text(g_static_offset, buf);

    format_lsb_x1000(a, sizeof(a), s->gain_error_lsb_x1000);
    snprintf(buf,
             sizeof(buf),
             "Gain: %s / %ld ppm",
             a,
             static_cast<long>(s->gain_error_ppm));
    lv_label_set_text(g_static_gain, buf);

    format_lsb_x1000(a, sizeof(a), s->dnl_min_x1000);
    format_lsb_x1000(b, sizeof(b), s->dnl_max_x1000);
    snprintf(buf, sizeof(buf), "DNL: %s / %s", a, b);
    lv_label_set_text(g_static_dnl, buf);

    format_lsb_x1000(a, sizeof(a), s->inl_min_x1000);
    format_lsb_x1000(b, sizeof(b), s->inl_max_x1000);
    snprintf(buf, sizeof(buf), "INL: %s / %s", a, b);
    lv_label_set_text(g_static_inl, buf);

    snprintf(buf,
             sizeof(buf),
             "Missing: %lu",
             static_cast<unsigned long>(s->missing_codes));
    lv_label_set_text(g_static_missing, buf);

    format_elapsed(a, sizeof(a), s->elapsed_ms);
    snprintf(buf, sizeof(buf), "Elapsed: %s", a);
    lv_label_set_text(g_static_elapsed, buf);

    lv_bar_set_value(g_bar, s->progress_permille, LV_ANIM_OFF);

    format_db_x100(a, sizeof(a), "SNR", s->snr_db_x100);
    format_db_x100(b, sizeof(b), "SINAD", s->sinad_db_x100);
    format_enob_x100(c, sizeof(c), s->enob_x100);
    snprintf(buf, sizeof(buf), "%s  %s  %s", a, b, c);
    lv_label_set_text(g_dynamic_line1, buf);
    lv_obj_set_style_text_color(g_dynamic_line1,
                                lv_color_hex(s->snr_db_x100 == DYNAMIC_INVALID ? COLOR_YELLOW : COLOR_TEXT),
                                LV_PART_MAIN);

    format_db_x100(a, sizeof(a), "SFDR", s->sfdr_db_x100);
    format_db_x100(b, sizeof(b), "THD", s->thd_db_x100);
    snprintf(buf, sizeof(buf), "%s  %s", a, b);
    lv_label_set_text(g_dynamic_line2, buf);
    lv_obj_set_style_text_color(g_dynamic_line2,
                                lv_color_hex(s->sfdr_db_x100 == DYNAMIC_INVALID ? COLOR_YELLOW : COLOR_TEXT),
                                LV_PART_MAIN);

    snprintf(buf,
             sizeof(buf),
             "SPI packets=%lu err=%lu timeout=%lu",
             static_cast<unsigned long>(s->packets),
             static_cast<unsigned long>(s->frame_errors),
             static_cast<unsigned long>(s->timeouts));
    lv_label_set_text(g_link, buf);

    if (s->state == 4U) {
        lv_label_set_text(g_hint, "Error: check STM32 link or ADC0832 wiring");
        lv_obj_set_style_text_color(g_hint, lv_color_hex(COLOR_RED), LV_PART_MAIN);
    } else if (!s->monitor_ok) {
        lv_label_set_text(g_hint, "Hint: STM32 monitor ADC warning");
        lv_obj_set_style_text_color(g_hint, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN);
    } else {
        lv_label_set_text(g_hint, "Hint: static test data updating");
        lv_obj_set_style_text_color(g_hint, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN);
    }
}
