#include "test_screen.h"

#include "lvgl.h"

#include <stdio.h>

#define TEST_SCREEN_W 1024
#define TEST_SCREEN_H 600

// 颜色
#define COLOR_BG        0x0B1020
#define COLOR_LINE      0x334155
#define COLOR_TEXT      0xF8FAFC
#define COLOR_SUBTEXT   0xCBD5E1
#define COLOR_BLUE      0x93C5FD
#define COLOR_GREEN     0x86EFAC
#define COLOR_YELLOW    0xFBBF24
#define COLOR_RED       0xFCA5A5

// 顶部
static lv_obj_t *g_title = nullptr;
static lv_obj_t *g_status = nullptr;

// 控制区
static lv_obj_t *g_btn_start = nullptr;
static lv_obj_t *g_btn_stop = nullptr;
static lv_obj_t *g_btn_basic = nullptr;
static lv_obj_t *g_btn_adv = nullptr;

// 实时数据
static lv_obj_t *g_input = nullptr;
static lv_obj_t *g_adc_code = nullptr;
static lv_obj_t *g_progress = nullptr;
static lv_obj_t *g_bar = nullptr;
static lv_obj_t *g_sample = nullptr;

// 静态参数
static lv_obj_t *g_offset = nullptr;
static lv_obj_t *g_gain = nullptr;
static lv_obj_t *g_inl = nullptr;
static lv_obj_t *g_dnl = nullptr;
static lv_obj_t *g_missing = nullptr;
static lv_obj_t *g_conv_time = nullptr;

// 底部
static lv_obj_t *g_link = nullptr;
static lv_obj_t *g_hint = nullptr;

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
    lv_obj_set_size(btn, 132, 48);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_center(label);

    return btn;
}

static void format_voltage(char *buf, size_t len, uint32_t mv)
{
    snprintf(buf,
             len,
             "%lu.%03lu V",
             static_cast<unsigned long>(mv / 1000U),
             static_cast<unsigned long>(mv % 1000U));
}

static void format_lsb_x1000(char *buf, size_t len, int32_t value)
{
    const char *sign = "";
    int32_t v = value;

    if (v < 0) {
        sign = "-";
        v = -v;
    }

    snprintf(buf,
             len,
             "%s%ld.%03ld LSB",
             sign,
             static_cast<long>(v / 1000),
             static_cast<long>(v % 1000));
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

    // 0 ~ 64：标题区
    g_title = create_label(screen,
                           "ADC Characteristic Test",
                           24,
                           16,
                           430,
                           &lv_font_montserrat_24,
                           COLOR_TEXT);

    g_status = create_label(screen,
                            "Basic 8-bit SAR | Waiting",
                            530,
                            18,
                            470,
                            &lv_font_montserrat_20,
                            COLOR_BLUE);

    create_hline(screen, 64);

    // 64 ~ 138：控制区
    g_btn_start = create_button(screen, "Start", 24, 78);
    g_btn_stop  = create_button(screen, "Stop", 174, 78);
    g_btn_basic = create_button(screen, "8-bit", 324, 78);
    g_btn_adv   = create_button(screen, "12-bit", 474, 78);

    create_label(screen,
                 "Press Start once, then auto refresh",
                 640,
                 88,
                 360,
                 &lv_font_montserrat_20,
                 COLOR_YELLOW);

    create_hline(screen, 144);

    // 144 ~ 470：主显示区
    create_label(screen,
                 "Live Data",
                 24,
                 162,
                 460,
                 &lv_font_montserrat_24,
                 COLOR_BLUE);

    create_label(screen,
                 "Static Parameters",
                 540,
                 162,
                 460,
                 &lv_font_montserrat_24,
                 COLOR_BLUE);

    create_vline(screen, 512, 154, 316);

    // 左侧 Live Data，x=24, 宽 460
    g_input = create_label(screen,
                           "Input: ---.--- V",
                           24,
                           214,
                           460,
                           &lv_font_montserrat_24,
                           COLOR_GREEN);

    g_adc_code = create_label(screen,
                              "ADC: --- / ---",
                              24,
                              262,
                              460,
                              &lv_font_montserrat_24,
                              COLOR_TEXT);

    g_progress = create_label(screen,
                              "Progress: 0.0%",
                              24,
                              310,
                              460,
                              &lv_font_montserrat_24,
                              COLOR_SUBTEXT);

    g_sample = create_label(screen,
                            "Sample: 0 / 0",
                            24,
                            358,
                            460,
                            &lv_font_montserrat_24,
                            COLOR_SUBTEXT);

    g_bar = lv_bar_create(screen);
    lv_obj_set_pos(g_bar, 24, 418);
    lv_obj_set_size(g_bar, 440, 28);
    lv_bar_set_range(g_bar, 0, 1000);
    lv_bar_set_value(g_bar, 0, LV_ANIM_OFF);

    // 右侧 Static Parameters，x=540, 宽 460
    g_offset = create_label(screen,
                            "Offset: -- uV",
                            540,
                            214,
                            460,
                            &lv_font_montserrat_24,
                            COLOR_TEXT);

    g_gain = create_label(screen,
                          "Gain: -- ppm",
                          540,
                          258,
                          460,
                          &lv_font_montserrat_24,
                          COLOR_TEXT);

    g_inl = create_label(screen,
                         "INL: -- LSB",
                         540,
                         302,
                         460,
                         &lv_font_montserrat_24,
                         COLOR_TEXT);

    g_dnl = create_label(screen,
                         "DNL: -- LSB",
                         540,
                         346,
                         460,
                         &lv_font_montserrat_24,
                         COLOR_TEXT);

    g_missing = create_label(screen,
                             "Missing: --",
                             540,
                             390,
                             460,
                             &lv_font_montserrat_24,
                             COLOR_TEXT);

    g_conv_time = create_label(screen,
                               "Conv Time: -- ns",
                               540,
                               434,
                               460,
                               &lv_font_montserrat_24,
                               COLOR_TEXT);

    create_hline(screen, 486);

    // 486 ~ 600：底部状态区
    g_link = create_label(screen,
                          "SPI: packets=0  err=0  timeout=0",
                          24,
                          506,
                          960,
                          &lv_font_montserrat_24,
                          COLOR_BLUE);

    g_hint = create_label(screen,
                          "Target: 8-bit static test <=120s | INL DNL Offset Gain Missing Code",
                          24,
                          552,
                          960,
                          &lv_font_montserrat_20,
                          COLOR_YELLOW);
}

void test_screen_update_measurement(const adc_ui_status_t *s)
{
    if (s == nullptr) {
        return;
    }

    char buf[128];

    char vin[32];
    format_voltage(vin, sizeof(vin), s->input_mv);

    snprintf(buf, sizeof(buf), "Input: %s", vin);
    lv_label_set_text(g_input, buf);

    const uint32_t max_code =
        (s->adc_bits >= 31U) ? 0xFFFFFFFFU : ((1UL << s->adc_bits) - 1U);

    snprintf(buf,
             sizeof(buf),
             "ADC: %lu / %lu (%lu-bit)",
             static_cast<unsigned long>(s->adc_code),
             static_cast<unsigned long>(max_code),
             static_cast<unsigned long>(s->adc_bits));
    lv_label_set_text(g_adc_code, buf);

    snprintf(buf,
             sizeof(buf),
             "Progress: %lu.%lu%%",
             static_cast<unsigned long>(s->progress_permille / 10U),
             static_cast<unsigned long>(s->progress_permille % 10U));
    lv_label_set_text(g_progress, buf);

    snprintf(buf,
             sizeof(buf),
             "Sample: %lu / %lu",
             static_cast<unsigned long>(s->sample_index),
             static_cast<unsigned long>(s->total_samples));
    lv_label_set_text(g_sample, buf);

    lv_bar_set_value(g_bar, s->progress_permille, LV_ANIM_OFF);

    snprintf(buf,
             sizeof(buf),
             "Offset: %ld uV",
             static_cast<long>(s->offset_error_uv));
    lv_label_set_text(g_offset, buf);

    snprintf(buf,
             sizeof(buf),
             "Gain: %ld ppm",
             static_cast<long>(s->gain_error_ppm));
    lv_label_set_text(g_gain, buf);

    char inl[32];
    char dnl[32];

    format_lsb_x1000(inl, sizeof(inl), s->inl_lsb_x1000);
    format_lsb_x1000(dnl, sizeof(dnl), s->dnl_lsb_x1000);

    snprintf(buf, sizeof(buf), "INL: %s", inl);
    lv_label_set_text(g_inl, buf);

    snprintf(buf, sizeof(buf), "DNL: %s", dnl);
    lv_label_set_text(g_dnl, buf);

    snprintf(buf,
             sizeof(buf),
             "Missing: %lu",
             static_cast<unsigned long>(s->missing_codes));
    lv_label_set_text(g_missing, buf);

    snprintf(buf,
             sizeof(buf),
             "Conv Time: %lu ns",
             static_cast<unsigned long>(s->conversion_time_ns));
    lv_label_set_text(g_conv_time, buf);

    snprintf(buf,
             sizeof(buf),
             "SPI: packets=%lu  err=%lu  timeout=%lu",
             static_cast<unsigned long>(s->packets),
             static_cast<unsigned long>(s->frame_errors),
             static_cast<unsigned long>(s->timeouts));
    lv_label_set_text(g_link, buf);

    lv_label_set_text(g_status, "Basic 8-bit SAR | Running");
}