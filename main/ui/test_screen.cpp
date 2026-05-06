#include "test_screen.h"

#include "lvgl.h"

#include <stdio.h>

#define TEST_SCREEN_W 1024
#define TEST_SCREEN_H 600

static lv_obj_t *g_touch_label = nullptr;
static lv_obj_t *g_log_label = nullptr;
static lv_obj_t *g_button_count_label = nullptr;
static lv_obj_t *g_slider_value_label = nullptr;
static lv_obj_t *g_switch_state_label = nullptr;
static lv_obj_t *g_bar = nullptr;
static lv_obj_t *g_keyboard = nullptr;

static int g_button_count = 0;

static void set_log(const char *text)
{
    if (g_log_label != nullptr) {
        lv_label_set_text(g_log_label, text);
    }
}

static lv_obj_t *create_panel(lv_obj_t *parent, int32_t w, int32_t h)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_style_radius(panel, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x3A4658), LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x18212F), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

static lv_obj_t *create_caption(lv_obj_t *parent, const char *text, int32_t x, int32_t y)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(0xC8D2E0), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_pos(label, x, y);
    return label;
}

static void touch_area_event_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING || code == LV_EVENT_CLICKED) {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t point = {0, 0};

        if (indev != nullptr) {
            lv_indev_get_point(indev, &point);

            char buf[96];
            snprintf(buf, sizeof(buf), "Touch X: %d    Y: %d", (int)point.x, (int)point.y);

            if (g_touch_label != nullptr) {
                lv_label_set_text(g_touch_label, buf);
            }

            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf), "Last event: Touch area X=%d, Y=%d", (int)point.x, (int)point.y);
            set_log(log_buf);
        }
    }
}

static void button_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        g_button_count++;

        char buf[64];
        snprintf(buf, sizeof(buf), "Button Count: %d", g_button_count);

        if (g_button_count_label != nullptr) {
            lv_label_set_text(g_button_count_label, buf);
        }

        set_log("Last event: Button clicked");
    }
}

static void slider_event_cb(lv_event_t *event)
{
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(event);
    int32_t value = lv_slider_get_value(slider);

    char buf[64];
    snprintf(buf, sizeof(buf), "Slider: %ld%%", (long)value);

    if (g_slider_value_label != nullptr) {
        lv_label_set_text(g_slider_value_label, buf);
    }

    if (g_bar != nullptr) {
        lv_bar_set_value(g_bar, value, LV_ANIM_OFF);
    }

    char log_buf[96];
    snprintf(log_buf, sizeof(log_buf), "Last event: Slider changed to %ld%%", (long)value);
    set_log(log_buf);
}

static void switch_event_cb(lv_event_t *event)
{
    lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(event);

    bool checked = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if (g_switch_state_label != nullptr) {
        lv_label_set_text(g_switch_state_label, checked ? "Switch: ON" : "Switch: OFF");
    }

    set_log(checked ? "Last event: Switch ON" : "Last event: Switch OFF");
}

static void dropdown_event_cb(lv_event_t *event)
{
    lv_obj_t *dropdown = (lv_obj_t *)lv_event_get_target(event);

    if (lv_event_get_code(event) == LV_EVENT_VALUE_CHANGED) {
        char selected[64];
        lv_dropdown_get_selected_str(dropdown, selected, sizeof(selected));

        char log_buf[128];
        snprintf(log_buf, sizeof(log_buf), "Last event: Dropdown selected [%s]", selected);
        set_log(log_buf);
    }
}

static void checkbox_event_cb(lv_event_t *event)
{
    lv_obj_t *checkbox = (lv_obj_t *)lv_event_get_target(event);

    bool checked = lv_obj_has_state(checkbox, LV_STATE_CHECKED);
    set_log(checked ? "Last event: Checkbox checked" : "Last event: Checkbox unchecked");
}

static void textarea_event_cb(lv_event_t *event)
{
    lv_obj_t *textarea = (lv_obj_t *)lv_event_get_target(event);
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_FOCUSED) {
        if (g_keyboard != nullptr) {
            lv_keyboard_set_textarea(g_keyboard, textarea);
            lv_obj_clear_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
        set_log("Last event: Textarea focused");
    } else if (code == LV_EVENT_DEFOCUSED || code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (g_keyboard != nullptr) {
            lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
            lv_keyboard_set_textarea(g_keyboard, nullptr);
        }
        set_log("Last event: Textarea input finished");
    } else if (code == LV_EVENT_VALUE_CHANGED) {
        set_log("Last event: Textarea value changed");
    }
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
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *title_bar = lv_obj_create(screen);
    lv_obj_set_size(title_bar, 1024, 60);
    lv_obj_set_pos(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(title_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(title_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(0x0B1220), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(title_bar, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(title_bar);
    lv_label_set_text(title, "ESP32-P4 LCD & Touch Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 24, 0);

    lv_obj_t *resolution = lv_label_create(title_bar);
    lv_label_set_text(resolution, "1024 x 600");
    lv_obj_set_style_text_color(resolution, lv_color_hex(0x8FD6C8), LV_PART_MAIN);
    lv_obj_set_style_text_font(resolution, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_align(resolution, LV_ALIGN_RIGHT_MID, -24, 0);

    lv_obj_t *touch_panel = create_panel(screen, 360, 450);
    lv_obj_set_pos(touch_panel, 20, 80);
    lv_obj_add_flag(touch_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(touch_panel, touch_area_event_cb, LV_EVENT_ALL, nullptr);

    lv_obj_t *touch_title = lv_label_create(touch_panel);
    lv_label_set_text(touch_title, "Touch Area");
    lv_obj_set_style_text_color(touch_title, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_set_style_text_font(touch_title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(touch_title, LV_ALIGN_TOP_MID, 0, 28);

    g_touch_label = lv_label_create(touch_panel);
    lv_label_set_text(g_touch_label, "Touch X: --    Y: --");
    lv_obj_set_style_text_color(g_touch_label, lv_color_hex(0x8FD6C8), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_touch_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(g_touch_label, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t *touch_hint = lv_label_create(touch_panel);
    lv_label_set_text(touch_hint, "Press or drag here\nto test touch position");
    lv_obj_set_style_text_color(touch_hint, lv_color_hex(0xA9B4C3), LV_PART_MAIN);
    lv_obj_set_style_text_align(touch_hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(touch_hint, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(touch_hint, LV_ALIGN_BOTTOM_MID, 0, -32);

    lv_obj_t *control_panel = create_panel(screen, 604, 450);
    lv_obj_set_pos(control_panel, 400, 80);

    create_caption(control_panel, "Button Test", 24, 24);

    lv_obj_t *button = lv_button_create(control_panel);
    lv_obj_set_size(button, 160, 52);
    lv_obj_set_pos(button, 24, 56);
    lv_obj_add_event_cb(button, button_event_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *button_label = lv_label_create(button);
    lv_label_set_text(button_label, "Click Me");
    lv_obj_center(button_label);

    g_button_count_label = lv_label_create(control_panel);
    lv_label_set_text(g_button_count_label, "Button Count: 0");
    lv_obj_set_style_text_color(g_button_count_label, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_button_count_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_pos(g_button_count_label, 210, 70);

    create_caption(control_panel, "Slider / Progress Test", 24, 128);

    lv_obj_t *slider = lv_slider_create(control_panel);
    lv_obj_set_size(slider, 300, 24);
    lv_obj_set_pos(slider, 24, 166);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    g_slider_value_label = lv_label_create(control_panel);
    lv_label_set_text(g_slider_value_label, "Slider: 50%");
    lv_obj_set_style_text_color(g_slider_value_label, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_slider_value_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_pos(g_slider_value_label, 350, 158);

    g_bar = lv_bar_create(control_panel);
    lv_obj_set_size(g_bar, 300, 22);
    lv_obj_set_pos(g_bar, 24, 212);
    lv_bar_set_range(g_bar, 0, 100);
    lv_bar_set_value(g_bar, 50, LV_ANIM_OFF);

    create_caption(control_panel, "Switch / Dropdown / Checkbox", 24, 260);

    lv_obj_t *sw = lv_switch_create(control_panel);
    lv_obj_set_pos(sw, 24, 296);
    lv_obj_add_event_cb(sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    g_switch_state_label = lv_label_create(control_panel);
    lv_label_set_text(g_switch_state_label, "Switch: OFF");
    lv_obj_set_style_text_color(g_switch_state_label, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_switch_state_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_pos(g_switch_state_label, 100, 300);

    lv_obj_t *dropdown = lv_dropdown_create(control_panel);
    lv_dropdown_set_options(dropdown,
                            "Option 1\n"
                            "Option 2\n"
                            "Option 3\n"
                            "Touch Test");
    lv_obj_set_size(dropdown, 180, 46);
    lv_obj_set_pos(dropdown, 260, 286);
    lv_obj_add_event_cb(dropdown, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *checkbox = lv_checkbox_create(control_panel);
    lv_checkbox_set_text(checkbox, "Check me");
    lv_obj_set_pos(checkbox, 460, 298);
    lv_obj_set_style_text_color(checkbox, lv_color_hex(0xF5F7FA), LV_PART_MAIN);
    lv_obj_add_event_cb(checkbox, checkbox_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    create_caption(control_panel, "Textarea Test", 24, 358);

    lv_obj_t *textarea = lv_textarea_create(control_panel);
    lv_obj_set_size(textarea, 360, 54);
    lv_obj_set_pos(textarea, 24, 386);
    lv_textarea_set_placeholder_text(textarea, "Tap here to input...");
    lv_textarea_set_one_line(textarea, true);
    lv_obj_add_event_cb(textarea, textarea_event_cb, LV_EVENT_ALL, nullptr);

    g_log_label = lv_label_create(screen);
    lv_label_set_text(g_log_label, "Last event: UI created");
    lv_obj_set_size(g_log_label, 980, 40);
    lv_obj_set_pos(g_log_label, 24, 548);
    lv_obj_set_style_text_color(g_log_label, lv_color_hex(0xFFD166), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_log_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_label_set_long_mode(g_log_label, LV_LABEL_LONG_SCROLL_CIRCULAR);

    g_keyboard = lv_keyboard_create(screen);
    lv_obj_set_size(g_keyboard, 1024, 180);
    lv_obj_align(g_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(g_keyboard, LV_OBJ_FLAG_HIDDEN);
}
