// LifeLink Settings Screen (Screen 3)
// Custom numpad for entering SMS alert phone number on 466x466 round AMOLED

#include "../ui.h"
#include "esp_log.h"
#include "gsm_a6.h"
#include <string.h>

lv_obj_t *ui_Screen3 = NULL;
lv_obj_t *ui_TextAreaPhone = NULL;
lv_obj_t *ui_SwitchBLE = NULL;
lv_obj_t *ui_SwitchSound = NULL;

static lv_obj_t *numpad_panel = NULL;
static lv_obj_t *label_title = NULL;
static lv_obj_t *label_saved = NULL;
static lv_obj_t *ui_LabelBLE = NULL;
static lv_obj_t *ui_LabelSound = NULL;

// Stored phone number (persisted in RAM, default value)
static char g_phone_number[20] = "+3816*****";

// --- Numpad button click handler ---
static void numpad_btn_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    const char *txt = lv_label_get_text(lv_obj_get_child(btn, 0));

    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0)
    {
        lv_textarea_del_char(ui_TextAreaPhone);
    }
    else
    {
        lv_textarea_add_text(ui_TextAreaPhone, txt);
    }

    // Auto-save the phone number as we type
    const char *phone = lv_textarea_get_text(ui_TextAreaPhone);
    strncpy(g_phone_number, phone, sizeof(g_phone_number) - 1);
    g_phone_number[sizeof(g_phone_number) - 1] = '\0';
}

// --- Helper to create a numpad button ---
static lv_obj_t *create_numpad_btn(lv_obj_t *parent, const char *text, int x, int y, int w, int h)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1A2A3A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, 255, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x3A5A7A), LV_PART_MAIN);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_center(lbl);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, LV_PART_MAIN);

    lv_obj_add_event_cb(btn, numpad_btn_cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

// The test_sms_btn_cb has been removed since the button is no longer needed

// --- Gesture navigation ---
void ui_event_Screen3(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Screen2_screen_init);
    }
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_Screen1_screen_init);
    }
}

// --- Screen Init ---
void ui_Screen3_screen_init(void)
{
    ui_Screen3 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_Screen3, lv_color_hex(0x000a14), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // --- Title ---
    label_title = lv_label_create(ui_Screen3);
    lv_obj_set_align(label_title, LV_ALIGN_TOP_MID);
    lv_obj_set_y(label_title, 20);
    lv_label_set_text(label_title, LV_SYMBOL_SETTINGS "  Podesavanja");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_20, LV_PART_MAIN);

    // --- BLE Switch ---
    ui_LabelBLE = lv_label_create(ui_Screen3);
    lv_obj_set_align(ui_LabelBLE, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_LabelBLE, "BLE");
    lv_obj_set_x(ui_LabelBLE, -70);
    lv_obj_set_y(ui_LabelBLE, 50);
    lv_obj_set_style_text_font(ui_LabelBLE, &lv_font_montserrat_16, LV_PART_MAIN);

    ui_SwitchBLE = lv_switch_create(ui_Screen3);
    lv_obj_set_size(ui_SwitchBLE, 45, 25);
    lv_obj_set_align(ui_SwitchBLE, LV_ALIGN_TOP_MID);
    lv_obj_set_x(ui_SwitchBLE, -20);
    lv_obj_set_y(ui_SwitchBLE, 45);
    lv_obj_add_state(ui_SwitchBLE, LV_STATE_CHECKED);
    lv_obj_add_event_cb(ui_SwitchBLE, ui_event_SwitchBLE, LV_EVENT_VALUE_CHANGED, NULL);

    // --- Sound Switch ---
    ui_LabelSound = lv_label_create(ui_Screen3);
    lv_obj_set_align(ui_LabelSound, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_LabelSound, "SND");
    lv_obj_set_x(ui_LabelSound, 35);
    lv_obj_set_y(ui_LabelSound, 50);
    lv_obj_set_style_text_font(ui_LabelSound, &lv_font_montserrat_16, LV_PART_MAIN);

    ui_SwitchSound = lv_switch_create(ui_Screen3);
    lv_obj_set_size(ui_SwitchSound, 45, 25);
    lv_obj_set_align(ui_SwitchSound, LV_ALIGN_TOP_MID);
    lv_obj_set_x(ui_SwitchSound, 85);
    lv_obj_set_y(ui_SwitchSound, 45);
    lv_obj_add_state(ui_SwitchSound, LV_STATE_CHECKED);
    lv_obj_add_event_cb(ui_SwitchSound, ui_event_SwitchSound, LV_EVENT_VALUE_CHANGED, NULL);

    // --- Phone number label ---
    lv_obj_t *lbl_phone = lv_label_create(ui_Screen3);
    lv_obj_set_align(lbl_phone, LV_ALIGN_TOP_MID);
    lv_obj_set_y(lbl_phone, 80);
    lv_label_set_text(lbl_phone, "SMS Broj:");
    lv_obj_set_style_text_color(lbl_phone, lv_color_hex(0x80C0FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_phone, &lv_font_montserrat_16, LV_PART_MAIN);

    // --- Text Area for phone number ---
    ui_TextAreaPhone = lv_textarea_create(ui_Screen3);
    lv_obj_set_size(ui_TextAreaPhone, 260, 38);
    lv_obj_set_align(ui_TextAreaPhone, LV_ALIGN_TOP_MID);
    lv_obj_set_y(ui_TextAreaPhone, 100);
    lv_textarea_set_max_length(ui_TextAreaPhone, 16);
    lv_textarea_set_one_line(ui_TextAreaPhone, true);
    lv_textarea_set_text(ui_TextAreaPhone, g_phone_number);
    lv_obj_set_style_text_font(ui_TextAreaPhone, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_TextAreaPhone, lv_color_hex(0x0A1A2A), LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_TextAreaPhone, lv_color_hex(0x00FF88), LV_PART_MAIN);
    lv_obj_set_style_border_color(ui_TextAreaPhone, lv_color_hex(0x3A5A7A), LV_PART_MAIN);

    // --- Saved confirmation label ---
    label_saved = lv_label_create(ui_Screen3);
    lv_obj_set_align(label_saved, LV_ALIGN_TOP_MID);
    lv_obj_set_y(label_saved, 140);
    lv_label_set_text(label_saved, "");
    lv_obj_set_style_text_font(label_saved, &lv_font_montserrat_16, LV_PART_MAIN);

    // --- Custom Numpad ---
    // Shifted down slightly to fit switches
    const int btn_w = 70;
    const int btn_h = 42;
    const int pad = 5;
    const int start_x = 122;
    const int start_y = 162;

    // Row 1: 1 2 3
    create_numpad_btn(ui_Screen3, "1", start_x + 0 * (btn_w + pad), start_y + 0 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "2", start_x + 1 * (btn_w + pad), start_y + 0 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "3", start_x + 2 * (btn_w + pad), start_y + 0 * (btn_h + pad), btn_w, btn_h);

    // Row 2: 4 5 6
    create_numpad_btn(ui_Screen3, "4", start_x + 0 * (btn_w + pad), start_y + 1 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "5", start_x + 1 * (btn_w + pad), start_y + 1 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "6", start_x + 2 * (btn_w + pad), start_y + 1 * (btn_h + pad), btn_w, btn_h);

    // Row 3: 7 8 9
    create_numpad_btn(ui_Screen3, "7", start_x + 0 * (btn_w + pad), start_y + 2 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "8", start_x + 1 * (btn_w + pad), start_y + 2 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "9", start_x + 2 * (btn_w + pad), start_y + 2 * (btn_h + pad), btn_w, btn_h);

    // Row 4: + 0 Backspace
    create_numpad_btn(ui_Screen3, "+", start_x + 0 * (btn_w + pad), start_y + 3 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, "0", start_x + 1 * (btn_w + pad), start_y + 3 * (btn_h + pad), btn_w, btn_h);
    create_numpad_btn(ui_Screen3, LV_SYMBOL_BACKSPACE, start_x + 2 * (btn_w + pad), start_y + 3 * (btn_h + pad), btn_w, btn_h);

    // --- Gesture for navigation ---
    lv_obj_add_event_cb(ui_Screen3, ui_event_Screen3, LV_EVENT_ALL, NULL);
}

void ui_Screen3_screen_destroy(void)
{
    if (ui_Screen3)
        lv_obj_del(ui_Screen3);
    ui_Screen3 = NULL;
    ui_TextAreaPhone = NULL;
    ui_SwitchBLE = NULL;
    ui_SwitchSound = NULL;
    numpad_panel = NULL;
    label_title = NULL;
    label_saved = NULL;
    ui_LabelBLE = NULL;
    ui_LabelSound = NULL;
}

const char *ui_get_phone_number(void)
{
    return g_phone_number;
}
