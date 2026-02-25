// This file implements the Fall Simulation Screen (Screen 4)

#include "../ui.h"
#include <stdio.h>

lv_obj_t *ui_Screen4 = NULL;
lv_obj_t *ui_LabelFallQ = NULL;
lv_obj_t *ui_LabelCountdown = NULL;
lv_obj_t *ui_LabelTapCancel = NULL;

static lv_timer_t *countdown_timer = NULL;
static int countdown_val = 5;

extern void trigger_fall_sms(void); // Defined in lifelink.cpp
extern void trigger_real_fall_sms(void);

bool g_is_simulated_fall = true;

// Timer callback: Decrements counter, updates UI, triggers SMS at 0
static void scr4_countdown_cb(lv_timer_t *timer)
{
    countdown_val--;

    if (countdown_val <= 0)
    {
        lv_timer_del(timer);
        countdown_timer = NULL;

        // Trigger the actual fallback logic natively based on simulation flag
        if (g_is_simulated_fall)
        {
            trigger_fall_sms();
        }
        else
        {
            trigger_real_fall_sms();
        }

        // Return to main watch face
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_Screen1_screen_init);
    }
    else
    {
        // Update the big number
        if (ui_LabelCountdown)
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", countdown_val);
            lv_label_set_text(ui_LabelCountdown, buf);
        }
    }
}

// Tap Event to cancel the simulation
void scr4_tap_cb(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        // Cancel the timer
        if (countdown_timer != NULL)
        {
            lv_timer_del(countdown_timer);
            countdown_timer = NULL;
        }

        // Return safely to Screen 1
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_Screen1_screen_init);
    }
}

void start_fall_countdown_ui(bool is_simulated)
{
    g_is_simulated_fall = is_simulated;
    countdown_val = 5; // Reset the timer logic

    // Initialize the new screen
    _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_Screen4_screen_init);

    // Start the LVGL timer (1000ms = 1 sec)
    if (countdown_timer == NULL)
    {
        countdown_timer = lv_timer_create(scr4_countdown_cb, 1000, NULL);
    }
}

// Button click from Screen 2 to trigger this simulation screen
void btn_simulate_fall_cb(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        start_fall_countdown_ui(true);
    }
}

void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);                                          /// Flags
    lv_obj_set_style_bg_color(ui_Screen4, lv_color_hex(0x8B0000), LV_PART_MAIN | LV_STATE_DEFAULT); // Dark Red alert
    lv_obj_set_style_bg_opa(ui_Screen4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelFallQ = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_LabelFallQ, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_LabelFallQ, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_LabelFallQ, 0);
    lv_obj_set_y(ui_LabelFallQ, -80);
    lv_obj_set_align(ui_LabelFallQ, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelFallQ, "DA LI STE PALI?");
    lv_obj_set_style_text_font(ui_LabelFallQ, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_LabelFallQ, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelCountdown = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_LabelCountdown, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_LabelCountdown, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_LabelCountdown, 0);
    lv_obj_set_y(ui_LabelCountdown, 0);
    lv_obj_set_align(ui_LabelCountdown, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelCountdown, "5");
    lv_obj_set_style_text_font(ui_LabelCountdown, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_LabelCountdown, lv_color_hex(0xFFDD00), LV_PART_MAIN | LV_STATE_DEFAULT); // Warning Yellow

    ui_LabelTapCancel = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_LabelTapCancel, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_LabelTapCancel, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_LabelTapCancel, 0);
    lv_obj_set_y(ui_LabelTapCancel, 80);
    lv_obj_set_align(ui_LabelTapCancel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTapCancel, "TAPNITE AKO NISTE");
    lv_obj_set_style_text_font(ui_LabelTapCancel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_LabelTapCancel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Make the entire screen clickable (to cancel)
    lv_obj_add_flag(ui_Screen4, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_Screen4, scr4_tap_cb, LV_EVENT_ALL, NULL);
}

void ui_Screen4_screen_destroy(void)
{
    if (ui_Screen4)
        lv_obj_del(ui_Screen4);

    ui_Screen4 = NULL;
    ui_LabelFallQ = NULL;
    ui_LabelCountdown = NULL;
    ui_LabelTapCancel = NULL;
}
