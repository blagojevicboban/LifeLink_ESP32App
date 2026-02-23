#include "ui.h"
#include "esp_log.h"

// External functions defined in lifelink.cpp
extern void toggle_ble(bool enable);
extern void toggle_sound(bool enable);

void ui_event_SwitchBLE(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_VALUE_CHANGED)
    {
        bool state = lv_obj_has_state(target, LV_STATE_CHECKED);
        ESP_LOGI("UI", "BLE Switch: %d", state);
        toggle_ble(state);
    }
}

void ui_event_SwitchSound(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_VALUE_CHANGED)
    {
        bool state = lv_obj_has_state(target, LV_STATE_CHECKED);
        ESP_LOGI("UI", "Sound Switch: %d", state);
        toggle_sound(state);
    }
}
