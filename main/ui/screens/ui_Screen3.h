// LifeLink Settings Screen (Screen 3)
// Phone number configuration for SMS alerts

#ifndef UI_SCREEN3_H
#define UI_SCREEN3_H

#ifdef __cplusplus
extern "C"
{
#endif

    // SCREEN: ui_Screen3
    extern void ui_Screen3_screen_init(void);
    extern void ui_Screen3_screen_destroy(void);
    extern void ui_event_Screen3(lv_event_t *e);
    extern lv_obj_t *ui_Screen3;

    // Phone number text area (accessible from lifelink.cpp)
    extern lv_obj_t *ui_TextAreaPhone;
    extern lv_obj_t *ui_SwitchBLE;
    extern lv_obj_t *ui_SwitchSound;

    // Returns the currently entered phone number string
    const char *ui_get_phone_number(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
