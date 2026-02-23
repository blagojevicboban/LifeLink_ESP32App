#ifndef _SQUARELINE_PROJECT_UI_SCREEN4_H
#define _SQUARELINE_PROJECT_UI_SCREEN4_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"
#include "../ui.h"

    void ui_Screen4_screen_init(void);
    void ui_Screen4_screen_destroy(void);
    void btn_simulate_fall_cb(lv_event_t *e);
    void scr4_tap_cb(lv_event_t *e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
