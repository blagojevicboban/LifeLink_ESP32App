SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_logo128_png.c
    images/ui_img_pulse36_png.c
    images/ui_img_pulse36_png.c
    images/ui_img_1716151603.c
    ui_events.c)

add_library(ui ${SOURCES})
