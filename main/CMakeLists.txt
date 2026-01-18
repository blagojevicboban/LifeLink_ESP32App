idf_component_register(
    SRCS "lifelink.cpp" "../lifelink.c"
         "ui/ui.c"
         "ui/ui_helpers.c"
         "ui/screens/ui_Screen1.c"
         "ui/screens/ui_Screen2.c"
         "ui/images/ui_img_logo128_png.c"
         "ui/images/ui_img_pulse36_png.c"
         "ui/images/ui_img_1716151603.c"
    INCLUDE_DIRS "."
                 "ui")

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
