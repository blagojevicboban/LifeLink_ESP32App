idf_component_register(
    SRCS "example_qspi_with_ram.cpp" "../example_qspi_with_ram.c"
         "ui/ui.c"
         "ui/ui_helpers.c"
         "ui/screens/ui_Screen1.c"
         "ui/screens/ui_Screen2.c"
         "ui/images/ui_img_logo128_png.c"
    INCLUDE_DIRS "."
                 "ui")

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
