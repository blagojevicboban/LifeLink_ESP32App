idf_component_register(SRCS "max30102.cpp" "max30102_algorithm.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer log)
