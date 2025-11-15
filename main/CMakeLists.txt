file(GLOB_RECURSE srcs "main.c" "includes/*.c")

idf_component_register(SRCS "${srcs}"
                       REQUIRES esp_driver_gpio esp_driver_i2c
                       INCLUDE_DIRS "./includes")
