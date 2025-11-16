file(GLOB_RECURSE srcs "main.c" "includes/*.c")

idf_component_register(SRCS "${srcs}"
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc bt nvs_flash
                       INCLUDE_DIRS "./includes")
