# Configuration: Choose between MAIN or MOTOR_TEST build
# Default to MAIN, can be overridden with -DBUILD_TYPE=MOTOR_TEST during build
if(NOT DEFINED BUILD_TYPE)
    set(BUILD_TYPE "MAIN")
endif()

if(BUILD_TYPE STREQUAL "MOTOR_TEST")
    file(GLOB_RECURSE srcs "motor_test.c" "includes/*.c")
    message(STATUS "Building MOTOR_TEST version")
else()
    file(GLOB_RECURSE srcs "main.c" "includes/*.c")
    message(STATUS "Building MAIN version (default)")
endif()

idf_component_register(SRCS "${srcs}"
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc bt nvs_flash
                       INCLUDE_DIRS "./includes")
