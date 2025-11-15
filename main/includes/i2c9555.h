/*
    TCA9555 I2C I/O Expander Driver (Support Multiple Devices)
    Can be reused for 9555 Series I2C I/O Expanders
    ** Must refix the define of IO pins according to the actual 9555 chip version **
    Only support normal digital I/O.
    NOT support advanced features like:
        - PWM
        - ADC
        - DAC

    Usage:
        1. Call i2c9555_add_device()
            - Parameters: 
                - gpio_num_t sda: GPIO number for SDA
                - gpio_num_t scl: GPIO number for SCL
                - uint16_t addr: Device address (0x40, 0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C, 0x4E)
                - gpio_num_t int_pin: GPIO number for INT pin (use GPIO_NUM_NC if not used)
                - i2c9555_input_cb_t f: Callback function when input changes
            - Return: device_id (0-7), -1 if failed
                    
        2. Use i2c9555_ioconfig(device_id, config) to configure I/O directions
            - Parameters:
                - uint8_t device_id: Device ID returned from i2c9555_add_device()
                - uint16_t config: 
                    - 1 = Input
                    - 0 = Output
                    - Example: 0x0000 = All outputs, 0xFFFF = All inputs
                    
        3. Read or write a gpio data
            - Functions:
                1. i2c9555pin_read(device_id, pin): Read a specific pin
                    - Parameters:
                        - uint8_t device_id: Device ID
                        - uint16_t pin (e.g., EXT_IO0, EXT_IO1, etc.)
                    - Return: int (1 for HIGH, 0 for LOW)
                    
                2. i2c9555pin_write(device_id, pin, level): Write to a specific pin
                    - Parameters:
                        - uint8_t device_id: Device ID
                        - uint16_t pin (e.g., EXT_IO0, EXT_IO1, etc.)
                        - int level (1 for HIGH, 0 for LOW)
                    - Return: esp_err_t
                    
        Example:
            // Add two TCA9555 devices
            int dev0 = i2c9555_add_device(GPIO_NUM_21, GPIO_NUM_22, 0x40, GPIO_NUM_NC, NULL);
            int dev1 = i2c9555_add_device(GPIO_NUM_21, GPIO_NUM_22, 0x42, GPIO_NUM_4, callback);
            
            // Configure device 0: all outputs
            i2c9555_ioconfig(dev0, 0x0000);
            
            // Configure device 1: all inputs
            i2c9555_ioconfig(dev1, 0xFFFF);
            
            // Control pins
            i2c9555pin_write(dev0, EXT_IO0, HIGH);
            int state = i2c9555pin_read(dev1, EXT_IO5);

        TCA9555 Address Sheet
        A2      A1	    A0	        DEVICE TWO-WIRE ADDRESS
        GND	    GND	    GND	        0x40 (write), 0x41 (read)
        GND	    GND	    VCC	        0x42 (write), 0x43 (read)
        GND	    VCC	    GND	        0x44 (write), 0x45 (read)
        GND	    VCC	    VCC	        0x46 (write), 0x47 (read)
        VCC	    GND	    GND	        0x48 (write), 0x49 (read)
        VCC	    GND	    VCC	        0x4A (write), 0x4B (read)
        VCC	    VCC	    GND	        0x4C (write), 0x4D (read)
        VCC	    VCC	    VCC	        0x4E (write), 0x4F (read)

        PCA9555 Address Sheet
        A2	    A1	    A0          DEVICE ADDRESS
        L	    L	    L	        0x20
        L	    L	    H	        0x21
        L	    H	    L	        0x22
        L	    H	    H	        0x23
        H	    L	    L	        0x24
        H	    L	    H	        0x25
        H	    H	    L	        0x26
        H	    H	    H	        0x27
        
    2025/11 by Shizuku
*/
#ifndef MAIN_9555_H
#define MAIN_9555_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#define EXT_IO0 (1 << 0)
#define EXT_IO1 (1 << 1)
#define EXT_IO2 (1 << 2)
#define EXT_IO3 (1 << 3)
#define EXT_IO4 (1 << 4)
#define EXT_IO5 (1 << 5)
#define EXT_IO6 (1 << 6)
#define EXT_IO7 (1 << 7)
#define EXT_IO8 NULL
#define EXT_IO9 NULL
#define EXT_IO10 (1 << 8)
#define EXT_IO11 (1 << 9)
#define EXT_IO12 (1 << 10)
#define EXT_IO13 (1 << 11)
#define EXT_IO14 (1 << 12)
#define EXT_IO15 (1 << 13)
#define EXT_IO16 (1 << 14)
#define EXT_IO17 (1 << 15)

#define HIGH 1
#define LOW 0

typedef void (*i2c9555_input_cb_t)(uint16_t pin, int level);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Add a TCA9555 device to the I2C bus
 * @param sda GPIO number for SDA
 * @param scl GPIO number for SCL
 * @param addr Device address (0x40, 0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C, 0x4E)
 * @param int_pin GPIO number for INT pin (use GPIO_NUM_NC if not used)
 * @param f Callback function when input changes
 * @return device_id (0-7), -1 if failed
 */
int i2c9555_add_device(gpio_num_t sda, gpio_num_t scl, uint16_t addr, gpio_num_t int_pin, i2c9555_input_cb_t f);

/**
 * @brief Write a 16-bit word to a device register
 * @param device_id Device ID
 * @param reg Register address
 * @param data 16-bit data to write
 * @return esp_err_t
 */
esp_err_t i2c9555_write_word(uint8_t device_id, uint8_t reg, uint16_t data);

/**
 * @brief Read a 16-bit word from a device register
 * @param device_id Device ID
 * @param reg Register address
 * @param data Pointer to store the read data
 * @return esp_err_t
 */
esp_err_t i2c9555_read_word(uint8_t device_id, uint8_t reg, uint16_t *data);

/**
 * @brief Configure I/O directions for a device
 * @param device_id Device ID
 * @param config 16-bit configuration (1 = Input, 0 = Output)
 * @return esp_err_t
 */
esp_err_t i2c9555_ioconfig(uint8_t device_id, uint16_t config);

/**
 * @brief Read a specific pin from a device
 * @param device_id Device ID
 * @param pin Pin mask (e.g., EXT_IO0, EXT_IO1, etc.)
 * @return int (1 for HIGH, 0 for LOW)
 */
int i2c9555pin_read(uint8_t device_id, uint16_t pin);

/**
 * @brief Write to a specific pin of a device
 * @param device_id Device ID
 * @param pin Pin mask (e.g., EXT_IO0, EXT_IO1, etc.)
 * @param level 1 for HIGH, 0 for LOW
 * @return esp_err_t
 */
esp_err_t i2c9555pin_write(uint8_t device_id, uint16_t pin, int level);

#ifdef __cplusplus
}
#endif

#endif // MAIN_9555_H
