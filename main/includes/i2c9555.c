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

#include "i2c9555.h"

#define TAG "I2C9555"
#define MAX_I2C9555_DEVICES 8

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint16_t addr;
    i2c9555_input_cb_t callback;
    gpio_num_t int_pin;
    EventGroupHandle_t event_group;
    TaskHandle_t task_handle;
} i2c9555_device_t;

static i2c_master_bus_handle_t i2c9555_bus_handle = NULL;
static i2c9555_device_t i2c9555_devices[MAX_I2C9555_DEVICES];
static uint8_t i2c9555_device_count = 0;

#define I2C9555_ISR_BIT BIT0

static void IRAM_ATTR i2c9555_int_handle(void *arg)
{
    uint8_t device_id = (uint32_t)arg;
    BaseType_t taskWake;
    xEventGroupSetBitsFromISR(i2c9555_devices[device_id].event_group, I2C9555_ISR_BIT, &taskWake);
    portYIELD_FROM_ISR(taskWake);
}

static void i2c9555_task(void *arg)
{
    uint8_t device_id = (uint32_t)arg;
    i2c9555_device_t *dev = &i2c9555_devices[device_id];
    uint16_t last_state = 0xFFFF;

    vTaskDelay(pdMS_TO_TICKS(200));

    uint16_t clear_state = 0;
    i2c9555_read_word(device_id, 0x00, &clear_state);
    ESP_LOGI(TAG, "[DEV%d] Cleared interrupt state: 0x%04X", device_id, clear_state);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t ret = i2c9555_read_word(device_id, 0x00, &last_state);
    ESP_LOGI(TAG, "[DEV%d] Initial state: 0x%04X (%s)", device_id, last_state, esp_err_to_name(ret));

    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(dev->event_group, I2C9555_ISR_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & I2C9555_ISR_BIT)
        {
            ESP_LOGI(TAG, "[DEV%d] Processing interrupt", device_id);

            uint16_t state = 0;
            vTaskDelay(pdMS_TO_TICKS(6)); 

            ret = i2c9555_read_word(device_id, 0x00, &state);
            ESP_LOGI(TAG, "[DEV%d] State: 0x%04X (%s), Last: 0x%04X", device_id, state, esp_err_to_name(ret), last_state);

            if (ret == ESP_OK && state != last_state)
            {
                for (uint16_t i = 0; i < 16; i++)
                {
                    uint8_t last_bit = (last_state >> i) & 1;
                    uint8_t current_bit = (state >> i) & 1;

                    if (current_bit != last_bit)
                    {
                        ESP_LOGI(TAG, "[DEV%d] Pin %u changed: %d -> %d", device_id, i, last_bit, current_bit);
                        if (dev->callback)
                        {
                            dev->callback(1 << i, current_bit);
                        }
                    }
                }
                last_state = state;
            }
        }
    }
}
/**
 * @brief Add a 9555 device to the bus
 * @param sda GPIO number for SDA
 * @param scl GPIO number for SCL
 * @param addr Device address (0x40, 0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C, 0x4E)
 * @param int_pin GPIO number for INT pin (use GPIO_NUM_NC if unused)
 * @param f Callback invoked when input changes
 * @return Device ID (0-7), -1 on failure
 */
int i2c9555_add_device(gpio_num_t sda, gpio_num_t scl, uint16_t addr, gpio_num_t int_pin, i2c9555_input_cb_t f)
{
    if (i2c9555_device_count >= MAX_I2C9555_DEVICES)
    {
        ESP_LOGE(TAG, "Max devices reached");
        return -1;
    }

    if (i2c9555_device_count == 0)
    {
        i2c_master_bus_config_t bus_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .glitch_ignore_cnt = 7,
            .i2c_port = 1,
            .flags = {.enable_internal_pullup = 1}
        };
        i2c_new_master_bus(&bus_config, &i2c9555_bus_handle);
    }

    uint8_t device_id = i2c9555_device_count;
    i2c9555_device_t *dev = &i2c9555_devices[device_id];

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 1000000,
    };
    i2c_master_bus_add_device(i2c9555_bus_handle, &dev_config, &dev->dev_handle);

    dev->addr = addr;
    dev->callback = f;
    dev->int_pin = int_pin;

    ESP_LOGI(TAG, "Added device %d at address 0x%02X", device_id, addr);

    if (int_pin != GPIO_NUM_NC)
    {
        dev->event_group = xEventGroupCreate();

        gpio_config_t int_cfg = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pin_bit_mask = (1ULL << int_pin),
        };
        gpio_config(&int_cfg);

        // 只需要安装一次ISR服务
        if (device_id == 0)
            gpio_install_isr_service(0);

        gpio_isr_handler_add(int_pin, i2c9555_int_handle, (void *)device_id);

        char task_name[16];
        snprintf(task_name, sizeof(task_name), "i2c9555_%d", device_id);
        xTaskCreatePinnedToCore(i2c9555_task, task_name, 4096, (void *)device_id, 3, &dev->task_handle, 1);
    }

    i2c9555_device_count++;
    return device_id;
}

/**
 * @brief Write a 16-bit word to a specific register
 * @param device_id Device ID
 * @param reg Register address
 * @param data 16-bit data to write
 * @return esp_err_t
 */
esp_err_t i2c9555_write_word(uint8_t device_id, uint8_t reg, uint16_t data)
{
    if (device_id >= i2c9555_device_count)
        return ESP_ERR_INVALID_ARG;

    uint8_t write_buf[3];
    write_buf[0] = reg;
    write_buf[1] = data & 0xFF;        // Low byte (Port 0)
    write_buf[2] = (data >> 8) & 0xFF; // High byte (Port 1)

    ESP_LOGI(TAG, "[DEV%d] Write reg 0x%02X: 0x%02X 0x%02X (data=0x%04X)",
             device_id, reg, write_buf[1], write_buf[2], data);

    return i2c_master_transmit(i2c9555_devices[device_id].dev_handle, write_buf, sizeof(write_buf), 500);
}

/**
 * @brief Read a 16-bit word from a specific register
 * @param device_id Device ID
 * @param reg Register address
 * @param data Pointer to store the read 16-bit data
 * @return esp_err_t
 */
esp_err_t i2c9555_read_word(uint8_t device_id, uint8_t reg, uint16_t *data)
{
    if (device_id >= i2c9555_device_count)
        return ESP_ERR_INVALID_ARG;

    uint8_t addr[1];
    uint8_t buf[2];
    addr[0] = reg;
    esp_err_t err = i2c_master_transmit_receive(i2c9555_devices[device_id].dev_handle, addr, 1, buf, 2, 500);
    if (err != ESP_OK)
    {
        return err;
    }

    *data = ((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8);

    ESP_LOGI(TAG, "[DEV%d] Read reg 0x%02X: 0x%02X 0x%02X (data=0x%04X)",
             device_id, reg, buf[0], buf[1], *data);

    return ESP_OK;
}

/**
 * @brief Read a specific pin
 * @param device_id Device ID
 * @param pin Pin mask (e.g., EXT_IO0, EXT_IO1, etc.)
 * @return int 1 for HIGH, 0 for LOW
 */
int i2c9555pin_read(uint8_t device_id, uint16_t pin)
{
    uint16_t input_state = 0;
    i2c9555_read_word(device_id, 0x00, &input_state);
    return (input_state & pin) ? HIGH : LOW;
}

/**
 * @brief Write to a specific pin
 * @param device_id Device ID
 * @param pin Pin mask (e.g., EXT_IO0, EXT_IO1, etc.)
 * @param level 1 for HIGH, 0 for LOW
 * @return esp_err_t
 */
esp_err_t i2c9555pin_write(uint8_t device_id, uint16_t pin, int level)
{
    uint16_t output_state = 0;
    i2c9555_read_word(device_id, 0x02, &output_state);
    if (level)
    {
        output_state |= pin;
    }
    else
    {
        output_state &= ~pin;
    }
    return i2c9555_write_word(device_id, 0x02, output_state);
}

/**
 * @brief Configure I/O directions
 * @param device_id Device ID
 * @param config 16-bit configuration (1 = Input, 0 = Output)
 * @return esp_err_t
 * @example i2c9555_ioconfig(0, 0b0011111111111111);
 */
esp_err_t i2c9555_ioconfig(uint8_t device_id, uint16_t config)
{
    if (device_id >= i2c9555_device_count)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = -1;
    do
    {
        ret = i2c9555_write_word(device_id, 0x06, config);
        if (ret != ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } while (ret != ESP_OK);

    return ret;
}