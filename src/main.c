/**
* SSD1327 OLED Display Driver for Nordic Thingy:53
* 
* This example demonstrates how to initialize and drive an SSD1327 OLED display
* with the Thingy:53 board using Zephyr RTOS.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ssd1327_demo, LOG_LEVEL_INF);

/* SSD1327 I2C address */
#define SSD1327_I2C_ADDR 0x3D

/* Display dimensions */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128

/* SSD1327 Commands */
#define SSD1327_CMD_SET_COLUMN_ADDRESS 0x15
#define SSD1327_CMD_SET_ROW_ADDRESS 0x75
#define SSD1327_CMD_SET_CONTRAST 0x81
#define SSD1327_CMD_SET_REMAP 0xA0
#define SSD1327_CMD_SET_DISPLAY_START_LINE 0xA1
#define SSD1327_CMD_SET_DISPLAY_OFFSET 0xA2
#define SSD1327_CMD_SET_DISPLAY_MODE_NORMAL 0xA4
#define SSD1327_CMD_SET_DISPLAY_MODE_ALL_ON 0xA5
#define SSD1327_CMD_SET_DISPLAY_MODE_ALL_OFF 0xA6
#define SSD1327_CMD_SET_DISPLAY_MODE_INVERSE 0xA7
#define SSD1327_CMD_SET_MULTIPLEX_RATIO 0xA8
#define SSD1327_CMD_FUNCTION_SELECTION_A 0xAB
#define SSD1327_CMD_SET_DISPLAY_ON 0xAF
#define SSD1327_CMD_SET_DISPLAY_OFF 0xAE
#define SSD1327_CMD_SET_PHASE_LENGTH 0xB1
#define SSD1327_CMD_SET_CLOCK_DIVIDER 0xB3
#define SSD1327_CMD_SET_SECOND_PRECHARGE 0xB6
#define SSD1327_CMD_SET_PRECHARGE_VOLTAGE 0xBC
#define SSD1327_CMD_SET_VCOMH 0xBE
#define SSD1327_CMD_FUNCTION_SELECTION_B 0xD5
#define SSD1327_CMD_SET_COMMAND_LOCK 0xFD

/* Control byte definitions */
#define SSD1327_CONTROL_BYTE_CMD 0x00
#define SSD1327_CONTROL_BYTE_DATA 0x40

/* Buffer for display data */
static uint8_t display_buffer[DISPLAY_HEIGHT * DISPLAY_WIDTH / 2];

/**
* @brief Write command to SSD1327
* 
* @param dev I2C device
* @param cmd Command byte
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_write_cmd(const struct device *dev, uint8_t cmd)
{
    uint8_t buf[2] = {SSD1327_CONTROL_BYTE_CMD, cmd};
    return i2c_write(dev, buf, sizeof(buf), SSD1327_I2C_ADDR);
}

/**
* @brief Write data to SSD1327
* 
* @param dev I2C device
* @param data Data buffer
* @param len Data length
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_write_data(const struct device *dev, const uint8_t *data, size_t len)
{
    uint8_t *buf = k_malloc(len + 1);
    int ret;
    
    if (!buf) {
        return -ENOMEM;
    }
    
    buf[0] = SSD1327_CONTROL_BYTE_DATA;
    memcpy(&buf[1], data, len);
    
    ret = i2c_write(dev, buf, len + 1, SSD1327_I2C_ADDR);
    k_free(buf);
    
    return ret;
}

/**
* @brief Initialize SSD1327 display
* 
* @param i2c_dev I2C device
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_init(const struct device *i2c_dev)
{
    int ret;
    
    /* Display off */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_DISPLAY_OFF);
    if (ret < 0) {
        LOG_ERR("Failed to turn display off");
        return ret;
    }
    
    /* Set clock divider */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_CLOCK_DIVIDER);
    ret |= ssd1327_write_cmd(i2c_dev, 0x51); /* Default value */
    if (ret < 0) {
        return ret;
    }
    
    /* Set multiplex ratio */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_MULTIPLEX_RATIO);
    ret |= ssd1327_write_cmd(i2c_dev, 0x7F); /* 1/128 duty */
    if (ret < 0) {
        return ret;
    }
    
    /* Set display offset */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_DISPLAY_OFFSET);
    ret |= ssd1327_write_cmd(i2c_dev, 0x00);
    if (ret < 0) {
        return ret;
    }
    
    /* Set start line */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_DISPLAY_START_LINE);
    ret |= ssd1327_write_cmd(i2c_dev, 0x00);
    if (ret < 0) {
        return ret;
    }
    
    /* Set remap configuration */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_REMAP);
    ret |= ssd1327_write_cmd(i2c_dev, 0x51); /* Enable column address remapping, COM remapping */
    if (ret < 0) {
        return ret;
    }
    
    /* Set phase length */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_PHASE_LENGTH);
    ret |= ssd1327_write_cmd(i2c_dev, 0x55); /* Phase 1: 5 DCLKs, Phase 2: 5 DCLKs */
    if (ret < 0) {
        return ret;
    }
    
    /* Set contrast */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_CONTRAST);
    ret |= ssd1327_write_cmd(i2c_dev, 0x7F); /* Default */
    if (ret < 0) {
        return ret;
    }
    
    /* Normal display mode */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_DISPLAY_MODE_NORMAL);
    if (ret < 0) {
        return ret;
    }
    
    /* Turn display on */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_DISPLAY_ON);
    if (ret < 0) {
        LOG_ERR("Failed to turn display on");
        return ret;
    }
    
    LOG_INF("SSD1327 display initialized");
    return 0;
}

/**
* @brief Set cursor position
* 
* @param i2c_dev I2C device
* @param column Column address
* @param row Row address
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_set_cursor(const struct device *i2c_dev, uint8_t column, uint8_t row)
{
    int ret;
    
    /* Set column address */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_COLUMN_ADDRESS);
    ret |= ssd1327_write_cmd(i2c_dev, column);
    ret |= ssd1327_write_cmd(i2c_dev, DISPLAY_WIDTH / 2 - 1);
    if (ret < 0) {
        return ret;
    }
    
    /* Set row address */
    ret = ssd1327_write_cmd(i2c_dev, SSD1327_CMD_SET_ROW_ADDRESS);
    ret |= ssd1327_write_cmd(i2c_dev, row);
    ret |= ssd1327_write_cmd(i2c_dev, DISPLAY_HEIGHT - 1);
    
    return ret;
}

/**
* @brief Clear the display
* 
* @param i2c_dev I2C device
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_clear(const struct device *i2c_dev)
{
    int ret;
    
    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(i2c_dev, 0, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Fill with zeros */
    memset(display_buffer, 0, sizeof(display_buffer));
    ret = ssd1327_write_data(i2c_dev, display_buffer, sizeof(display_buffer));
    
    return ret;
}

/**
* @brief Draw a test pattern on the display
* 
* @param i2c_dev I2C device
* @return int 0 on success, negative errno on failure
*/
static int ssd1327_draw_test_pattern(const struct device *i2c_dev)
{
    int ret;
    
    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(i2c_dev, 0, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Create grayscale gradient pattern */
    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        for (int j = 0; j < DISPLAY_WIDTH / 2; j++) {
            uint8_t gray_value = ((i * 16) / DISPLAY_HEIGHT) & 0x0F;
            display_buffer[i * (DISPLAY_WIDTH / 2) + j] = (gray_value << 4) | gray_value;
        }
    }
    
    /* Write pattern to display */
    ret = ssd1327_write_data(i2c_dev, display_buffer, sizeof(display_buffer));
    
    return ret;
}

void i2c_scan(const struct device *dev)
{
uint8_t error, addr;
    LOG_INF("I2C Scanning...");

    for (addr = 0; addr <= 0x7F; addr++) {
        error = i2c_write(dev, NULL, 0, addr);
        if (error == 0) {
            LOG_INF("I2C device found at address 0x%02X", addr);
        }
    }

    LOG_INF("I2C scan complete");
}

int main(void)
{
    const struct device *i2c_dev;
    
    /* Get I2C device - on Thingy:53, the I2C is on the 'I2C_1' bus */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    // i2c_dev = device_get_binding(DT_NODELABEL(ssd1327));
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -1;
    }
    
    LOG_INF("Initializing SSD1327 display on I2C bus: %s", i2c_dev->name);

    // i2c_scan(i2c_dev);
    
#if 1
    /* Initialize display */
    int ret = ssd1327_init(i2c_dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize display: %d", ret);
        return -1;
    }
    
    /* Clear the display */
    ret = ssd1327_clear(i2c_dev);
    if (ret < 0) {
        LOG_ERR("Failed to clear display: %d", ret);
        return -1;
    }
    
    k_sleep(K_MSEC(1000));
    
    /* Draw test pattern */
    LOG_INF("Drawing test pattern");
    ret = ssd1327_draw_test_pattern(i2c_dev);
    if (ret < 0) {
        LOG_ERR("Failed to draw test pattern: %d", ret);
        return -1;
    }
    
    LOG_INF("Test pattern drawn successfully");
#endif
    /* Main loop */
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}