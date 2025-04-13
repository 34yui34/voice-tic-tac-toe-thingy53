/**
* SSD1327 OLED Display Driver for Nordic Thingy:53
* 
* This example demonstrates how to initialize and drive an SSD1327 OLED display
* with the Thingy:53 board using Zephyr RTOS.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "ssd1327_oled.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* OLED Display buffer */
static uint8_t s_oled_display[(DISPLAY_HEIGHT * DISPLAY_WIDTH / 2)] = {0};

int draw_test_pattern(void) {
    int ret;
    
    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(0, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Create grayscale gradient pattern */
    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        for (int j = 0; j < DISPLAY_WIDTH / 2; j++) {
            uint8_t gray_value = ((i * 16) / DISPLAY_HEIGHT) & 0x0F;
            s_oled_display[i * (DISPLAY_WIDTH / 2) + j] = (gray_value << 4) | gray_value;
        }
    }
    
    /* Write pattern to display */
    ret = ssd1327_write_data(s_oled_display, sizeof(s_oled_display));
    
    return ret;
}

int main(void) {
    /* Initialize display */
    int ret = ssd1327_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize display: %d", ret);
        return -1;
    }
    
    /* Clear the display */
    ret = ssd1327_clear();
    if (ret < 0) {
        LOG_ERR("Failed to clear display: %d", ret);
        return -1;
    }
    
    k_sleep(K_MSEC(1000));
    
    /* Draw test pattern */
    LOG_INF("Drawing test pattern");
    ret = draw_test_pattern();
    if (ret < 0) {
        LOG_ERR("Failed to draw test pattern: %d", ret);
        return -1;
    }
    LOG_INF("Test pattern drawn successfully");
    
    /* Main loop */
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}