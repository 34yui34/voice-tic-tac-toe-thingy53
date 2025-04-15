/*
 * Copyright (c) 2025 34yui34
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "ssd1327_oled.h"

LOG_MODULE_REGISTER(ssd1327_oled, LOG_LEVEL_INF);

static const struct device * s_ssd1327_oled_i2c = NULL;
/* Buffer for display data */
static uint8_t s_display_buffer[(DISPLAY_HEIGHT * DISPLAY_WIDTH / 2)] = {0};

int ssd1327_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {SSD1327_CONTROL_BYTE_CMD, cmd};
    return i2c_write(s_ssd1327_oled_i2c, buf, sizeof(buf), SSD1327_I2C_ADDR);
}

int ssd1327_write_data(const uint8_t *data, size_t len) {
    uint8_t *buf = (uint8_t *)k_malloc(len + 1);
    int ret;
    
    if (!buf) {
        return -ENOMEM;
    }
    
    buf[0] = SSD1327_CONTROL_BYTE_DATA;
    memcpy(&buf[1], data, len);
    
    ret = i2c_write(s_ssd1327_oled_i2c, buf, len + 1, SSD1327_I2C_ADDR);
    k_free(buf);
    
    return ret;
}

int ssd1327_set_cursor(uint8_t column, uint8_t row) {
    int ret;
    
    /* Set column address */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_COLUMN_ADDRESS);
    ret |= ssd1327_write_cmd(column / 2);
    ret |= ssd1327_write_cmd(DISPLAY_WIDTH / 2 - 1);
    if (ret < 0) {
        return ret;
    }
    
    /* Set row address */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_ROW_ADDRESS);
    ret |= ssd1327_write_cmd(row);
    ret |= ssd1327_write_cmd(DISPLAY_HEIGHT - 1);
    
    return ret;
}

int ssd1327_clear(void) {
    int ret;
    
    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(0, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Fill with zeros */
    memset(s_display_buffer, 0, sizeof(s_display_buffer));
    ret = ssd1327_write_data(s_display_buffer, sizeof(s_display_buffer) - 1);
    
    return ret;
}

int ssd_1327_draw_image(const uint8_t *data, image_area_t area) {
    int ret = 0;
    
    if (area.first_x >= DISPLAY_WIDTH ||
        area.second_x >= DISPLAY_WIDTH ||
        area.first_y >= DISPLAY_HEIGHT ||
        area.second_y >= DISPLAY_HEIGHT)
    {
        LOG_ERR("Image area must not exceed display size");
        return -1;
    }
    if (area.first_x >= area.second_x || area.first_y >= area.second_y) {
        LOG_ERR("Expected that first point must be less than second point in the draw area");
        return -1;
    }

    for (size_t i = 0; i < (area.second_y - area.first_y + 1); i++) {
        ret = ssd1327_set_cursor(area.first_x, area.first_y + i);
        if (ret < 0) {
            return ret;
        }
        ret = ssd1327_write_data(&data[i * ((area.second_x - area.first_x + 1) / 2)], (area.second_x - area.first_x + 1) / 2);
    }

    return ret;
}

int ssd1327_init(void) {
    int ret;
        
    /* Get I2C device - The SSD1327 OLED is connected to I2C0 */
    s_ssd1327_oled_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    
    if (!device_is_ready(s_ssd1327_oled_i2c)) {
        LOG_ERR("SSD1327 OLED device not ready");
        return -1;
    }
    
    LOG_INF("Initializing SSD1327 display on I2C bus: %s", s_ssd1327_oled_i2c->name);

    /* Display off */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_DISPLAY_OFF);
    if (ret < 0) {
        LOG_ERR("Failed to turn display off");
        return ret;
    }
    
    /* Set clock divider */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_CLOCK_DIVIDER);
    ret |= ssd1327_write_cmd(0x51); /* Default value */
    if (ret < 0) {
        return ret;
    }
    
    /* Set multiplex ratio */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_MULTIPLEX_RATIO);
    ret |= ssd1327_write_cmd(0x7F); /* 1/128 duty */
    if (ret < 0) {
        return ret;
    }
    
    /* Set display offset */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_DISPLAY_OFFSET);
    ret |= ssd1327_write_cmd(0x00);
    if (ret < 0) {
        return ret;
    }
    
    /* Set start line */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_DISPLAY_START_LINE);
    ret |= ssd1327_write_cmd(0x00);
    if (ret < 0) {
        return ret;
    }
    
    /* Set remap configuration */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_REMAP);
    ret |= ssd1327_write_cmd(0x51); /* Enable column address remapping, COM remapping */
    if (ret < 0) {
        return ret;
    }
    
    /* Set phase length */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_PHASE_LENGTH);
    ret |= ssd1327_write_cmd(0x55); /* Phase 1: 5 DCLKs, Phase 2: 5 DCLKs */
    if (ret < 0) {
        return ret;
    }
    
    /* Set contrast */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_CONTRAST);
    ret |= ssd1327_write_cmd(0x7F); /* Default */
    if (ret < 0) {
        return ret;
    }
    
    /* Normal display mode */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_DISPLAY_MODE_NORMAL);
    if (ret < 0) {
        return ret;
    }
    
    /* Turn display on */
    ret = ssd1327_write_cmd(SSD1327_CMD_SET_DISPLAY_ON);
    if (ret < 0) {
        LOG_ERR("Failed to turn display on");
        return ret;
    }
    
    LOG_INF("SSD1327 display initialized");
    return 0;
}