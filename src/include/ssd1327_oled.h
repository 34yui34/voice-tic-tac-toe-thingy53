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

#ifndef SSD1327_OLED_H
#define SSD1327_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

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

typedef struct {
    uint8_t first_x;
    uint8_t first_y;
    uint8_t second_x;
    uint8_t second_y;
} image_area_t;

/**
* @brief Initialize SSD1327 display
* 
* @return int 0 on success, negative errno on failure
*/
int ssd1327_init(void);

/**
* @brief Write command to SSD1327
* 
* @param cmd Command byte
* @return int 0 on success, negative errno on failure
*/
int ssd1327_write_cmd(uint8_t cmd);

/**
* @brief Write data to SSD1327
* 
* @param data Data buffer
* @param len Data length
* @return int 0 on success, negative errno on failure
*/
int ssd1327_write_data(const uint8_t *data, size_t len);

/**
* @brief Draw image on SSD1327
* 
* @param data Data buffer
* @param area Area to draw data
* @return int 0 on success, negative errno on failure
*/
int ssd_1327_draw_image(const uint8_t *data, image_area_t area);

/**
* @brief Set cursor position
* 
* @param column Column address
* @param row Row address
* @return int 0 on success, negative errno on failure
*/
int ssd1327_set_cursor(uint8_t column, uint8_t row);

/**
* @brief Clear the display
* 
* @return int 0 on success, negative errno on failure
*/
int ssd1327_clear(void);

#ifdef __cplusplus
}  /* End of extern "C" block */
#endif

#endif /* SSD1327_OLED_H */