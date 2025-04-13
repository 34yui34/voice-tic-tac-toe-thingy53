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
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>

#include "table_image.h"
#include "square_image.h"
#include "black_square_image.h"
#include "cross_image.h"
#include "ssd1327_oled.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* OLED Display buffer */
static uint8_t s_oled_display[(DISPLAY_HEIGHT * DISPLAY_WIDTH / 2)] = {0};

/* Coordinates for all 9 grid in the table */
static const image_area_t one_one_grid_area     = { .first_x = 6,  .first_y = 6,  .second_x = 35,  .second_y = 35  };
static const image_area_t one_two_grid_area     = { .first_x = 50, .first_y = 6,  .second_x = 79,  .second_y = 35  };
static const image_area_t one_three_grid_area   = { .first_x = 94, .first_y = 6,  .second_x = 123, .second_y = 35  };
static const image_area_t two_one_grid_area     = { .first_x = 6,  .first_y = 50, .second_x = 35,  .second_y = 79  };
static const image_area_t two_two_grid_area     = { .first_x = 50, .first_y = 50, .second_x = 79,  .second_y = 79  };
static const image_area_t two_three_grid_area   = { .first_x = 94, .first_y = 50, .second_x = 123, .second_y = 79  };
static const image_area_t three_one_grid_area   = { .first_x = 6,  .first_y = 94, .second_x = 35,  .second_y = 123 };
static const image_area_t three_two_grid_area   = { .first_x = 50, .first_y = 94, .second_x = 79,  .second_y = 123 };
static const image_area_t three_three_grid_area = { .first_x = 94, .first_y = 94, .second_x = 123, .second_y = 123 };

int draw_test_pattern(void) {
    int ret;
    
    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(0, 0);
    if (ret < 0) {
        return ret;
    }
    /* Write the table image */
    ret = ssd1327_write_data(table_image, sizeof(table_image));

    /* Fill out all the grids */
    ret = ssd_1327_draw_image(square_image, one_one_grid_area);
    ret = ssd_1327_draw_image(square_image, one_two_grid_area);
    ret = ssd_1327_draw_image(square_image, one_three_grid_area);
    ret = ssd_1327_draw_image(square_image, two_one_grid_area);
    ret = ssd_1327_draw_image(square_image, two_two_grid_area);
    ret = ssd_1327_draw_image(square_image, two_three_grid_area);
    ret = ssd_1327_draw_image(square_image, three_one_grid_area);
    ret = ssd_1327_draw_image(square_image, three_two_grid_area);
    ret = ssd_1327_draw_image(square_image, three_three_grid_area);
    if (ret != 0) {
        LOG_ERR("draw image error!");
    }
    
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
        LOG_INF("Time passed 1 second");
        log_thread_trigger();
        k_sleep(K_SECONDS(1));
    }

    return 0;
}