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
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "table_image.h"
#include "circle_image.h"
#include "black_square_image.h"
#include "cross_image.h"
#include "cpu_win_text_image.h"
#include "you_win_text_image.h"
#include "ssd1327_oled.h"
#include "dmic_voice_recognition.h"

#include <stdlib.h>

/* Button definitions for Thingy:53 - use the appropriate GPIO pins */
#define BUTTON1_NODE DT_ALIAS(sw0)
/* The devicetree node identifier for the "led0" alias. */
#define RED_LED_NODE DT_ALIAS(led0)
#define GREEN_LED_NODE DT_ALIAS(led1)
#define BLUE_LED_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(RED_LED_NODE, gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(GREEN_LED_NODE, gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(BLUE_LED_NODE, gpios);
static struct gpio_callback button1_cb_data;

/* Create a semaphore to signal button presses */
K_SEM_DEFINE(button_sem, 0, 1);

typedef enum {
    IDLE,
    SAMPLING,
    SAMPLE_OK,
    SAMPLE_NOK,
    CONFIRMATION,
    RESET,
    DONE,
} state_t;

/* Tic-tac-toe game variables */
typedef enum {
    NO_MARK = 0,
    PLAYER_MARK = 1,
    CPU_MARK = 2
} board_marker_t;

typedef enum {
    ONGOING = 0,
    PLAYER_WIN = 1,
    CPU_WIN = 2,
    DRAW = 3
} game_result_t;

static board_marker_t board_status[3][3] = {};
static state_t game_state = IDLE;

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

static const image_area_t result_text_area = { .first_x = 14, .first_y = 43, .second_x = 114, .second_y = 83 };

static game_result_t check_win_condition(void) {
    /*
        There should be total 8 win conditions:
        - 3 in horizontal rows
        - 3 in vertical columns
        - 2 in diagonal direction
    */
    int result = 0;
    if ((((result = board_status[0][0]) == board_status[0][1]) && (board_status[0][1] == board_status[0][2]) && (result != 0)) ||
        (((result = board_status[1][0]) == board_status[1][1]) && (board_status[1][1] == board_status[1][2]) && (result != 0)) ||
        (((result = board_status[2][0]) == board_status[2][1]) && (board_status[2][1] == board_status[2][2]) && (result != 0)) ||
        (((result = board_status[0][0]) == board_status[1][0]) && (board_status[1][0] == board_status[2][0]) && (result != 0)) ||
        (((result = board_status[0][1]) == board_status[1][1]) && (board_status[1][1] == board_status[2][1]) && (result != 0)) ||
        (((result = board_status[0][2]) == board_status[1][2]) && (board_status[1][2] == board_status[2][2]) && (result != 0)) ||
        (((result = board_status[0][0]) == board_status[1][1]) && (board_status[1][1] == board_status[2][2]) && (result != 0)) ||
        (((result = board_status[0][2]) == board_status[1][1]) && (board_status[1][1] == board_status[2][0]) && (result != 0)))
    {
        printk("%d %d %d\n%d %d %d\n%d %d %d\n",
                board_status[0][0],
                board_status[0][1],
                board_status[0][2],
                board_status[1][0],
                board_status[1][1],
                board_status[1][2],
                board_status[2][0],
                board_status[2][1],
                board_status[2][2]
        );
        if (result == CPU_MARK) {
            printk("CPU WIN\n");
            return CPU_WIN;
        }
        else {
            printk("PLAYER WIN\n");
            return PLAYER_WIN;
        }
    }

    /* Check draw */
    bool no_empty_cell = true;
    for (size_t col = 0; col < 3; col++) {
        for (size_t row = 0; row < 3; row++) {
            if (board_status[col][row] == NO_MARK) {
                no_empty_cell = false;
                break;
            }
        }
    }
    if (no_empty_cell) {
        printk("DRAW\n");
        return DRAW;
    }

    /* Else, it is still ongoing */
    printk("ONGOING\n");
    return ONGOING;
}

static bool is_cell_empty(uint8_t col, uint8_t row) {
    if (col >= 3 || row >= 3) {
        /* This condition should not occurred! */
        printk("Error! column or row value should not be more than or equal to 3\n");
    }

    if (board_status[col][row] != NO_MARK) {
        return false;
    }
    return true;
}

static bool is_sample_result_valid(voice_classification_label_t result) {
    bool valid = false;
    switch (result) {
        case ONE_ONE_DETECTED: {
            if (is_cell_empty(0, 0)) {
                valid = true;
            }
            break;
        }
        case ONE_TWO_DETECTED: {
            if (is_cell_empty(0, 1)) {
                valid = true;
            }
            break;
        }
        case ONE_THREE_DETECTED: {
            if (is_cell_empty(0, 2)) {
                valid = true;
            }
            break;
        }
        case TWO_ONE_DETECTED: {
            if (is_cell_empty(1, 0)) {
                valid = true;
            }
            break;
        }
        case TWO_TWO_DETECTED: {
            if (is_cell_empty(1, 1)) {
                valid = true;
            }
            break;
        }
        case TWO_THREE_DETECTED: {
            if (is_cell_empty(1, 2)) {
                valid = true;
            }
            break;
        }
        case THREE_ONE_DETECTED: {
            if (is_cell_empty(2, 0)) {
                valid = true;
            }
            break;
        }
        case THREE_TWO_DETECTED: {
            if (is_cell_empty(2, 1)) {
                valid = true;
            }
            break;
        }
        case THREE_THREE_DETECTED: {
            if (is_cell_empty(2, 2)) {
                valid = true;
            }
            break;
        }
        case RESET_DETECTED: {
            valid = true;
            break;
        }
        case NOISE_DETECTED: {
            valid = false;
            break;
        }
        case NONE_DETECTED: {
            valid = false;
            break;
        }
        default: {
            printk("Invalid classification label result!\n");
            break;
        }
    }

    return valid;
}

static image_area_t get_cell_area_from_class_label(voice_classification_label_t result) {
    image_area_t ret_val = one_one_grid_area;
    switch (result) {
        case ONE_ONE_DETECTED:
            ret_val = one_one_grid_area;
            printk("get_cell_area_from_class_label: return one_one_grid_area\n");
            break;

        case ONE_TWO_DETECTED:
            ret_val = one_two_grid_area;
            printk("get_cell_area_from_class_label: return one_two_grid_area\n");
            break;

        case ONE_THREE_DETECTED:
            ret_val = one_three_grid_area;
            printk("get_cell_area_from_class_label: return one_three_grid_area\n");
            break;

        case TWO_ONE_DETECTED:
            ret_val = two_one_grid_area;
            printk("get_cell_area_from_class_label: return two_one_grid_area\n");
            break;

        case TWO_TWO_DETECTED:
            ret_val = two_two_grid_area;
            printk("get_cell_area_from_class_label: return two_two_grid_area\n");
            break;

        case TWO_THREE_DETECTED:
            ret_val = two_three_grid_area;
            printk("get_cell_area_from_class_label: return two_three_grid_area\n");
            break;

        case THREE_ONE_DETECTED:
            ret_val = three_one_grid_area;
            printk("get_cell_area_from_class_label: return three_one_grid_area\n");
            break;

        case THREE_TWO_DETECTED:
            ret_val = three_two_grid_area;
            printk("get_cell_area_from_class_label: return three_two_grid_area\n");
            break;

        case THREE_THREE_DETECTED:
            ret_val = three_three_grid_area;
            printk("get_cell_area_from_class_label: return three_three_grid_area\n");
            break;

        default:
            printk("get_cell_area_from_class_label: default! %d\n", result);
            break;
    }

    return ret_val;
}

static image_area_t get_cell_area_from_coordinate(uint8_t col, uint8_t row) {
    image_area_t ret_val = one_one_grid_area;
    if (col == 0 && row == 0) {
        ret_val = one_one_grid_area;
    }
    else if (col == 0 && row == 1) {
        ret_val = one_two_grid_area;
    }
    else if (col == 0 && row == 2) {
        ret_val = one_three_grid_area;
    }
    else if (col == 1 && row == 0) {
        ret_val = two_one_grid_area;
    }
    else if (col == 1 && row == 1) {
        ret_val = two_two_grid_area;
    }
    else if (col == 1 && row == 2) {
        ret_val = two_three_grid_area;
    }
    else if (col == 2 && row == 0) {
        ret_val = three_one_grid_area;
    }
    else if (col == 2 && row == 1) {
        ret_val = three_two_grid_area;
    }
    else if (col == 2 && row == 2) {
        ret_val = three_three_grid_area;
    }

    return ret_val;
}

static void get_coordinate_from_class_label(voice_classification_label_t result, uint8_t * col, uint8_t * row) {
    switch (result) {
        case ONE_ONE_DETECTED:
            *col = 0;
            *row = 0;
            break;

        case ONE_TWO_DETECTED:
            *col = 0;
            *row = 1;
            break;

        case ONE_THREE_DETECTED:
            *col = 0;
            *row = 2;
            break;

        case TWO_ONE_DETECTED:
            *col = 1;
            *row = 0;
            break;

        case TWO_TWO_DETECTED:
            *col = 1;
            *row = 1;
            break;

        case TWO_THREE_DETECTED:
            *col = 1;
            *row = 2;
            break;

        case THREE_ONE_DETECTED:
            *col = 2;
            *row = 0;
            break;

        case THREE_TWO_DETECTED:
            *col = 2;
            *row = 1;
            break;

        case THREE_THREE_DETECTED:
            *col = 2;
            *row = 2;
            break;

        default:
            break;
    }
}

static void request_cpu_move(uint8_t * col, uint8_t * row) {
    srand(1);
    while (true) {
        uint8_t rand_col = rand() % 3;
        uint8_t rand_row = rand() % 3;
        if (board_status[rand_col][rand_row] == NO_MARK) {
            *col = rand_col;
            *row = rand_row;
            break;
        }
    }
}

/* Button interrupt handler */
void button_pressed_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* Signal the main thread that a button was pressed */
    k_sem_give(&button_sem);
}

int main(void)
{
    /* Get button device pointers */
    const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);
    
    int ret;

    /* Config red_led */
    if (!gpio_is_ready_dt(&red_led)) {
        return 0;
    }
    ret = gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    gpio_pin_set_dt(&red_led, 0);

    /* Config green_led */
    if (!gpio_is_ready_dt(&green_led)) {
        return 0;
    }
    ret = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    gpio_pin_set_dt(&green_led, 0);

    /* Config blue_led */
    if (!gpio_is_ready_dt(&blue_led)) {
        return 0;
    }
    ret = gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    gpio_pin_set_dt(&blue_led, 0);

    /* Check if devices are ready */
    if (!device_is_ready(button1.port)) {
        printk("Button1 device not ready\n");
        return -1;
    }

    /* Configure buttons as inputs with pull-up and interrupt on falling edge */
    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0) {
        printk("Error configuring button 1: %d\n", ret);
        return -1;
    }

    /* Configure interrupts */
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error configuring button 1 interrupt: %d\n", ret);
        return -1;
    }

    /* Initialize callbacks */
    gpio_init_callback(&button1_cb_data, button_pressed_callback, BIT(button1.pin));

    /* Add callbacks */
    gpio_add_callback(button1.port, &button1_cb_data);

    /* Init dmic & voice recognition module */
    ret = dmic_voice_recognition_init();
    if (ret != 0) {
        printk("Error init DMIC\n");
        return -1;
    }

    /* Initialize display */
    ret = ssd1327_init();
    if (ret < 0) {
        printk("Failed to initialize display: %d", ret);
        return -1;
    }
    
    /* Clear the display */
    ret = ssd1327_clear();
    if (ret < 0) {
        printk("Failed to clear display: %d", ret);
        return -1;
    }

    /* Set cursor to (0,0) */
    ret = ssd1327_set_cursor(0, 0);
    if (ret < 0) {
        return ret;
    }
    /* Write the table image */
    ret = ssd1327_write_data(table_image, sizeof(table_image));

    /* Main loop - can do other tasks here */
    voice_classification_label_t voice_recognition_result = NONE_DETECTED;
    while (1) {
        /* Wait for a button press */
        switch (game_state) {
            case IDLE: {
                k_sem_take(&button_sem, K_FOREVER);
                game_state = SAMPLING;
                break;
            }

            case SAMPLING: {
                gpio_pin_set_dt(&blue_led, 1);
                k_sleep(K_MSEC(200));
                dmic_voice_recognition_sample_and_classify(&voice_recognition_result);
                gpio_pin_set_dt(&blue_led, 0);
                if (is_sample_result_valid(voice_recognition_result)) {
                    game_state = CONFIRMATION;
                }
                else {
                    printk("voice sample not valid\n");
                    game_state = SAMPLE_NOK;
                }
                break;
            }

            case CONFIRMATION: {
                k_sem_reset(&button_sem);
                bool break_loop = false;
                int loop_cnt = 0;
                while (!break_loop) {
                    if (voice_recognition_result == RESET_DETECTED) {
                        /* Show RESET confirmation (maybe led?) */
                        gpio_pin_set_dt(&green_led, 1);
                        if (k_sem_take(&button_sem, K_MSEC(500)) == 0) {
                            /* Button is pressed during this time */
                            game_state = RESET;
                            break_loop = true;
                            continue;
                        }
                        gpio_pin_set_dt(&green_led, 0);
                        if (k_sem_take(&button_sem, K_MSEC(500)) == 0) {
                            /* Button is pressed during this time */
                            game_state = RESET;
                            break_loop = true;
                            continue;
                        }
                        loop_cnt++;
                        if (loop_cnt == 5) {
                            game_state = SAMPLE_NOK;
                            break_loop = true;
                            continue;
                        }
                    }
                    else {
                        /* Show the cell location and display led */
                        image_area_t cell_area = get_cell_area_from_class_label(voice_recognition_result);
                        gpio_pin_set_dt(&green_led, 1);
                        ssd_1327_draw_image(cross_image, cell_area);
                        if (k_sem_take(&button_sem, K_MSEC(500)) == 0) {
                            gpio_pin_set_dt(&green_led, 0);
                            break_loop = true;
                            /* Button is pressed during this time */
                            uint8_t col, row;
                            /* Mark player move in board status */
                            ssd_1327_draw_image(cross_image, cell_area);
                            get_coordinate_from_class_label(voice_recognition_result, &col, &row);
                            board_status[col][row] = PLAYER_MARK;
                            /* Check if win condition is reached */
                            if (check_win_condition() == PLAYER_WIN || check_win_condition() == DRAW) {
                                game_state = DONE;
                                continue;
                            }
                            /* Get CPU to make a move and mark it */
                            request_cpu_move(&col, &row);
                            board_status[col][row] = CPU_MARK;
                            cell_area = get_cell_area_from_coordinate(col, row);
                            ssd_1327_draw_image(circle_image, cell_area);
                            if (check_win_condition() == CPU_WIN || check_win_condition() == DRAW) {
                                game_state = DONE;
                                continue;
                            }
                            game_state = IDLE;
                            k_sem_reset(&button_sem);
                            continue;
                        }
                        gpio_pin_set_dt(&green_led, 0);
                        ssd_1327_draw_image(black_square_image, cell_area);
                        if (k_sem_take(&button_sem, K_MSEC(500)) == 0) {
                            gpio_pin_set_dt(&green_led, 0);
                            break_loop = true;
                            /* Button is pressed during this time */
                            uint8_t col, row;
                            /* Mark player move in board status */
                            ssd_1327_draw_image(cross_image, cell_area);
                            get_coordinate_from_class_label(voice_recognition_result, &col, &row);
                            board_status[col][row] = PLAYER_MARK;
                            /* Check if win condition is reached */
                            if (check_win_condition() == PLAYER_WIN || check_win_condition() == DRAW) {
                                game_state = DONE;
                                continue;
                            }
                            /* Get CPU to make a move and mark it */
                            request_cpu_move(&col, &row);
                            board_status[col][row] = CPU_MARK;
                            cell_area = get_cell_area_from_coordinate(col, row);
                            ssd_1327_draw_image(circle_image, cell_area);
                            if (check_win_condition() == CPU_WIN || check_win_condition() == DRAW) {
                                game_state = DONE;
                                continue;
                            }
                            game_state = IDLE;
                            k_sem_reset(&button_sem);
                            continue;
                        }
                        loop_cnt++;
                        if (loop_cnt == 5) {
                            game_state = IDLE;
                            break_loop = true;
                            k_sem_reset(&button_sem);
                            continue;
                        }
                    }
                }
                break;
            }

            case RESET: {
                printk("Enter state RESET\n");
                ssd1327_set_cursor(0, 0);
                ssd1327_write_data(table_image, sizeof(table_image));
                memset(board_status, 0, sizeof(board_status));
                game_state = IDLE;
                k_sem_reset(&button_sem);
                break;
            }

            case DONE: {
                printk("Enter state DONE\n");
                k_sem_reset(&button_sem);
                game_result_t result = check_win_condition();
                if (result == CPU_WIN) {
                    ssd_1327_draw_image(cpu_win_text_image, result_text_area);
                }
                else if (result == PLAYER_WIN) {
                    ssd_1327_draw_image(you_win_text_image, result_text_area);
                }
                k_sem_take(&button_sem, K_FOREVER);
                ssd1327_set_cursor(0, 0);
                ssd1327_write_data(table_image, sizeof(table_image));
                memset(board_status, 0, sizeof(board_status));
                game_state = IDLE;
                k_sem_reset(&button_sem);
                break;
            }

            case SAMPLE_NOK: {
                for (size_t i = 0; i < 2; i++) {
                    gpio_pin_set_dt(&red_led, 1);
                    k_msleep(500);
                    gpio_pin_set_dt(&red_led, 0);
                    k_msleep(500);
                }
                k_sem_reset(&button_sem);
                game_state = IDLE;
            }

            default: {
                break;
            }
        }
    }
}