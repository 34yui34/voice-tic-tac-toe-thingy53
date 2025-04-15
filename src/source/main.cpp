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

#if 0
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

/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(dmic_sample);

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     3000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
    (BYTES_PER_SAMPLE * (_sample_rate * 2 / 1) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
* Application, after getting a given block from the driver and processing its
* data, needs to free that block.
*/
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      2
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 1);

/* Function to print sample histogram for debugging */
static void print_sample_histogram(const int16_t *samples, size_t sample_count)
{
    /* Simple 10-bin histogram */
    int bins[10] = {0};
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    
    /* Find range */
    for (size_t i = 0; i < sample_count; i++) {
        if (samples[i] < min_val) min_val = samples[i];
        if (samples[i] > max_val) max_val = samples[i];
    }
    
    /* Allow for small values */
    if (max_val - min_val < 100) {
        min_val = -50;
        max_val = 50;
    }
    
    /* Create histogram */
    double bin_size = (max_val - min_val) / 10.0;
    
    if (bin_size > 0) {
        for (size_t i = 0; i < sample_count; i++) {
            int bin = (int)((samples[i] - min_val) / bin_size);
            if (bin >= 0 && bin < 10) {
                bins[bin]++;
            }
        }
    }
    
    /* Print results */
    LOG_INF("Sample distribution:");
    for (int i = 0; i < 10; i++) {
        LOG_INF("Bin %d (%d to %d): %d samples", 
                i, 
                (int)(min_val + i * bin_size),
                (int)(min_val + (i+1) * bin_size),
                bins[i]);
    }
}

static float calculate_rms(const int16_t *samples, size_t sample_count)
{
    double sum_squares = 0.0;
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    
    /* Check for signal range */
    for (size_t i = 0; i < sample_count; i++) {
        if (samples[i] < min_val) min_val = samples[i];
        if (samples[i] > max_val) max_val = samples[i];
    }
    
    LOG_INF("Signal range - Min: %d, Max: %d, Range: %d", 
            min_val, max_val, max_val - min_val);
    
    /* Calculate RMS */
    for (size_t i = 0; i < sample_count; i++) {
        double sample = (double)samples[i] / 32768.0;
        sum_squares += sample * sample;
    }
    
    return (float)sqrt(sum_squares / sample_count);
}

static int do_pdm_transfer(const struct device *dmic_dev,
            struct dmic_cfg *cfg,
            size_t block_count)
{
    int ret;

    // LOG_INF("PCM output rate: %u, channels: %u",
    //     cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

    ret = dmic_configure(dmic_dev, cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure the driver: %d", ret);
        return ret;
    }

    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("START trigger failed: %d", ret);
        return ret;
    }

    for (int i = 0; i < block_count; ++i) {
        void *buffer;
        uint32_t size;

        ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
        if (ret < 0) {
            LOG_ERR("%d - read failed: %d", i, ret);
            return ret;
        }
        // calculate_rms(buffer, size / sizeof(int16_t));

        LOG_INF("%d - got buffer %p of %u bytes, RMS %.4f", i, buffer, size, calculate_rms(buffer, size / sizeof(int16_t)));

        k_mem_slab_free(&mem_slab, buffer);
    }

    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    if (ret < 0) {
        LOG_ERR("STOP trigger failed: %d", ret);
        return ret;
    }

    return ret;
}

int main(void)
{
    const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
    int ret;

    LOG_INF("DMIC sample");

    if (!device_is_ready(dmic_dev)) {
        LOG_ERR("%s is not ready", dmic_dev->name);
        return 0;
    }

    struct pcm_stream_cfg stream = {
        .pcm_rate = MAX_SAMPLE_RATE,
        .pcm_width = SAMPLE_BIT_WIDTH,
        .mem_slab  = &mem_slab,
    };
    struct dmic_cfg cfg = {
        .io = {
            /* These fields can be used to limit the PDM clock
            * configurations that the driver is allowed to use
            * to those supported by the microphone.
            */
            .min_pdm_clk_freq = 1000000,
            .max_pdm_clk_freq = 1500000,
            .min_pdm_clk_dc   = 40,
            .max_pdm_clk_dc   = 60,
        },
        .streams = &stream,
        .channel = {
            .req_num_streams = 1,
        },
    };

    cfg.channel.req_num_chan = 1;
    cfg.channel.req_chan_map_lo =
        dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
    cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
    cfg.streams[0].block_size =
        BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

    while (true) {        
        ret = do_pdm_transfer(dmic_dev, &cfg, 1);
        if (ret < 0) {
            return 0;
        }
    }

    return 0;
}
#endif

#if 0
// Zephyr 3.1.x and newer uses different include scheme
#include <version.h>
#if (KERNEL_VERSION_MAJOR > 3) || ((KERNEL_VERSION_MAJOR == 3) && (KERNEL_VERSION_MINOR >= 1))
#include <zephyr/kernel.h>
#else
#include <zephyr.h>
#endif
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
// #ifdef EI_NORDIC
#include <nrfx_clock.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
// #endif

LOG_MODULE_REGISTER(dmic_sample);

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
    (BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
* Application, after getting a given block from the driver and processing its
* data, needs to free that block.
*/
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 1);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static float features[16000] = {
    // copy raw features here (for example from the 'Live classification' page)
    // see https://docs.edgeimpulse.com/docs/running-your-impulse-locally-zephyr
};
static uint64_t current_window = 0;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

int main() {
    // This is needed so that output of printf is output immediately without buffering
    setvbuf(stdout, NULL, _IONBF, 0);

    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

    LOG_INF("DMIC sample");

    if (!device_is_ready(dmic_dev)) {
        LOG_ERR("%s is not ready", dmic_dev->name);
        return 0;
    }

    struct pcm_stream_cfg stream = {
        .pcm_rate = MAX_SAMPLE_RATE,
        .pcm_width = SAMPLE_BIT_WIDTH,
        .mem_slab  = &mem_slab,
    };
    struct dmic_cfg cfg = {
        .io = {
            /* These fields can be used to limit the PDM clock
            * configurations that the driver is allowed to use
            * to those supported by the microphone.
            */
            .min_pdm_clk_freq = 1000000,
            .max_pdm_clk_freq = 1500000,
            .min_pdm_clk_dc   = 40,
            .max_pdm_clk_dc   = 60,
        },
        .streams = &stream,
        .channel = {
            .req_num_streams = 1,
        },
    };

    cfg.channel.req_num_chan = 1;
    cfg.channel.req_chan_map_lo =
        dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
    cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
    cfg.streams[0].block_size =
        BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

#ifdef CONFIG_SOC_NRF5340_CPUAPP // this comes from Zephyr
    // Switch CPU core clock to 128 MHz
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
#endif

    printk("Edge Impulse standalone inferencing (Zephyr)\n");

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        return 1;
    }

    ei_impulse_result_t result = { 0 };

    while (1) {
        // gpio_pin_set_dt(&led, 1);

        ret = dmic_configure(dmic_dev, &cfg);
        if (ret < 0) {
            LOG_ERR("Failed to configure the driver: %d", ret);
            return ret;
        }

        ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
        if (ret < 0) {
            LOG_ERR("START trigger failed: %d", ret);
            return ret;
        }

        gpio_pin_set_dt(&led, 1);
        while (current_window < 10) {   
            void *buffer;
            uint32_t size;

            ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
            if (ret < 0) {
                LOG_ERR("- read failed: %d", ret);
                return ret;
            }
            // calculate_rms(buffer, size / sizeof(int16_t));

            LOG_INF("- got buffer %p of %u bytes", buffer, size);
            int16_t * i16_buf = (int16_t *)buffer;
            if (current_window < 10) {
                for (size_t i = 0; i < (size/2); i++) {
                    features[(current_window * size/2) + i] = i16_buf[i];
                }
            }
            else {
                memmove(&features[0], &features[size/2], 9 * (size/2) * sizeof(float));
                for (size_t i = 0; i < (size/2); i++) {
                    features[(9 * size/2) + i] = i16_buf[i];
                }
            }

            k_mem_slab_free(&mem_slab, buffer);
            current_window++;
        }
        gpio_pin_set_dt(&led, 0);

        ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
        if (ret < 0) {
            LOG_ERR("STOP trigger failed: %d", ret);
            return ret;
        }

        current_window = 0;

        // gpio_pin_set_dt(&led, 0);

            // the features are stored into flash, and we don't want to load everything into RAM
            signal_t features_signal;
            features_signal.total_length = sizeof(features) / sizeof(features[0]);
            features_signal.get_data = &raw_feature_get_data;

            // invoke the impulse
            EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
            printk("run_classifier returned: %d\n", res);

            if (res != 0) return 1;

            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                printk("    %s: %.5f\n", result.classification[ix].label,
                                        result.classification[ix].value);
            }
            // if (result.classification[0].value > 0.5F && result.classification[1].value < 0.2F && result.classification[2].value < 0.2F) {
            //     printk("%s detected at value %.5f!\n", result.classification[0].label, result.classification[0].value);
            // }
            // else if (result.classification[1].value > 0.5F && result.classification[0].value < 0.2F && result.classification[2].value < 0.2F) {
            //     printk("%s detected at value %.5f!\n", result.classification[1].label, result.classification[1].value);
            // }
            // else if (result.classification[2].value > 0.5F && result.classification[0].value < 0.2F && result.classification[1].value < 0.2F) {
            //     printk("%s detected at value %.5f!\n", result.classification[2].label, result.classification[2].value);
            // }
            // else {
            //     printk("None detected!\n");
            // }

//             printk("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
//                     result.timing.dsp, result.timing.classification, result.timing.anomaly);
// #if EI_CLASSIFIER_OBJECT_DETECTION == 1
//             bool bb_found = result.bounding_boxes[0].value > 0;
//             for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
//                 auto bb = result.bounding_boxes[ix];
//                 if (bb.value == 0) {
//                     continue;
//                 }
//                 printk("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
//             }
//             if (!bb_found) {
//                 printk("    No objects found\n");
//             }
// #else
//             for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//                 printk("    %s: %.5f\n", result.classification[ix].label,
//                                         result.classification[ix].value);
//             }
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//             printk("    anomaly score: %.3f\n", result.anomaly);
// #endif
// #endif
            k_msleep(5000);
    }
}
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "dmic_voice_recognition.h"

/* Button definitions for Thingy:53 - use the appropriate GPIO pins */
#define BUTTON1_NODE DT_ALIAS(sw0)
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static struct gpio_callback button1_cb_data;

/* Create a semaphore to signal button presses */
K_SEM_DEFINE(button_sem, 0, 1);

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

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    gpio_pin_set_dt(&led, 0);

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

    ret = dmic_voice_recognition_init();
    if (ret != 0) {
        printk("Error init DMIC\n");
        return -1;
    }

    voice_classification_label_t classification_label;
    /* Main loop - can do other tasks here */
    while (1) {
        /* Wait for a button press */
        k_sem_take(&button_sem, K_FOREVER);
        printk("Button1 pressed\n");
        k_sleep(K_MSEC(200));
        gpio_pin_set_dt(&led, 1);
        dmic_voice_recognition_sample_and_classify(&classification_label);
        gpio_pin_set_dt(&led, 0);
    }
}
