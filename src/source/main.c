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
#endif

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
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
    (BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
* Application, after getting a given block from the driver and processing its
* data, needs to free that block.
*/
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

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
        calculate_rms(buffer, size / sizeof(int16_t));

        // LOG_INF("%d - got buffer %p of %u bytes, RMS %.4f", i, buffer, size, calculate_rms(buffer, size / sizeof(int16_t)));

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
        ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
        if (ret < 0) {
            return 0;
        }
    }

    return 0;
}