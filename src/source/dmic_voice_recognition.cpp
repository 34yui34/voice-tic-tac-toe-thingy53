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

#include <cstdint>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#include "dmic_voice_recognition.h"

LOG_MODULE_REGISTER(dmic, LOG_LEVEL_INF);

#define SECOND_PER_WINDOW 1
#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     3000

/* Size of a block for 1s of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
    (BYTES_PER_SAMPLE * (_sample_rate) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
* Application, after getting a given block from the driver and processing its
* data, needs to free that block.
*/
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      3
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 1);

/* Threshold for determining successful classification */
#define CLASSIFICATION_THRESHOLD (0.65F)

/* DMIC device instance */
static const struct device * s_dmic_voice_recognition = NULL;

/* Buffer for storing raw feature (used for edge impulse processing) */
static float f32_raw_features[MAX_SAMPLE_RATE * SECOND_PER_WINDOW] = {0};

/* Edge impulse module use this function for getting raw data */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, f32_raw_features + offset, length * sizeof(float));
    return 0;
}

static voice_classification_label_t get_classification_label(const ei_impulse_result_classification_t * ei_classification_result) {
    voice_classification_label_t ret_val = NONE_DETECTED;
    float max_classification_value = 0.0F;
    const char * max_classification_label = NULL;

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (ei_classification_result[i].value >= max_classification_value) {
            max_classification_value = ei_classification_result[i].value;
            max_classification_label = ei_classification_result[i].label;
        }
    }
    if (max_classification_value > CLASSIFICATION_THRESHOLD) {
        if (strncmp(max_classification_label, "noise", strlen("noise")) == 0) {
            ret_val = NOISE_DETECTED;
            printk("classification: NOISE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "oneone", strlen("oneone")) == 0) {
            ret_val = ONE_ONE_DETECTED;
            printk("classification: ONE_ONE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "onetwo", strlen("onetwo")) == 0) {
            ret_val = ONE_TWO_DETECTED;
            printk("classification: ONE_TWO_DETECTED\n");
        } 
        else if (strncmp(max_classification_label, "onethree", strlen("onethree")) == 0) {
            ret_val = ONE_THREE_DETECTED;
            printk("classification: ONE_THREE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "twoone", strlen("twoone")) == 0) {
            ret_val = TWO_ONE_DETECTED;
            printk("classification: TWO_ONE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "twotwo", strlen("twotwo")) == 0) {
            ret_val = TWO_TWO_DETECTED;
            printk("classification: TWO_TWO_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "twothree", strlen("twothree")) == 0) {
            ret_val = TWO_THREE_DETECTED;
            printk("classification: TWO_THREE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "threeone", strlen("threeone")) == 0) {
            ret_val = THREE_ONE_DETECTED;
            printk("classification: THREE_ONE_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "threetwo", strlen("threetwo")) == 0) {
            ret_val = THREE_TWO_DETECTED;
            printk("classification: THREE_TWO_DETECTED\n");
        }
        else if (strncmp(max_classification_label, "threethree", strlen("threethree")) == 0) {
            ret_val = THREE_THREE_DETECTED;
            printk("classification: THREE_THREE_DETECTED\n");
        }
    }
    else {
        printk("classification: NONE_DETECTED\n");
    }
    
    /* If gets here, failed to classify */
    return ret_val;
}

int dmic_voice_recognition_init(void) {
    int ret = 0;

    s_dmic_voice_recognition = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

    if (!device_is_ready(s_dmic_voice_recognition)) {
        printk("%s is not ready\n", s_dmic_voice_recognition->name);
        return -1;
    }

    /* DMIC module configuration */
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

    ret = dmic_configure(s_dmic_voice_recognition, &cfg);
    if (ret < 0) {
        printk("Failed to configure the driver: %d\n", ret);
        return ret;
    }
    return 0;
}

int dmic_voice_recognition_sample_and_classify(voice_classification_label_t * classification_result) {
    int ret = 0;

    if (sizeof(f32_raw_features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(f32_raw_features) / sizeof(float));
        return 1;
    }

    ret = dmic_trigger(s_dmic_voice_recognition, DMIC_TRIGGER_START);
    if (ret < 0) {
        printk("START trigger failed: %d\n", ret);
        return ret;
    }

    void * buffer;
    uint32_t size;
    ret = dmic_read(s_dmic_voice_recognition, 0, &buffer, &size, READ_TIMEOUT);
    if (ret < 0) {
        printk("- read failed: %d\n", ret);
        return ret;
    }
    int16_t * i16_buffer = (int16_t *)buffer;
    for (size_t i = 0; i < size / sizeof(int16_t); i++) {
        f32_raw_features[i] = i16_buffer[i];
    }
    k_mem_slab_free(&mem_slab, buffer);

    ret = dmic_trigger(s_dmic_voice_recognition, DMIC_TRIGGER_STOP);
    if (ret < 0) {
        printk("STOP trigger failed: %d\n", ret);
        return ret;
    }

    ei_impulse_result_t result = { 0 };
    signal_t features_signal;
    features_signal.total_length = sizeof(f32_raw_features) / sizeof(f32_raw_features[0]);
    features_signal.get_data = &raw_feature_get_data;

    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
    printk("run_classifier returned: %d\n", res);

    if (res != 0) {
        return -1;
    }

#if 0
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        printk("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#endif

    *classification_result = get_classification_label(result.classification);

    return 0;
}