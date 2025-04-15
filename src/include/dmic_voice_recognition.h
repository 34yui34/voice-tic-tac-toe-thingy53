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

#ifndef DMIC_VOICE_RECOGNITION_H
#define DMIC_VOICE_RECOGNITION_H

typedef enum {
    ONE_ONE_DETECTED,
    ONE_TWO_DETECTED,
    ONE_THREE_DETECTED,
    TWO_ONE_DETECTED,
    TWO_TWO_DETECTED,
    TWO_THREE_DETECTED,
    THREE_ONE_DETECTED,
    THREE_TWO_DETECTED,
    THREE_THREE_DETECTED,
    RESET_DETECTED,
    NOISE_DETECTED,
    NONE_DETECTED,
} voice_classification_label_t;

/**
* @brief Initialize DMIC module for voice recognition
* 
* @return int 0 on success, negative errno on failure
*/
int dmic_voice_recognition_init(void);

/**
* @brief Initialize DMIC module for voice recognition
* @param classification_result - Pointer to store classification result 
*
* @return int 0 on success, negative errno on failure
*/
int dmic_voice_recognition_sample_and_classify(voice_classification_label_t * classification_result);

#endif /* DMIC_VOICE_RECOGNITION_H */