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

#ifndef YOU_WIN_TEXT_IMAGE_H
#define YOU_WIN_TEXT_IMAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#define YOU_WIN_TEXT_IMAGE_WIDTH 100
#define YOU_WIN_TEXT_IMAGE_HEIGHT 40
#define PIXEL_PER_BYTE 2

extern const unsigned char you_win_text_image[(YOU_WIN_TEXT_IMAGE_WIDTH * YOU_WIN_TEXT_IMAGE_HEIGHT) / PIXEL_PER_BYTE];

#ifdef __cplusplus
}  /* End of extern "C" block */
#endif

#endif /* YOU_WIN_TEXT_IMAGE_H */