/*
 * Copyright (c) 2015 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#if !defined(SIMULATION)
#include "kitty.h"
#endif

calibration_t calibration = {
#if (KITTY_CONFIG_ALTERNATIVE == 0)
    0.021,                                  /* rpm scale           */
#else /* KITTY_CONFIG_ALTERNATIVE == 0 */
    0.016,                                  /* rpm scale           */
#endif /* KITTY_CONFIG_ALTERNATIVE == 0 */
    1.0,                                    /* gyro scale          */
    0.0,                                    /* gyro bias           */
    { 0.0, 0.0, 0.0 },                      /* accel bias          */
    { 1.0, 1.0, 1.0 },                      /* accel scale         */
#if (KITTY_CONFIG_ALTERNATIVE == 0)
    { 1.437563, 27.622307, -77.063327 },
    { {  0.303435, -0.009147, -0.009757 },
      { -0.009147,  0.368494, -0.003557 },
      { -0.009757, -0.003557,  0.388703 } },
#else /* KITTY_CONFIG_ALTERNATIVE == 0 */
    { 64.584681, 23.810181, -115.091722 },
    { {  0.338349,  0.001759, -0.002715, },
      {  0.001759,  0.352185, -0.005869, },
      { -0.002715, -0.005869,  0.373855, } },
#endif /* KITTY_CONFIG_ALTERNATIVE == 0 */

    1456,                                   /* steering center     */
    200,                                    /* steering range min  */
    400,                                    /* steering range max  */
    4.0,                                    /* steering speed min  */
    8.0,                                    /* steering speed max  */
    600.0,                                  /* steering gain       */
    1525,                                   /* throttle min        */
    1975,                                   /* throttle max        */
    8.0,                                    /* throttle limit      */
    4,                                      /* throttle slope      */
    { 0.05, 0.0005, 0.0 },                  /* throttle Kp, Ki, Kd */
};


