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

#if !defined(_EKF_MATH_H)
#define _EKF_MATH_H

#include <stdint.h>

typedef struct _matrix_t {
    uint16_t rows;
    uint16_t cols;
    float    *data;
} matrix_t;

typedef const matrix_t *MATRIX;

#define MAT_ROWS(_a)	          ((_a)->rows)
#define	MAT_COLS(_a)	          ((_a)->cols)
#define	MAT_ELEMENT(_a,_row,_col) (*((_a)->data + ((_a)->cols * (_row)) + (_col)))

#define MAT_CREATE(_name,_row,_col) \
static float __mat_data_##_name[(_row)*(_col)]; \
static const matrix_t __mat_matrix_##_name = { (_row), (_col), __mat_data_##_name }; \
MATRIX _name = &__mat_matrix_##_name;

#define MAT_CREATE_STATIC(_name,_row,_col) \
static float __mat_data_##_name[(_row)*(_col)]; \
static const matrix_t __mat_matrix_##_name = { (_row), (_col), __mat_data_##_name }; \
static MATRIX _name = &__mat_matrix_##_name;

extern void mat_init_zero(MATRIX out);
extern void mat_init_identity(MATRIX out);
extern void mat_copy(MATRIX a, MATRIX out);
extern void mat_copy_transpose(MATRIX a, MATRIX out);
extern void mat_add(MATRIX a, MATRIX b, MATRIX out);
extern void mat_sub(MATRIX a, MATRIX b, MATRIX out);
extern void mat_mul(MATRIX a, MATRIX b, MATRIX out);
extern void mat_mul_transpose(MATRIX a, MATRIX b, MATRIX out);
extern void mat_scale(MATRIX a,float b, MATRIX out);
extern int mat_inverse(MATRIX m, MATRIX out);

#endif /* _EKF_MATH_H */

