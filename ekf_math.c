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

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define NDEBUG
#endif

#include <math.h>
#include <assert.h>

#include "ekf_math.h"

__attribute__((optimize("-O3"))) void mat_init_zero(MATRIX out)
{
    int i, j;

    for (i = 0; i < MAT_ROWS(out); i++) {
	for (j = 0; j < MAT_COLS(out); j++) {
	    MAT_ELEMENT(out,i,j) = 0.0;
	}
    }
}

__attribute__((optimize("-O3"))) void mat_init_identity(MATRIX out)
{
    int i, j;

    for (i = 0; i < MAT_ROWS(out); i++) {
	for (j = 0; j < MAT_COLS(out); j++) {
	    if (i == j) {
		MAT_ELEMENT(out,i,j) = 1.0;
	    } else {
		MAT_ELEMENT(out,i,j) = 0.0;
	    }
	}
    }
}

__attribute__((optimize("-O3"))) void mat_copy(MATRIX a, MATRIX out)
{
    int i, j;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_COLS(a) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(a); j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(a,i,j);
	}
    }
}

__attribute__((optimize("-O3"))) void mat_copy_transpose(MATRIX a, MATRIX out)
{
    int i, j;

    assert(MAT_ROWS(a) == MAT_COLS(out));
    assert(MAT_COLS(a) == MAT_ROWS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(a); j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(a,j,i);
	}
    }
}

__attribute__((optimize("-O3"))) void mat_add(MATRIX a, MATRIX b, MATRIX out)
{
    int i, j;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_COLS(a) == MAT_COLS(out));
    assert(MAT_ROWS(b) == MAT_ROWS(out));
    assert(MAT_COLS(b) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(a); j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(a,i,j) + MAT_ELEMENT(b,i,j);
	}
    }
}

__attribute__((optimize("-O3"))) void mat_sub(MATRIX a, MATRIX b, MATRIX out)
{
    int i, j;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_COLS(a) == MAT_COLS(out));
    assert(MAT_ROWS(b) == MAT_ROWS(out));
    assert(MAT_COLS(b) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(a); j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(a,i,j) - MAT_ELEMENT(b,i,j);
	}
    }
}

__attribute__((optimize("-O3"))) void mat_mul(MATRIX a, MATRIX b, MATRIX out)
{
    int i, j, k;
    float temp;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_COLS(b) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(b); j++) {
	    for (k = 0, temp = 0.0; k < MAT_COLS(a); k++) {
		temp += MAT_ELEMENT(a,i,k) * MAT_ELEMENT(b,k,j);
	    }
	    MAT_ELEMENT(out,i,j) = temp;
	}
    }
}

__attribute__((optimize("-O3"))) void mat_mul_transpose(MATRIX a, MATRIX b, MATRIX out)
{
    int	i, j, k;
    float temp;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_ROWS(b) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_ROWS(b); j++) {
	    for (k = 0, temp = 0.0; k < MAT_COLS(a); k++) {
		temp += MAT_ELEMENT(a,i,k) * MAT_ELEMENT(b,j,k);
	    }
	    MAT_ELEMENT(out,i,j) = temp;
	}
    }
}

__attribute__((optimize("-O3"))) void mat_scale(MATRIX a,float b, MATRIX out)
{
    int i, j;

    assert(MAT_ROWS(a) == MAT_ROWS(out));
    assert(MAT_COLS(a) == MAT_COLS(out));

    for (i = 0; i < MAT_ROWS(a); i++) {
	for (j = 0; j < MAT_COLS(a); j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(a,i,j) * b;
	}
    }
}


#define SMALL_NUMBER 1.e-9

/*
Matrix Inversion
by Richard Carling
from "Graphics Gems", Academic Press, 1990
*/

/*
 * float = mat_det2x2( a, b, c, d )
 * 
 * calculate the determinant of a 2x2 matrix.
 */

static inline __attribute__((optimize("-O3"))) float mat_det2x2(float a, float b, float c, float d)
{
    return a * d - b * c;
}

/*
 * float = mat_det3x3(  a1, a2, a3, b1, b2, b3, c1, c2, c3 )
 * 
 * calculate the determinant of a 3x3 matrix
 * in the form
 *
 *     | a1,  b1,  c1 |
 *     | a2,  b2,  c2 |
 *     | a3,  b3,  c3 |
 */

static inline __attribute__((optimize("-O3"))) float mat_det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3)
{
    return (+ a1 * mat_det2x2(b2, b3, c2, c3)
	    - b1 * mat_det2x2(a2, a3, c2, c3)
	    + c1 * mat_det2x2(a2, a3, b2, b3));
}

/*
 * float = mat_det4x4( matrix )
 * 
 * calculate the determinant of a 4x4 matrix.
 */
static inline __attribute__((optimize("-O3"))) float mat_det4x4(MATRIX m)
{
    float a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;

    /* assign to individual variable names to aid selecting */
    /*  correct elements */

    a1 = MAT_ELEMENT(m,0,0); b1 = MAT_ELEMENT(m,0,1); 
    c1 = MAT_ELEMENT(m,0,2); d1 = MAT_ELEMENT(m,0,3);

    a2 = MAT_ELEMENT(m,1,0); b2 = MAT_ELEMENT(m,1,1); 
    c2 = MAT_ELEMENT(m,1,2); d2 = MAT_ELEMENT(m,1,3);

    a3 = MAT_ELEMENT(m,2,0); b3 = MAT_ELEMENT(m,2,1); 
    c3 = MAT_ELEMENT(m,2,2); d3 = MAT_ELEMENT(m,2,3);

    a4 = MAT_ELEMENT(m,3,0); b4 = MAT_ELEMENT(m,3,1); 
    c4 = MAT_ELEMENT(m,3,2); d4 = MAT_ELEMENT(m,3,3);

    return (+ a1 * mat_det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4)
	    - b1 * mat_det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4)
	    + c1 * mat_det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4)
	    - d1 * mat_det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4));
}

/* 
 *   adjoint( original_matrix, inverse_matrix )
 * 
 *     calculate the adjoint of a 4x4 matrix
 *
 *      Let  a   denote the minor determinant of matrix A obtained by
 *           ij
 *
 *      deleting the ith row and jth column from A.
 *
 *                    i+j
 *     Let  b   = (-1)    a
 *          ij            ji
 *
 *    The matrix B = (b  ) is the adjoint of A
 *                     ij
 */

static inline __attribute__((optimize("-O3"))) void mat_adjoint(MATRIX m, MATRIX out)
{
    float a1, a2, a3, a4, b1, b2, b3, b4;
    float c1, c2, c3, c4, d1, d2, d3, d4;

    /* assign to individual variable names to aid  */
    /* selecting correct values  */

    a1 = MAT_ELEMENT(m,0,0); b1 = MAT_ELEMENT(m,0,1); 
    c1 = MAT_ELEMENT(m,0,2); d1 = MAT_ELEMENT(m,0,3);

    a2 = MAT_ELEMENT(m,1,0); b2 = MAT_ELEMENT(m,1,1); 
    c2 = MAT_ELEMENT(m,1,2); d2 = MAT_ELEMENT(m,1,3);

    a3 = MAT_ELEMENT(m,2,0); b3 = MAT_ELEMENT(m,2,1);
    c3 = MAT_ELEMENT(m,2,2); d3 = MAT_ELEMENT(m,2,3);

    a4 = MAT_ELEMENT(m,3,0); b4 = MAT_ELEMENT(m,3,1); 
    c4 = MAT_ELEMENT(m,3,2); d4 = MAT_ELEMENT(m,3,3);


    /* row column labeling reversed since we transpose rows & columns */


    MAT_ELEMENT(out,0,0) =   mat_det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4);
    MAT_ELEMENT(out,1,0) = - mat_det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4);
    MAT_ELEMENT(out,2,0) =   mat_det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4);
    MAT_ELEMENT(out,3,0) = - mat_det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4);
        
    MAT_ELEMENT(out,0,1) = - mat_det3x3(b1, b3, b4, c1, c3, c4, d1, d3, d4);
    MAT_ELEMENT(out,1,1) =   mat_det3x3(a1, a3, a4, c1, c3, c4, d1, d3, d4);
    MAT_ELEMENT(out,2,1) = - mat_det3x3(a1, a3, a4, b1, b3, b4, d1, d3, d4);
    MAT_ELEMENT(out,3,1) =   mat_det3x3(a1, a3, a4, b1, b3, b4, c1, c3, c4);
        
    MAT_ELEMENT(out,0,2) =   mat_det3x3(b1, b2, b4, c1, c2, c4, d1, d2, d4);
    MAT_ELEMENT(out,1,2) = - mat_det3x3(a1, a2, a4, c1, c2, c4, d1, d2, d4);
    MAT_ELEMENT(out,2,2) =   mat_det3x3(a1, a2, a4, b1, b2, b4, d1, d2, d4);
    MAT_ELEMENT(out,3,2) = - mat_det3x3(a1, a2, a4, b1, b2, b4, c1, c2, c4);
        
    MAT_ELEMENT(out,0,3) = - mat_det3x3(b1, b2, b3, c1, c2, c3, d1, d2, d3);
    MAT_ELEMENT(out,1,3) =   mat_det3x3(a1, a2, a3, c1, c2, c3, d1, d2, d3);
    MAT_ELEMENT(out,2,3) = - mat_det3x3(a1, a2, a3, b1, b2, b3, d1, d2, d3);
    MAT_ELEMENT(out,3,3) =   mat_det3x3(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

/* 
 *   inverse( original_matrix, inverse_matrix )
 * 
 *    calculate the inverse of a 4x4 matrix
 *
 *     -1     
 *     A  = ___1__ adjoint A
 *         det A
 */

__attribute__((optimize("-O3"))) int mat_inverse(MATRIX m, MATRIX out)
{
    int i, j;
    float det;

    assert(MAT_ROWS(m) == 4);
    assert(MAT_COLS(m) == 4);
    assert(MAT_ROWS(out) == 4);
    assert(MAT_COLS(out) == 4);

    /* calculate the adjoint matrix */

    mat_adjoint(m, out);

    /*  calculate the 4x4 determinant
     *  if the determinant is zero, 
     *  then the inverse matrix is not unique.
     */

    det = mat_det4x4(m);

    if (fabs(det) < SMALL_NUMBER) {
	return -1;
    }

    /* scale the adjoint matrix to get the inverse */

    for (i=0; i<4; i++) {
        for(j=0; j<4; j++) {
	    MAT_ELEMENT(out,i,j) = MAT_ELEMENT(out,i,j) / det;
	}
    }

    return 0;
}

/*******************************************************************************************************************************/

