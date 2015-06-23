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

#if !defined(_ARMV7M_EXCEPTION_H)
#define _ARMV7M_EXCEPTION_H

extern void NMI_Handler(void);
extern void HardFault_Handler(void);

typedef struct _armv7m_exception_data_t {
    uint32_t               R[12];          /* R0-R12,          0 */
    void                   *SP;            /* R13             48 */
    void                   *LR;            /* R14             52 */  
    void                   *PC;            /* R15             56 */  
    uint32_t               XPSR;           /* PSR             60 */
    uint32_t               MSP;            /* MSP             64 */
    uint32_t               PSP;            /* PSP             68 */
    uint32_t               PRIMASK;        /* PRIMASK         72 */
    uint32_t               BASEPRI;        /* BASEPRI         76 */
    uint32_t               FAULTMASK;      /* FAULTMASK       80 */
    uint32_t               CONTROL;        /* CONTROL         84 */
    uint32_t               LR_EXCEPTION;   /* LR_EXCEPTION    88 */
    uint32_t               IPSR_EXCEPTION; /* IPSR_EXCEPTION  92 */
    uint32_t               CFSR;           /* CFSR            96 */
    uint32_t               MMFAR;          /* MMFAR          100 */
    uint32_t               BFAR;           /* BFAR           104 */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    float                  S[32];          /* S0-S31         108 */
    uint32_t               FPSCR;          /* FPSCR          236 */
#endif /* __VFP_FP__ && !__SOFTFP__ */
} armv7m_exception_data_t;

#define SCB_CFSR                          (*((volatile uint32_t*)0xe000ed28))
#define SCB_MMFAR                         (*((volatile uint32_t*)0xe000ed34))
#define SCB_BFAR                          (*((volatile uint32_t*)0xe000ed38))

extern armv7m_exception_data_t armv7m_exception_data;

#endif /* _ARMV7M_EXCEPTION_H */
