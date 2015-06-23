/*
 * Copyright (c) 2008-2009 Thomas Roell.  All rights reserved.
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

#if defined(__ARM_ARCH_6M__)
	.arch armv6s-m
#else /* __ARM_ARCH_6M__ */
	.arch armv7-m
#endif /* __ARM_ARCH_6M__ */

	.syntax unified

	.global	NMI_Handler
	.global	HardFault_Handler
	
        .text
	.thumb
	.thumb_func
NMI_Handler:
	.thumb_func
HardFault_Handler:
	ldr	r1, =armv7m_exception_data
#if defined(__ARM_ARCH_6M__)
	mov	r0, lr
	lsrs	r0, #4
	mov	r0, sp
	bcc.n	1f
	mrs	r0, PSP
1:
#else /* __ARM_ARCH_6M__ */
	tst	lr, #0x00000004
	ite	eq
	moveq	r0, sp
	mrsne	r0, PSP
#endif /* __ARM_ARCH_6M__ */
	ldr	r2, [r0, #0]					// R0
	str	r2, [r1, #0]

	ldr	r2, [r0, #4]					// R1
	str	r2, [r1, #4]

	ldr	r2, [r0, #8]					// R2
	str	r2, [r1, #8]

	ldr	r2, [r0, #12]					// R3
	str	r2, [r1, #12]

	ldr	r2, [r0, #16]					// R12
	str	r2, [r1, #48]

	ldr	r2, [r0, #20]					// LR
	str	r2, [r1, #52]

	ldr	r2, [r0, #24]					// PC
	str	r2, [r1, #56]
	ldr	r2, [r0, #28]					// XPSR
	str	r2, [r1, #60]
	mrs	r2, MSP					        // MSP
	str	r2, [r1, #64]
	mrs	r2, PSP					        // PSP
	str	r2, [r1, #68]
	mrs	r2, PRIMASK				        // PRIMASK
	str	r2, [r1, #72]
	mrs	r2, BASEPRI				        // BASEPRI
	str	r2, [r1, #76]
	mrs	r2, FAULTMASK				        // FAULTMASK
	str	r2, [r1, #80]
	mrs	r2, CONTROL				        // CONTROL
	str	r2, [r1, #84]
	movs	r2, lr						// LR_EXCEPTION
	str	r2, [r1, #88]
	mrs	r2, IPSR				        // IPSR_EXCEPTION
	str	r2, [r1, #92]
	ldr	r3, =0xe000ed00
	ldr	r2, [r3, #0x28]					// SCB_CFSR */
	str	r2, [r1, #96]
	ldr	r2, [r3, #0x34]					// SCB_MMFAR */
	str	r2, [r1, #100]
	ldr	r2, [r3, #0x38]					// SCB_BFAR */
	str	r2, [r1, #104]
	adds	r0, 64
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        tst     lr, #0x00000010
        bne.n	2f
        vldmia  r0!, { s0-s15 }
	ldr	r2, [r0, #0]					// FPSCR */
	str	r2, [r1, #236]
        vstmdb  r1!, { s0-s31 }
	adds	r0, #8
2:	
#endif /* __VFP_FP__ && !__SOFTFP__ */
	str	r0, [r1, #48]					// SP
	bl	tm4c123_servo_fault
	b.n	.
