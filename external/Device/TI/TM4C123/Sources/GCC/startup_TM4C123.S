/*
 * Copyright (c) 2014 Thomas Roell.  All rights reserved.
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
 * THE SOFTWARE IS PROVINCED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

	.arch armv7-m
	.syntax unified

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000000
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit
	
	.section .isr_vector
	.align	2
	.globl	__isr_vector
__isr_vector:
	.long	__StackTop	      /* Top of Stack */
	.long	Reset_Handler	      /* Reset Handler */
	.long	NMI_Handler	      /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0		      /* Reserved */
	.long	0		      /* Reserved */
	.long	0		      /* Reserved */
	.long	0		      /* Reserved */
	.long	SVC_Handler	      /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0		      /* Reserved */
	.long	PendSV_Handler	      /* PendSV Handler */
	.long	SysTick_Handler	      /* SysTick Handler */

	/* External interrupts */
	.long	GPIOA_IRQHandler	    /* GPIO Port A */
	.long	GPIOB_IRQHandler	    /* GPIO Port B */
	.long	GPIOC_IRQHandler	    /* GPIO Port C */
	.long	GPIOD_IRQHandler	    /* GPIO Port D */
	.long	GPIOE_IRQHandler	    /* GPIO Port E */
	.long	UART0_IRQHandler	    /* UART0 Rx and Tx */
	.long	UART1_IRQHandler	    /* UART1 Rx and Tx */
	.long	SSI0_IRQHandler		    /* SSI0 Rx and Tx */
	.long	I2C0_IRQHandler		    /* I2C0 Master and Slave */
	.long	PMW0_FAULT_IRQHandler	    /* PWM Fault */
	.long	PWM0_0_IRQHandler	    /* PWM Generator 0 */
	.long	PWM0_1_IRQHandler	    /* PWM Generator 1 */
	.long	PWM0_2_IRQHandler	    /* PWM Generator 2 */
	.long	QEI0_IRQHandler		    /* Quadrature Encoder 0 */
	.long	ADC0SS0_IRQHandler	    /* ADC Sequence 0 */
	.long	ADC0SS1_IRQHandler	    /* ADC Sequence 1 */
	.long	ADC0SS2_IRQHandler	    /* ADC Sequence 2 */
	.long	ADC0SS3_IRQHandler	    /* ADC Sequence 3 */
	.long	WDT0_IRQHandler		    /* Watchdog timer */
	.long	TIMER0A_IRQHandler	    /* Timer 0 subtimer A */
	.long	TIMER0B_IRQHandler	    /* Timer 0 subtimer B */
	.long	TIMER1A_IRQHandler	    /* Timer 1 subtimer A */
	.long	TIMER1B_IRQHandler	    /* Timer 1 subtimer B */
	.long	TIMER2A_IRQHandler	    /* Timer 2 subtimer A */
	.long	TIMER2B_IRQHandler	    /* Timer 2 subtimer B */
	.long	COMP0_IRQHandler	    /* Analog Comparator 0 */
	.long	COMP1_IRQHandler	    /* Analog Comparator 1 */
	.long	COMP2_IRQHandler	    /* Analog Comparator 2 */
	.long	SYSCTL_IRQHandler	    /* System Control (PLL, OSC, BO) */
	.long	FLASH_IRQHandler	    /* FLASH Control */
	.long	GPIOF_IRQHandler	    /* GPIO Port F */
	.long	GPIOG_IRQHandler	    /* GPIO Port G */
	.long	GPIOH_IRQHandler	    /* GPIO Port H */
	.long	UART2_IRQHandler	    /* UART2 Rx and Tx */
	.long	SSI1_IRQHandler		    /* SSI1 Rx and Tx */
	.long	TIMER3A_IRQHandler	    /* Timer 3 subtimer A */
	.long	TIMER3B_IRQHandler	    /* Timer 3 subtimer B */
	.long	I2C1_IRQHandler		    /* I2C1 Master and Slave */
	.long	QEI1_IRQHandler		    /* Quadrature Encoder 1 */
	.long	CAN0_IRQHandler		    /* CAN0 */
	.long	CAN1_IRQHandler		    /* CAN1 */
	.long	CAN2_IRQHandler		    /* CAN2 */
	.long	0			    /* Reserved */
	.long	HIB_IRQHandler		    /* Hibernate */
	.long	USB0_IRQHandler		    /* USB0 */
	.long	PWM0_3_IRQHandler	    /* PWM Generator 3 */
	.long	UDMA_IRQHandler		    /* uDMA Software Transfer */
	.long	UDMAERR_IRQHandler	    /* uDMA Error */
	.long	ADC1SS0_IRQHandler	    /* ADC1 Sequence 0 */
	.long	ADC1SS1_IRQHandler	    /* ADC1 Sequence 1 */
	.long	ADC1SS2_IRQHandler	    /* ADC1 Sequence 2 */
	.long	ADC1SS3_IRQHandler	    /* ADC1 Sequence 3 */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	GPIOJ_IRQHandler	    /* GPIO Port J */
	.long	GPIOK_IRQHandler	    /* GPIO Port K */
	.long	GPIOL_IRQHandler	    /* GPIO Port L */
	.long	SSI2_IRQHandler		    /* SSI2 Rx and Tx */
	.long	SSI3_IRQHandler		    /* SSI3 Rx and Tx */
	.long	UART3_IRQHandler	    /* UART3 Rx and Tx */
	.long	UART4_IRQHandler	    /* UART4 Rx and Tx */
	.long	UART5_IRQHandler	    /* UART5 Rx and Tx */
	.long	UART6_IRQHandler	    /* UART6 Rx and Tx */
	.long	UART7_IRQHandler	    /* UART7 Rx and Tx */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	I2C2_IRQHandler		    /* I2C2 Master and Slave */
	.long	I2C3_IRQHandler		    /* I2C3 Master and Slave */
	.long	TIMER4A_IRQHandler	    /* Timer 4 subtimer A */
	.long	TIMER4B_IRQHandler	    /* Timer 4 subtimer B */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	TIMER5A_IRQHandler	    /* Timer 5 subtimer A */
	.long	TIMER5B_IRQHandler	    /* Timer 5 subtimer B */
	.long	WTIMER0A_IRQHandler	    /* Wide Timer 0 subtimer A */
	.long	WTIMER0B_IRQHandler	    /* Wide Timer 0 subtimer B */
	.long	WTIMER1A_IRQHandler	    /* Wide Timer 1 subtimer A */
	.long	WTIMER1B_IRQHandler	    /* Wide Timer 1 subtimer B */
	.long	WTIMER2A_IRQHandler	    /* Wide Timer 2 subtimer A */
	.long	WTIMER2B_IRQHandler	    /* Wide Timer 2 subtimer B */
	.long	WTIMER3A_IRQHandler	    /* Wide Timer 3 subtimer A */
	.long	WTIMER3B_IRQHandler	    /* Wide Timer 3 subtimer B */
	.long	WTIMER4A_IRQHandler	    /* Wide Timer 4 subtimer A */
	.long	WTIMER4B_IRQHandler	    /* Wide Timer 4 subtimer B */
	.long	WTIMER5A_IRQHandler	    /* Wide Timer 5 subtimer A */
	.long	WTIMER5B_IRQHandler	    /* Wide Timer 5 subtimer B */
	.long	FPU_IRQHandler		    /* FPU */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	I2C4_IRQHandler		    /* I2C4 Master and Slave */
	.long	I2C5_IRQHandler		    /* I2C5 Master and Slave */
	.long	GPIOM_IRQHandler	    /* GPIO Port M */
	.long	GPION_IRQHandler	    /* GPIO Port N */
	.long	QEI2_IRQHandler		    /* Quadrature Encoder 2 */
	.long	0			    /* Reserved */
	.long	0			    /* Reserved */
	.long	GPIOP0_IRQHandler	    /* GPIO Port P (Summary or P0) */
	.long	GPIOP1_IRQHandler	    /* GPIO Port P1 */
	.long	GPIOP2_IRQHandler	    /* GPIO Port P2 */
	.long	GPIOP3_IRQHandler	    /* GPIO Port P3 */
	.long	GPIOP4_IRQHandler	    /* GPIO Port P4 */
	.long	GPIOP5_IRQHandler	    /* GPIO Port P5 */
	.long	GPIOP6_IRQHandler	    /* GPIO Port P6 */
	.long	GPIOP7_IRQHandler	    /* GPIO Port P7 */
	.long	GPIOQ0_IRQHandler	    /* GPIO Port Q (Summary or Q0) */
	.long	GPIOQ1_IRQHandler	    /* GPIO Port Q1 */
	.long	GPIOQ2_IRQHandler	    /* GPIO Port Q2 */
	.long	GPIOQ3_IRQHandler	    /* GPIO Port Q3 */
	.long	GPIOQ4_IRQHandler	    /* GPIO Port Q4 */
	.long	GPIOQ5_IRQHandler	    /* GPIO Port Q5 */
	.long	GPIOQ6_IRQHandler	    /* GPIO Port Q6 */
	.long	GPIOQ7_IRQHandler	    /* GPIO Port Q7 */
	.long	GPIOR_IRQHandler	    /* GPIO Port R */
	.long	GPIOS_IRQHandler	    /* GPIO Port S */
	.long	PMW1_0_IRQHandler	    /* PWM 1 Generator 0 */
	.long	PWM1_1_IRQHandler	    /* PWM 1 Generator 1 */
	.long	PWM1_2_IRQHandler	    /* PWM 1 Generator 2 */
	.long	PWM1_3_IRQHandler	    /* PWM 1 Generator 3 */
	.long	PWM1_FAULT_IRQHandler	    /* PWM 1 Fault */
	
	.size	__isr_vector, . - __isr_vector


	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
#ifndef __NO_SYSTEM_INIT
	bl    	SystemInit
#endif
	
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

	subs    r3, r2
	beq.n	2f

1:	subs	r3, #4
	ldr	r0, [r1, r3]
	str	r0, [r2, r3]
	bne.n	1b
2:

	movs	r0, #0
	ldr	r2, =__bss_start__
	ldr	r3, =__bss_end__
	subs    r3, r2
	beq.n	4f

3:	subs	r3, #4
	str	r0, [r2, r3]
	bne.n	3b
4:

#ifndef __START
#define __START _start
#endif
	b	__START

	.size	Reset_Handler, . - Reset_Handler

	.text
	.thumb
	.thumb_func
	.align	2
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

	/*
	 * Macro to define default handlers. Default handler
	 * will be weak symbol and just dead loops. They can be
	 * overwritten by other handlers
	 */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler
	def_irq_handler	GPIOA_IRQHandler
	def_irq_handler	GPIOB_IRQHandler
	def_irq_handler	GPIOC_IRQHandler
	def_irq_handler	GPIOD_IRQHandler
	def_irq_handler	GPIOE_IRQHandler
	def_irq_handler	UART0_IRQHandler
	def_irq_handler	UART1_IRQHandler
	def_irq_handler	SSI0_IRQHandler
	def_irq_handler	I2C0_IRQHandler
	def_irq_handler	PMW0_FAULT_IRQHandler
	def_irq_handler	PWM0_0_IRQHandler
	def_irq_handler	PWM0_1_IRQHandler
	def_irq_handler	PWM0_2_IRQHandler
	def_irq_handler	QEI0_IRQHandler
	def_irq_handler	ADC0SS0_IRQHandler
	def_irq_handler	ADC0SS1_IRQHandler
	def_irq_handler	ADC0SS2_IRQHandler
	def_irq_handler	ADC0SS3_IRQHandler
	def_irq_handler	WDT0_IRQHandler
	def_irq_handler	TIMER0A_IRQHandler
	def_irq_handler	TIMER0B_IRQHandler
	def_irq_handler	TIMER1A_IRQHandler
	def_irq_handler	TIMER1B_IRQHandler
	def_irq_handler	TIMER2A_IRQHandler
	def_irq_handler	TIMER2B_IRQHandler
	def_irq_handler	COMP0_IRQHandler
	def_irq_handler	COMP1_IRQHandler
	def_irq_handler	COMP2_IRQHandler
	def_irq_handler	SYSCTL_IRQHandler
	def_irq_handler	FLASH_IRQHandler
	def_irq_handler	GPIOF_IRQHandler
	def_irq_handler	GPIOG_IRQHandler
	def_irq_handler	GPIOH_IRQHandler
	def_irq_handler	UART2_IRQHandler
	def_irq_handler	SSI1_IRQHandler
	def_irq_handler	TIMER3A_IRQHandler
	def_irq_handler	TIMER3B_IRQHandler
	def_irq_handler	I2C1_IRQHandler
	def_irq_handler	QEI1_IRQHandler
	def_irq_handler	CAN0_IRQHandler
	def_irq_handler	CAN1_IRQHandler
	def_irq_handler	CAN2_IRQHandler
	def_irq_handler	HIB_IRQHandler
	def_irq_handler	USB0_IRQHandler
	def_irq_handler	PWM0_3_IRQHandler
	def_irq_handler	UDMA_IRQHandler
	def_irq_handler	UDMAERR_IRQHandler
	def_irq_handler	ADC1SS0_IRQHandler
	def_irq_handler	ADC1SS1_IRQHandler
	def_irq_handler	ADC1SS2_IRQHandler
	def_irq_handler	ADC1SS3_IRQHandler
	def_irq_handler	GPIOJ_IRQHandler
	def_irq_handler	GPIOK_IRQHandler
	def_irq_handler	GPIOL_IRQHandler
	def_irq_handler	SSI2_IRQHandler
	def_irq_handler	SSI3_IRQHandler
	def_irq_handler	UART3_IRQHandler
	def_irq_handler	UART4_IRQHandler
	def_irq_handler	UART5_IRQHandler
	def_irq_handler	UART6_IRQHandler
	def_irq_handler	UART7_IRQHandler
	def_irq_handler	I2C2_IRQHandler
	def_irq_handler	I2C3_IRQHandler
	def_irq_handler	TIMER4A_IRQHandler
	def_irq_handler	TIMER4B_IRQHandler
	def_irq_handler	TIMER5A_IRQHandler
	def_irq_handler	TIMER5B_IRQHandler
	def_irq_handler	WTIMER0A_IRQHandler
	def_irq_handler	WTIMER0B_IRQHandler
	def_irq_handler	WTIMER1A_IRQHandler
	def_irq_handler	WTIMER1B_IRQHandler
	def_irq_handler	WTIMER2A_IRQHandler
	def_irq_handler	WTIMER2B_IRQHandler
	def_irq_handler	WTIMER3A_IRQHandler
	def_irq_handler	WTIMER3B_IRQHandler
	def_irq_handler	WTIMER4A_IRQHandler
	def_irq_handler	WTIMER4B_IRQHandler
	def_irq_handler	WTIMER5A_IRQHandler
	def_irq_handler	WTIMER5B_IRQHandler
	def_irq_handler	FPU_IRQHandler
	def_irq_handler	I2C4_IRQHandler
	def_irq_handler	I2C5_IRQHandler
	def_irq_handler	GPIOM_IRQHandler
	def_irq_handler	GPION_IRQHandler
	def_irq_handler	QEI2_IRQHandler
	def_irq_handler	GPIOP0_IRQHandler
	def_irq_handler	GPIOP1_IRQHandler
	def_irq_handler	GPIOP2_IRQHandler
	def_irq_handler	GPIOP3_IRQHandler
	def_irq_handler	GPIOP4_IRQHandler
	def_irq_handler	GPIOP5_IRQHandler
	def_irq_handler	GPIOP6_IRQHandler
	def_irq_handler	GPIOP7_IRQHandler
	def_irq_handler	GPIOQ0_IRQHandler
	def_irq_handler	GPIOQ1_IRQHandler
	def_irq_handler	GPIOQ2_IRQHandler
	def_irq_handler	GPIOQ3_IRQHandler
	def_irq_handler	GPIOQ4_IRQHandler
	def_irq_handler	GPIOQ5_IRQHandler
	def_irq_handler	GPIOQ6_IRQHandler
	def_irq_handler	GPIOQ7_IRQHandler
	def_irq_handler	GPIOR_IRQHandler
	def_irq_handler	GPIOS_IRQHandler
	def_irq_handler	PMW1_0_IRQHandler
	def_irq_handler	PWM1_1_IRQHandler
	def_irq_handler	PWM1_2_IRQHandler
	def_irq_handler	PWM1_3_IRQHandler
	def_irq_handler	PWM1_FAULT_IRQHandler

	.end
	
