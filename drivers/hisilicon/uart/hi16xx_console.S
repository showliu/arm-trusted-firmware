#include <arch.h>
#include <asm_macros.S>
#include <hisilicon/hi16xx_uart.h>

	.globl console_core_init
	.globl console_core_putc
	.globl console_core_getc

/*
 * x0: console base address
 * w1: UART clock in Hz
 * w2: baud rate
 */
func console_core_init

	/* Check parameters */
	cbz	x0, 1f
	cbz	w1, 1f
	cbz	w2, 1f

	/* Clear and enable FIFO */
	mov	w3, #(UART0_FCR_FIFO_EN | UART0_FCR_RX_FIFO_RST | UART0_FCR_TX_FIFO_RST)
	strb	w3, [x0, #UART0_FCR]

	/* Enable access to _DLL and _DLH */
	mov	w3, #UART0_LCR_DLAB
	strb	w3, [x0, #UART0_LCR]

	/* Calculate and set UART_DLL */
	mov	w3, #16
	mul	w3, w2, w3
	udiv	w3, w1, w3
	and	w3, w3, #0xff
	strb	w3, [x0, #UART0_DLL]

	/* Calculate and set UART_DLH */
	mov	w3, #16
	mul	w3, w2, w3
	udiv	w3, w1, w3
	lsr	w3, w3, #8
	and	w3, w3, #0xff
	strb	w3, [x0, #UART0_DLH]

	/*
	 * Clear _DLL and _DLH access bit, set data size (8 bits), parity etc.
	 */
	mov	w3, #UART0_LCR_DLS8
	strb	w3, [x0, #UART0_LCR]

	/* Disable interrupt mode */
	mov	w3, #0x0
	strb	w3, [x0, #UART0_IEL]

1:
	ret

endfunc console_core_init

/*
 * w0: character to be printed
 * x1: console base address
 * Returns -1 on error else the character printed
 */
func console_core_putc

	/* Check address */
	cbz	x1, 2f

1:
	/* Loop until FIFO is not full */
	ldr	w2, [x1, #UART0_USR]
	tbz	w2, #UART0_USR_TFNF_BIT, 1b

	/* Write character */
	str	w0, [x1, #UART0_THR]
	ret

2:
	mov w0, #-1
	ret

endfunc console_core_putc

/*
 * w0: character to be printed
 * x1: console base address
 * Returns -1 on error else the character grabbed
 */
/* TODO: does not seem to work (blocks) */
func console_core_getc

	/* Check address */
	cbz	x1, 2f

1:
	/* Check if receive FIFO is empty */
	ldr	w2, [x1, #UART0_USR]
	tbz	w2, #UART0_USR_RFNE_BIT, 1b
	str	w0, [x1, #UART0_RBR]
	ret

2:
	mov w0, #-1
	ret

endfunc console_core_getc
