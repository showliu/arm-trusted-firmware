#ifndef __HISILICON_HI16XX_UART_H__
#define __HISILICON_HI16XX_UART_H__

/* Register offsets */

#define UART0_RBR	0x00	/* RX data buffer register */
#define UART0_THR	0x00	/* TX data buffer register */
#define UART0_DLL	0x00	/* Lower-bit frequency divider register */

#define UART0_IEL	0x04	/* Interrupt enable register */
#define UART0_DLH	0x04	/* Upper-bit frequency divider register */

#define UART0_FCR	0x08	/* FIFO control register */

#define UART0_LCR	0x0C	/* Line control register */

#define UART0_LSR	0x14	/* Line status register */

#define UART0_USR	0x7C	/* Status register */

/*
 * Line control register
 */

/* Data length selection */
#define UART0_LCR_DLS5	0x0	/* 5 bits */
#define UART0_LCR_DLS6	0x1	/* 6 bits */
#define UART0_LCR_DLS7	0x2	/* 7 bits */
#define UART0_LCR_DLS8	0x3	/* 8 bits */

/* Enable access to UART_DLL and UART_DLH */
#define UART0_LCR_DLAB	0x80

/*
 * FIFO control register
 */

#define UART0_FCR_FIFO_EN	0x1	/* Enable FIFO (depth: 32 bytes) */
#define UART0_FCR_RX_FIFO_RST	0x2	/* Clear receive FIFO (auto reset) */
#define UART0_FCR_TX_FIFO_RST	0x4	/* Clear send FIFO (auto reset) */


/*
 * Status register
 */

#define UART0_USR_BUSY_BIT	0	/* 0: idle/non-activated, 1: busy */
#define UART0_USR_TFNF_BIT	1	/* Transmit FIFO not full bit */
#define UART0_USR_TFE_BIT	2	/* Transmit FIFO empty bit */
#define UART0_USR_RFNE_BIT	3	/* Receive FIFO not empty bit */
#define UART0_USR_RFF_BIT	4	/* Receive FIFO full bit */

#endif /* __HISILICON_HI16XX_UART_H__ */
