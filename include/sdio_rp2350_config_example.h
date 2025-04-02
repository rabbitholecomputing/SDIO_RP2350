/* Copy this file to rp2350_config.h in your own project */

#pragma once

// SDIO driver can optionally log debug and error messages.
// To enable this, uncomment the lines below and define the
// sdio_log() function in your own code.

// void sdio_log(const char *txt, uint32_t arg1, uint32_t arg2);
// #define SDIO_ERRMSG(txt, arg1, arg2) sdio_log(txt, arg1, arg2)
// #define SDIO_DBGMSG(txt, arg1, arg2) sdio_log(txt, arg1, arg2)

// Optional timing dividers
// #define SDIO_CLOCKDIV_INIT 400
// #define SDIO_CLOCKDIV_CMD  1
// #define SDIO_CLOCKDIV_DATA 1

// Block size shouldn't be changed, but request queue size can be adjusted.
// #define SDIO_BLOCK_SIZE 512
// #define SDIO_MAX_BLOCKS_PER_REQ 128

// PIO block to use
#define SDIO_PIO pio1
#define SDIO_SM  0

// GPIO configuration
#define SDIO_GPIO_FUNC GPIO_FUNC_PIO1
#define SDIO_GPIO_SLEW GPIO_SLEW_RATE_FAST
#define SDIO_GPIO_DRIVE GPIO_DRIVE_STRENGTH_8MA

// DMA channels to use
#define SDIO_DMACH_A 4
#define SDIO_DMACH_B 5
#define SDIO_DMAIRQ_IDX 1
#define SDIO_DMAIRQ DMA_IRQ_1

// GPIO pins
#define SDIO_CLK 34
#define SDIO_CMD 35
#define SDIO_D0  36
#define SDIO_D1  37
#define SDIO_D2  38
#define SDIO_D3  39

#if SDIO_CLK > 31
#define SDIO_PIO_IOBASE 16
#endif

