/* Copy this file to rp2350_config.h in your own project */

#pragma once

// SDIO driver can optionally log debug and error messages.
// To enable this, uncomment the lines below and define the
// sdio_log() function in your own code.

// void sdio_log(const char *txt, uint32_t arg1, uint32_t arg2);
// #define SDIO_ERRMSG(txt, arg1, arg2) sdio_log(txt, arg1, arg2)
// #define SDIO_DBGMSG(txt, arg1, arg2) sdio_log(txt, arg1, arg2)

// Maximum number of blocks queued for transmission or reception
// #define SDIO_MAX_BLOCKS_PER_REQ 128

// Timeouts for operations, in microseconds
// #define SDIO_CMD_TIMEOUT_US 2000
// #define SDIO_TRANSFER_TIMEOUT_US (1000 * 1000)
// #define SDIO_INIT_TIMEOUT_US (1000 * 1000)

// Enable the definition of SdFat library SdioCard class
// #define SDIO_USE_SDFAT 1

// Prefetch buffer in SdioCard, bytes
// Set to 0 to disable
// #define SDIO_SDFAT_PREFETCH_BUFFER 2048

// Number of retries for sector read/write
// #define SDIO_MAX_RETRYCOUNT 1

// When testing SDIO communication during init, which sector to read/write
// #define SDIO_COMMUNICATION_TEST_SECTOR_IDX 0

// Enable write check during initialization
// This writes back the same data as was read from the SD card
// #define SDIO_COMMUNICATION_TEST_DO_WRITE 1

// Default speed to use for SDIO communication
// If communication doesn't work, speed is automatically dropped
// #define SDIO_DEFAULT_SPEED SDIO_HIGHSPEED

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
