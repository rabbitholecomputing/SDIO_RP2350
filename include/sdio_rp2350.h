/** 
 * SDIO_RP2350 - Copyright (c) 2022-2025 Rabbit Hole Computing™
 * 
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

// SD card access using SDIO for RP2040 platform.
// This module contains the low-level SDIO bus implementation using
// the PIO peripheral. The high-level commands are in sd_card_sdio.cpp.

#pragma once
#include <stdint.h>
#include <stdbool.h>

#if defined __has_include
# if __has_include (<sdio_rp2350_config.h>)
#  include <sdio_rp2350_config.h>
#else
#  include <sdio_rp2350_config_example.h>
# endif
#else
# include <sdio_rp2350_config.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum sdio_status_t {
    SDIO_OK = 0,
    SDIO_BUSY = 1,
    SDIO_ERR_RESPONSE_TIMEOUT = 2, // Timed out waiting for response from card
    SDIO_ERR_RESPONSE_CRC = 3,     // Response CRC is wrong
    SDIO_ERR_RESPONSE_CODE = 4,    // Response command code does not match what was sent
    SDIO_ERR_DATA_TIMEOUT = 5,     // Timed out waiting for data block
    SDIO_ERR_DATA_CRC = 6,         // CRC for data packet is wrong
    SDIO_ERR_WRITE_CRC = 7,        // Card reports bad CRC for write
    SDIO_ERR_WRITE_FAIL = 8,       // Card reports write failure
    SDIO_ERR_STOP_TIMEOUT = 9,     // Timeout waiting for card to be idle
    SDIO_ERR_INVALID_PARAM = 10,   // Invalid parameters to function
};

// SDIO driver can optionally log debug and error messages.
// To enable this, edit sdio_rp2350_config.h
#ifndef SDIO_ERRMSG
#define SDIO_ERRMSG(txt, arg1, arg2)
#endif

#ifndef SDIO_DBGMSG
#define SDIO_DBGMSG(txt, arg1, arg2)
#endif

#ifndef SDIO_BLOCK_SIZE
#define SDIO_BLOCK_SIZE 512
#endif

#ifndef SDIO_MAX_CMD_RESPONSE_WORDS
#define SDIO_MAX_CMD_RESPONSE_WORDS 16
#endif

// Maximum number of blocks queued for transmission or reception
#ifndef SDIO_MAX_BLOCKS_PER_REQ
#define SDIO_MAX_BLOCKS_PER_REQ 128
#endif

#ifndef SDIO_TIME_US
#include <pico/time.h>
#define SDIO_TIME_US() ((uint32_t)get_absolute_time())
#endif

#ifndef SDIO_ELAPSED_US
#define SDIO_ELAPSED_US(start) ((uint32_t)(SDIO_TIME_US() - start))
#endif

#ifndef SDIO_WAIT_US
#include <pico/time.h>
#define SDIO_WAIT_US(x) (busy_wait_us_32((x)))
#endif

// Timeout for executing SDIO commands
#ifndef SDIO_CMD_TIMEOUT_US
#define SDIO_CMD_TIMEOUT_US 2000
#endif

// Timeout for read/write transfers, total
#ifndef SDIO_TRANSFER_TIMEOUT_US
#define SDIO_TRANSFER_TIMEOUT_US (1000 * 1000)
#endif

// Timeout for card initialization
#ifndef SDIO_INIT_TIMEOUT_US
#define SDIO_INIT_TIMEOUT_US (1000 * 1000)
#endif

// On 48-pin devices, IOBASE determines which pins
// the PIO block can access.
#ifndef SDIO_PIO_IOBASE
# if SDIO_CLK > 31
#  define SDIO_PIO_IOBASE 16
# else
#  define SDIO_PIO_IOBASE 0
# endif
#endif

// Enable the definition of SdFat library SdioCard class
#ifndef SDIO_USE_SDFAT
#define SDIO_USE_SDFAT 1
#endif

// Prefetch buffer in SdioCard, bytes
// Set to 0 to disable
#ifndef SDIO_SDFAT_PREFETCH_BUFFER
// TODO: Prefetch is issuing unnecessary StopTransmission() commands which kills performance
#define SDIO_SDFAT_PREFETCH_BUFFER 0
#endif

// Number of retries for sector read/write
#ifndef SDIO_MAX_RETRYCOUNT
#define SDIO_MAX_RETRYCOUNT 1
#endif

// When testing SDIO communication during init, which sector to read/write
#ifndef SDIO_COMMUNICATION_TEST_SECTOR_IDX
#define SDIO_COMMUNICATION_TEST_SECTOR_IDX 0
#endif

// Enable write check during initialization
// This writes back the same data as was read from the SD card
#ifndef SDIO_COMMUNICATION_TEST_DO_WRITE
#define SDIO_COMMUNICATION_TEST_DO_WRITE 1
#endif

// Default speed to use for SDIO communication
// If communication doesn't work, speed is automatically dropped
#ifndef SDIO_DEFAULT_SPEED
#define SDIO_DEFAULT_SPEED SDIO_HIGHSPEED
#endif

#ifndef SDIO_CARD_OCR_MODE
// See 4.2.3.1 Initialization Command (ACMD41) in SD specification
// Default value sets high capacity, maximum performance, 3.3V supply voltage
#define SDIO_CARD_OCR_MODE ((1 << 30) | (1 << 28) | (1 << 20))
#endif

// These come from properties of the PIO programs
#define SDIO_MIN_CMD_CLK_DIVIDER 6
#define SDIO_MAX_CMD_CLK_DIVIDER 65535
#define SDIO_MIN_DATA_CLK_DIVIDER 6
#define SDIO_MAX_DATA_CLK_DIVIDER 65535
#define SDIO_MIN_HS_DATA_CLK_DIVIDER 2
#define SDIO_MAX_HS_DATA_CLK_DIVIDER 15

// Flags that can be given to rp2350_sdio_command()
#define SDIO_FLAG_NO_CRC      0x0001
#define SDIO_FLAG_NO_LOGMSG   0x0002
#define SDIO_FLAG_NO_CMD_TAG  0x0004
#define SDIO_FLAG_STOP_CLK    0x0008

// Execute SDIO command with uint32_t result word.
// Convenience wrapper around rp2350_sdio_command()
sdio_status_t rp2350_sdio_command_u32(uint8_t command, uint32_t arg, uint32_t *response, uint32_t flags);

// Execute SDIO command
// If resp_bytes is 0, does not wait for reply.
sdio_status_t rp2350_sdio_command(uint8_t command, uint32_t arg, void *response, int resp_bytes, uint32_t flags);

// Start transferring data from SD card to memory buffer
// Memory buffer must be aligned to word boundary
sdio_status_t rp2350_sdio_rx_start(uint8_t *buffer, uint32_t num_blocks, uint32_t blocksize);

// Check if reception is complete
// Returns SDIO_BUSY while transferring, SDIO_OK when done and error on failure.
// If blocks_complete is not NULL, it will store how many blocks have been received.
sdio_status_t rp2350_sdio_rx_poll(uint32_t *blocks_complete);

// Start transferring data from memory to SD card
// Memory buffer must be aligned to word boundary
sdio_status_t rp2350_sdio_tx_start(const uint8_t *buffer, uint32_t num_blocks, uint32_t blocksize);

// Check if transmission is complete
// If blocks_complete is not NULL, it will store how many blocks have been sent.
sdio_status_t rp2350_sdio_tx_poll(uint32_t *blocks_complete);

// Force everything to idle state
sdio_status_t rp2350_sdio_stop();

// This should be kept in order by increasing speed
typedef enum {
    SDIO_INITIALIZE             = 0, // Initialization 400 kHz
    SDIO_MMC                    = 1, // Old MMC cards, 20 MHz
    SDIO_STANDARD               = 2, // Standard 25 MHz
    SDIO_HIGHSPEED              = 3, // High-speed 50 MHz
    SDIO_HIGHSPEED_OVERCLOCK    = 4, // High-speed 75 MHz, experimental
} rp2350_sdio_mode_t;

typedef struct {
    rp2350_sdio_mode_t mode;
    bool use_high_speed; // Uses the high-speed mode rising edge based timing
    int cmd_clk_divider;
    int data_clk_divider;
} rp2350_sdio_timing_t;

// Calculate timing parameters for mode based on current CPU frequency
rp2350_sdio_timing_t rp2350_sdio_get_timing(rp2350_sdio_mode_t mode);

// (Re)initialize the SDIO interface
// Note that for high speed modes you need to issue CMD6 to card to
// switch its mode.
void rp2350_sdio_init(rp2350_sdio_timing_t timing);

#ifdef SDIO_USE_SDFAT
// Optional callback can be used to do co-operative multitasking while SdFat is reading data.
// When a transfer to/from buffer is detected, callback gets called during transfer.
typedef void (*sd_callback_t)(uint32_t bytes_complete);
void rp2350_sdio_sdfat_set_callback(sd_callback_t func, const uint8_t *buffer);
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif
