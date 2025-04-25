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

// Driver for accessing SD card in SDIO mode on RP2350 using SdFat library

#include "sdio_rp2350.h"

#if SDIO_USE_SDFAT
#include <hardware/gpio.h>
#include <SdFat.h>
#include <SdCard/SdCardInfo.h>

static uint32_t g_sdio_ocr; // Operating condition register from card
static uint32_t g_sdio_rca; // Relative card address
static cid_t g_sdio_cid;
static csd_t g_sdio_csd;
static int g_sdio_error_line;
static sdio_status_t g_sdio_error;
static uint32_t g_sdio_tmp_buf[SDIO_BLOCK_SIZE / 4];
static uint32_t g_sdio_sector_count;
static int g_sdio_clk_hz;

#if SDIO_SDFAT_PREFETCH_BUFFER >= SDIO_BLOCK_SIZE
#define USE_PREFETCH
#define PREFETCH_BLOCKS (SDIO_SDFAT_PREFETCH_BUFFER / SDIO_BLOCK_SIZE)
static uint32_t g_sdio_prefetch_buf[PREFETCH_BLOCKS][SDIO_BLOCK_SIZE / 4];
static struct {
    uint32_t sector; // First sector number of prefetch
    uint32_t readcnt; // Total number of blocks that have been read out from the buffer
    uint32_t writecnt; // Total number of blocks that have been written to the buffer

    uint32_t prefetch_start; // Value of 'writecnt' when prefetch operation started
    uint32_t prefetch_count; // Number of blocks queued for active read operation
} g_sdio_prefetch;
#endif

#define checkReturnOk(call) ((g_sdio_error = (call)) == SDIO_OK ? true : logSDError(__LINE__))
static bool logSDError(int line)
{
    g_sdio_error_line = line;
    SDIO_ERRMSG("SDIO error on line", line, g_sdio_error);
    return false;
}

static void prefetchClear();

// Optional callback can be used to do co-operative multitasking while SdFat is reading data.
// When a transfer to/from buffer is detected, callback gets called during transfer.
static struct {
    sd_callback_t callback;
    const uint8_t *buffer;
    uint32_t bytes;         // Number of bytes transferred after current operation
    uint32_t bytes_start;   // Number of bytes transferred when operation started
} g_sd_callback;

void rp2350_sdio_sdfat_set_callback(sd_callback_t func, const uint8_t *buffer)
{
    g_sd_callback.callback = func;
    g_sd_callback.buffer = buffer;
    g_sd_callback.bytes = 0;
    g_sd_callback.bytes_start = 0;
}

// Check if the buffer pointer matches the one requested for the callback, and return the function pointer.
// Increments the internal stream count to check for sequential access to buffer.
static sd_callback_t get_stream_callback(const uint8_t *buf, uint32_t count, bool read, uint32_t sector)
{
    g_sd_callback.bytes_start = g_sd_callback.bytes;

    if (g_sd_callback.callback)
    {
        if (buf == g_sd_callback.buffer + g_sd_callback.bytes)
        {
            g_sd_callback.bytes += count;
            return g_sd_callback.callback;
        }
        else
        {
            if (read)
                SDIO_DBGMSG("SD card read does not match callback buffer", sector, count);
            else
                SDIO_DBGMSG("SD card write does not match callback buffer", sector, count);
            return NULL;
        }
    }
    
    return NULL;
}

// Read just a single sector. This is the most basic access mode and it is used
// for diagnostics and retry. The dst buffer must be aligned to 4 bytes.
static bool read_single_sector(SdioCard *card, uint32_t sector, uint8_t *dst)
{
    SDIO_DBGMSG("read_single_sector", sector, (uint32_t)dst);

    // Cards up to 2GB use byte addressing, SDHC cards use sector addressing
    uint32_t address = (card->type() == SD_CARD_TYPE_SDHC) ? sector : (sector * 512);

    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(16, SDIO_BLOCK_SIZE, &reply, 0)) || // SET_BLOCKLEN
        !checkReturnOk(rp2350_sdio_command_u32(CMD17, address, &reply, SDIO_FLAG_STOP_CLK)) || // READ_SINGLE_BLOCK
        !checkReturnOk(rp2350_sdio_rx_start(dst, 1, SDIO_BLOCK_SIZE))) // Prepare for reception
    {
        return false;
    }

    if (reply & 0xFFF80000)
    {
        // It is normal for the last sector to return OUT_OF_RANGE
        if (sector != g_sdio_sector_count - 1)
        {
            SDIO_ERRMSG("read_single_sector error", sector, reply);
            rp2350_sdio_stop();
            return false;
        }
    }

    do {
        g_sdio_error = rp2350_sdio_rx_poll(NULL);
    } while (g_sdio_error == SDIO_BUSY);

    rp2350_sdio_stop();

    if (g_sdio_error == SDIO_OK)
    {
        SDIO_DBGMSG("sector read success", *(uint32_t*)(dst + 0), *(uint32_t*)(dst + 4));
        return true;
    }
    else
    {
        SDIO_ERRMSG("sector read failure", *(uint32_t*)(dst + 0), g_sdio_error);
        return false;
    }
}

// Write just a single sector. This is the most basic access mode and it is used
// for diagnostics and retry. The src buffer must be aligned to 4 bytes.
static bool write_single_sector(SdioCard *card, uint32_t sector, const uint8_t *src)
{
    SDIO_DBGMSG("write_single_sector", sector, (uint32_t)src);

    // Cards up to 2GB use byte addressing, SDHC cards use sector addressing
    uint32_t address = (card->type() == SD_CARD_TYPE_SDHC) ? sector : (sector * 512);

    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(16, SDIO_BLOCK_SIZE, &reply, 0)) || // SET_BLOCKLEN
        !checkReturnOk(rp2350_sdio_command_u32(CMD24, address, &reply, SDIO_FLAG_STOP_CLK)) || // WRITE_BLOCK
        !checkReturnOk(rp2350_sdio_tx_start(src, 1, SDIO_BLOCK_SIZE))) // Start transmission
    {
        return false;
    }

    do {
        g_sdio_error = rp2350_sdio_tx_poll(NULL);
    } while (g_sdio_error == SDIO_BUSY);

    rp2350_sdio_stop();

    if (g_sdio_error == SDIO_OK)
    {
        SDIO_DBGMSG("sector write success", 0, 0);
        return true;
    }
    else
    {
        SDIO_ERRMSG("sector write failure", g_sdio_error, 0);
        return false;
    }
}

// Check that bidirectional data communication with the SD card works.
// This reads and rewrites a single sector of the card with the same data.
// If the sector data has non-zero bytes, this is a reasonable check that
// the data bus communication works.
static bool sd_communication_check(SdioCard *card, const uint32_t *expected, uint32_t *actual)
{
    uint32_t test_sector = SDIO_COMMUNICATION_TEST_SECTOR_IDX;
    if (!read_single_sector(card, test_sector, (uint8_t*)&g_sdio_tmp_buf))
    {
        return false;
    }

    uint32_t checksum = 1;
    bool all_zero = true;
    for (int i = 0; i < SDIO_BLOCK_SIZE / 4; i++)
    {
        checksum ^= checksum << 13;
        checksum ^= checksum >> 17;
        checksum ^= checksum << 5;
        checksum += g_sdio_tmp_buf[i];
        all_zero &= (g_sdio_tmp_buf[i] == 0);
    }

    if (actual)
    {
        *actual = checksum;
    }

    if (expected && checksum != *expected)
    {
        SDIO_ERRMSG("Communication check checksum mismatch", checksum, *expected);
        return false;
    }

    if (all_zero)
    {
        SDIO_DBGMSG("Test sector is all-zero, communication check may not detect problems", test_sector, 0);
    }

#if SDIO_COMMUNICATION_TEST_DO_WRITE
    // If communication fails, we should get a CRC error and no data is written.
    if (!write_single_sector(card, test_sector, (uint8_t*)&g_sdio_tmp_buf))
    {
        return false;
    }
#endif

    return true;
}

bool SdioCard::begin(SdioConfig sdioConfig)
{
    uint32_t reply;
    sdio_status_t status;
    
    m_curState = IDLE_STATE;
    prefetchClear();
    g_sdio_sector_count = 0;
    g_sdio_error = SDIO_OK;
    g_sdio_error_line = 0;
    g_sdio_clk_hz = 0;

    // Initialize at 400 kHz clock speed
    rp2350_sdio_init(rp2350_sdio_get_timing(SDIO_INITIALIZE));

    // Wait for initial clock cycles
    SDIO_WAIT_US(1000);

    // Establish initial connection with the card
    for (int retries = 0; retries < 5; retries++)
    {
        SDIO_WAIT_US(1000);
        reply = 0;
        rp2350_sdio_command(CMD0, 0, NULL, 0, SDIO_FLAG_NO_LOGMSG); // GO_IDLE_STATE
        SDIO_WAIT_US(1000);
        status = rp2350_sdio_command_u32(CMD8, 0x1AA, &reply, SDIO_FLAG_NO_LOGMSG); // SEND_IF_COND
        
        if (status == SDIO_OK && reply == 0x1AA)
        {
            break;
        }
    }

    if (reply != 0x1AA || status != SDIO_OK)
    {
        SDIO_DBGMSG("No response to CMD8 SEND_IF_COND", status, reply);
        g_sdio_error = status;
        return false;
    }

    // Send ACMD41 to begin card initialization and wait for it to complete
    uint32_t start = SDIO_TIME_US();
    do {
        if (!checkReturnOk(rp2350_sdio_command_u32(CMD55, 0, &reply, 0)) || // APP_CMD
            !checkReturnOk(rp2350_sdio_command_u32(ACMD41, SDIO_CARD_OCR_MODE, &g_sdio_ocr, SDIO_FLAG_NO_CRC | SDIO_FLAG_NO_CMD_TAG)))
        {
            return false;
        }

        if (SDIO_ELAPSED_US(start) > SDIO_INIT_TIMEOUT_US)
        {
            SDIO_ERRMSG("SD card initialization timeout", g_sdio_ocr, 0);
            return false;
        }
    } while (!(g_sdio_ocr & (1 << 31)));

    // Get CID
    if (!checkReturnOk(rp2350_sdio_command(CMD2, 0, &g_sdio_cid, 16, SDIO_FLAG_NO_CRC | SDIO_FLAG_NO_CMD_TAG)))
    {
        SDIO_ERRMSG("Failed to read CID", 0, 0);
        return false;
    }

    // Get relative card address
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD3, 0, &g_sdio_rca, 0)))
    {
        SDIO_ERRMSG("Failed to read RCA", 0, 0);
        return false;
    }

    // Get CSD
    if (!checkReturnOk(rp2350_sdio_command(CMD9, g_sdio_rca, &g_sdio_csd, 16, SDIO_FLAG_NO_CRC | SDIO_FLAG_NO_CMD_TAG)))
    {
        SDIO_ERRMSG("Failed to read CSD", 0, 0);
        return false;
    }

    g_sdio_sector_count = sectorCount();

    // Select card
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD7, g_sdio_rca, &reply, 0)))
    {
        SDIO_ERRMSG("Failed to select card", 0, 0);
        return false;
    }

    // Set 4-bit bus mode
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD55, g_sdio_rca, &reply, 0)) ||
        !checkReturnOk(rp2350_sdio_command_u32(ACMD6, 2, &reply, 0)))
    {
        SDIO_ERRMSG("Failed to set bus width", 0, 0);
        return false;
    }

    // Check that data bus works at this slow speed.
    uint32_t checksum;
    if (!sd_communication_check(this, NULL, &checksum))
    {
        SDIO_ERRMSG("SDIO data bus is not working", 0, 0);
        return false;
    }
    
    // Store the expected data
    uint32_t words_per_block = SDIO_BLOCK_SIZE / 4;
    uint32_t first = g_sdio_tmp_buf[0];
    uint32_t last = g_sdio_tmp_buf[words_per_block - 1];
    SDIO_DBGMSG("Reference data", first, last);

    rp2350_sdio_mode_t mode = SDIO_DEFAULT_SPEED;
    rp2350_sdio_timing_t timing;

    while ((int)mode > SDIO_INITIALIZE)
    {
        // Select clock rate to use
        timing = rp2350_sdio_get_timing(mode);
        
        // Configure card mode with CMD6
        uint32_t arg = timing.use_high_speed ? 0x80FFFF01 : 0x80FFFF00;
        cardCMD6(arg, (uint8_t*)&g_sdio_tmp_buf);
        
        // Apply new clock rate
        rp2350_sdio_init(timing);

        // Check that communication works at this clock rate
        if (!sd_communication_check(this, &checksum, NULL))
        {
            mode = (rp2350_sdio_mode_t)((int)mode - 1);
            SDIO_ERRMSG("SDIO communication check failed, reducing speed", (int)mode, 0);

            // Return to slow rate for running CMD6
            rp2350_sdio_init(rp2350_sdio_get_timing(SDIO_INITIALIZE));
        }
        else
        {
            break;
        }
    }

    g_sdio_clk_hz = clock_get_hz(clk_sys) / timing.data_clk_divider;
    SDIO_DBGMSG("SDIO card initialization succeeded", g_sdio_clk_hz, mode);

    return true;
}

uint8_t SdioCard::errorCode() const
{
    return g_sdio_error;
}

uint32_t SdioCard::errorData() const
{
    return 0;
}

uint32_t SdioCard::errorLine() const
{
    return g_sdio_error_line;
}

bool SdioCard::isBusy() 
{
    if (m_curState != IDLE_STATE)
    {
        stopTransmission(false);
    }

    return gpio_get(SDIO_D0) == 0;
}

uint32_t SdioCard::kHzSdClk()
{
    return g_sdio_clk_hz / 1000;
}

bool SdioCard::readCID(cid_t* cid)
{
    *cid = g_sdio_cid;
    return true;
}

bool SdioCard::readCSD(csd_t* csd)
{
    *csd = g_sdio_csd;
    return true;
}

bool SdioCard::readSDS(sds_t* sds)
{
    if (m_curState != IDLE_STATE)
    {
        stopTransmission(false);
    }

    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD55, g_sdio_rca, &reply, 0)) ||
        !checkReturnOk(rp2350_sdio_command_u32(ACMD13, 0, &reply, SDIO_FLAG_STOP_CLK)))
    {
        rp2350_sdio_stop();
        return false;
    }

    rp2350_sdio_rx_start((uint8_t*)g_sdio_tmp_buf, 1, 64);

    do {
        g_sdio_error = rp2350_sdio_rx_poll(NULL);
    } while (g_sdio_error == SDIO_BUSY);

    rp2350_sdio_stop();

    memcpy(sds, g_sdio_tmp_buf, sizeof(*sds));
    return true;
}

bool SdioCard::readOCR(uint32_t* ocr)
{
    *ocr = g_sdio_ocr;
    return true;
}

#ifndef USE_PREFETCH
static void prefetchClear() {}
static bool prefetchIsEmpty() { return true; }
static bool prefetchBusy(SdioCard *card) { return false; }
static bool prefetchProcess(SdioCard *card) { return false; }
static uint32_t prefetchStart(SdioCard *card, uint32_t sector) { return sector; }
static bool prefetchSeek(SdioCard *card, uint32_t sector) { return false; }
static void prefetchRead(SdioCard *card, uint8_t *dst)
{
    SDIO_ERRMSG("prefetchRead() called with prefetch disabled", 0, 0);
    assert(false);
}
#else
static void prefetchClear()
{
    memset(&g_sdio_prefetch, 0, sizeof(g_sdio_prefetch));
}

static bool prefetchIsEmpty()
{
    return g_sdio_prefetch.readcnt == g_sdio_prefetch.writecnt;
}

// Is a prefetch read operation in progress?
static bool prefetchBusy(SdioCard *card)
{
    return g_sdio_prefetch.prefetch_count > 0;
}

// Process any received prefetch blocks
static bool prefetchProcess(SdioCard *card)
{
    if (!prefetchBusy(card))
    {
        // Nothing in progress
        return true;
    }

    uint32_t blocks_done;
    g_sdio_error = rp2350_sdio_rx_poll(&blocks_done);

    g_sdio_prefetch.writecnt = g_sdio_prefetch.prefetch_start + blocks_done;

    if (g_sdio_error == SDIO_BUSY)
    {
        // Still transferring
        return true;
    }
    else if (g_sdio_error == SDIO_OK)
    {
        SDIO_DBGMSG("Prefetch transfer done",
            g_sdio_prefetch.sector + g_sdio_prefetch.prefetch_start,
            g_sdio_prefetch.prefetch_count);
        g_sdio_prefetch.writecnt = g_sdio_prefetch.prefetch_start + g_sdio_prefetch.prefetch_count;
        g_sdio_prefetch.prefetch_start = 0;
        g_sdio_prefetch.prefetch_count = 0;
        return true;
    }
    else
    {
        // Prefetch error
        SDIO_ERRMSG("Prefetch failed",
            g_sdio_prefetch.sector + g_sdio_prefetch.prefetch_start,
            g_sdio_prefetch.prefetch_count);
        card->stopTransmission(true);
        return false;
    }
}

// Start a prefetch read of as many sectors as will fit in the buffer.
// Sector should be the index of next sector that is going to come from the card.
static uint32_t prefetchStart(SdioCard *card, uint32_t sector)
{
    if (prefetchBusy(card))
    {
        // Previous operation is not yet finished
        prefetchProcess(card);
        if (prefetchBusy(card))
        {
            return sector;
        }
    }

    uint32_t oldsector = g_sdio_prefetch.sector + g_sdio_prefetch.writecnt;
    if (sector != oldsector)
    {
        SDIO_DBGMSG("Prefetch location changed", sector, oldsector);
        prefetchClear();
        g_sdio_prefetch.sector = sector;
    }

    // How many blocks can we read before buffer is full or wraps?
    uint32_t buffer_fill = g_sdio_prefetch.writecnt - g_sdio_prefetch.readcnt;
    uint32_t max_blocks = PREFETCH_BLOCKS - buffer_fill;
    uint32_t writeidx = g_sdio_prefetch.writecnt % PREFETCH_BLOCKS;
    uint32_t wrappos = PREFETCH_BLOCKS - writeidx;
    if (max_blocks > wrappos) max_blocks = wrappos;

    if (max_blocks == 0)
    {
        SDIO_DBGMSG("Prefetch buffer full", g_sdio_prefetch.writecnt, g_sdio_prefetch.readcnt);
        return sector;
    }

    g_sdio_prefetch.prefetch_start = g_sdio_prefetch.writecnt;
    g_sdio_prefetch.prefetch_count = max_blocks;

    if (!checkReturnOk(rp2350_sdio_rx_start((uint8_t*)&g_sdio_prefetch_buf[writeidx], max_blocks, SDIO_BLOCK_SIZE)))
    {
        SDIO_ERRMSG("Prefetch read start failed", g_sdio_error, sector);
        g_sdio_prefetch.prefetch_count = 0;
        card->stopTransmission(true);
    }

    SDIO_DBGMSG("Prefetch read started", sector, max_blocks);
    return sector + max_blocks;
}

// Query whether a given sector is available from prefetch buffer
// and advance read pointer to it.
static bool prefetchSeek(SdioCard *card, uint32_t sector)
{
    prefetchProcess(card);

    if (sector >= g_sdio_prefetch.sector + g_sdio_prefetch.writecnt &&
        sector < g_sdio_prefetch.sector + g_sdio_prefetch.prefetch_start + g_sdio_prefetch.prefetch_count &&
        sector < g_sdio_prefetch.sector + g_sdio_prefetch.writecnt + 3)
    {
        // Sector is part of currently running prefetch request and will be available
        // soon, wait for it.
        while (sector >= g_sdio_prefetch.sector + g_sdio_prefetch.writecnt)
        {
            if (!prefetchProcess(card)) return false;
        }
    }

    if (sector < g_sdio_prefetch.sector + g_sdio_prefetch.readcnt)
    {
        SDIO_DBGMSG("Prefetch miss under", sector, g_sdio_prefetch.sector + g_sdio_prefetch.readcnt);
        return false; // Sector has already been discarded or precedes operation
    }
    else if (sector >= g_sdio_prefetch.sector + g_sdio_prefetch.writecnt)
    {
        SDIO_DBGMSG("Prefetch miss over", sector, g_sdio_prefetch.sector + g_sdio_prefetch.writecnt);
        return false; // Sector has not yet been fetched
    }
    else
    {
        g_sdio_prefetch.readcnt = sector - g_sdio_prefetch.sector;
        SDIO_DBGMSG("Prefetch seek", sector, g_sdio_prefetch.readcnt);
        return true;
    }
}

// Copy data from prefetch read pointer and remove it from the buffer.
// prefetchSeek() should be called before this.
static void prefetchRead(SdioCard *card, uint8_t *dst)
{
    uint32_t pos = g_sdio_prefetch.readcnt % PREFETCH_BLOCKS;
    SDIO_DBGMSG("Prefetch read", g_sdio_prefetch.sector + g_sdio_prefetch.readcnt, pos);
    memcpy(dst, g_sdio_prefetch_buf[pos], SDIO_BLOCK_SIZE);
    g_sdio_prefetch.readcnt++;
}

#endif /* USE_PREFETCH */

bool SdioCard::readStart(uint32_t sector)
{
    if (m_curState == READ_STATE)
    {
        if (m_curSector == sector && !prefetchBusy(this))
        {
            SDIO_DBGMSG("Continuing multi-sector read", sector, 0);
            prefetchClear();
            return true;
        }
        else if (prefetchSeek(this, sector))
        {
            SDIO_DBGMSG("readStart: Block is available from prefetch", sector, 0);
            return true;
        }
    }

    if (m_curState != IDLE_STATE)
    {
        stopTransmission(true);
    }

    prefetchClear();

    // Cards up to 2GB use byte addressing, SDHC cards use sector addressing
    uint32_t address = (type() == SD_CARD_TYPE_SDHC) ? sector : (sector * 512);

    // Send the read command and then stop clock before first block
    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD18, address, &reply, SDIO_FLAG_STOP_CLK))) // READ_MULTIPLE_BLOCK
    {
        stopTransmission(false);
        return false;
    }

    m_curState = READ_STATE;
    m_curSector = sector;
    
    return true;
}

bool SdioCard::readData(uint8_t* dst)
{
    if (m_curState != READ_STATE)
    {
        SDIO_ERRMSG("readData() called when not in READ_STATE", (int)m_curState, dst);
        return false;
    }

    // If readStart() doesn't find data in prefetch buffer,
    // it will clear it.
    if (!prefetchIsEmpty())
    {
        prefetchRead(this, dst);
        return true;
    }

    if (!checkReturnOk(rp2350_sdio_rx_start(dst, 1, SDIO_BLOCK_SIZE)))
    {
        SDIO_ERRMSG("rp2350_sdio_rx_start failed", g_sdio_error, 0);
        return false;
    }

    sd_callback_t callback = get_stream_callback(dst, SDIO_BLOCK_SIZE, true, m_curSector);
    
    do {
        uint32_t blocks_done;
        g_sdio_error = rp2350_sdio_rx_poll(&blocks_done);

        if (callback)
        {
            callback(g_sd_callback.bytes_start + SDIO_BLOCK_SIZE * blocks_done);
        }
    } while (g_sdio_error == SDIO_BUSY);

    if (g_sdio_error != SDIO_OK)
    {
        SDIO_ERRMSG("SdioCard::readData failed", g_sdio_error, m_curSector);
        return false;
    }

    m_curSector++;
    return true;
}

bool SdioCard::readStop()
{
    return stopTransmission(true);
}

uint32_t SdioCard::sectorCount()
{
    return g_sdio_csd.capacity();
}

uint32_t SdioCard::status()
{
    if (m_curState != IDLE_STATE)
    {
        stopTransmission(true);
    }

    uint32_t reply;
    if (checkReturnOk(rp2350_sdio_command_u32(CMD13, g_sdio_rca, &reply, 0)))
        return reply;
    else
        return 0;
}

bool SdioCard::stopTransmission(bool blocking)
{
    // It's normal to get no response to CMD12 (STOP_TRANSMISSION)
    // if there is no transmission in progress. Running a command
    // also forces the low level code back to CMD state machine
    // and a continuously running clock.
    uint32_t reply;
    rp2350_sdio_command_u32(CMD12, 0, &reply, SDIO_FLAG_NO_LOGMSG);
    m_curState = IDLE_STATE;

    // Invalidate prefetch and read/write pointer
    m_curSector = 0;
    prefetchClear();
    
    if (!blocking)
    {
        return true;
    }
    else
    {
        uint32_t start = SDIO_TIME_US();
        while (SDIO_ELAPSED_US(start) < SDIO_TRANSFER_TIMEOUT_US)
        {
            if (isBusy()) continue;

            int state = (status() >> 9) & 0x0F;
            if (state != 5)
            {
                // Not busy and out of data state
                return true;
            }
        }
        
        SDIO_ERRMSG("SdioCard::stopTransmission() timeout", 0, 0);
        return false;
    }
}

bool SdioCard::syncDevice()
{
    return stopTransmission(true);
}

uint8_t SdioCard::type() const
{
    if (g_sdio_ocr & (1 << 30))
        return SD_CARD_TYPE_SDHC;
    else
        return SD_CARD_TYPE_SD2;
}

bool SdioCard::writeStart(uint32_t sector)
{
    if (m_curState == WRITE_STATE && m_curSector == sector)
    {
        SDIO_DBGMSG("Continuing multi-sector write", sector, 0);
        return true;
    }

    if (m_curState != IDLE_STATE)
    {
        stopTransmission(true);
    }

    // Cards up to 2GB use byte addressing, SDHC cards use sector addressing
    uint32_t address = (type() == SD_CARD_TYPE_SDHC) ? sector : (sector * 512);

    // Send the read command and then stop clock before first block
    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD25, address, &reply, SDIO_FLAG_STOP_CLK))) // WRITE_MULTIPLE_BLOCK
    {
        stopTransmission(false);
        return false;
    }

    m_curState = WRITE_STATE;
    m_curSector = sector;

    return true;
}

bool SdioCard::writeData(const uint8_t* src)
{
    if (m_curState != WRITE_STATE)
    {
        SDIO_ERRMSG("writeData() called when not in WRITE_STATE", (int)m_curState, src);
        return false;
    }

    if (!checkReturnOk(rp2350_sdio_tx_start(src, 1, SDIO_BLOCK_SIZE)))
    {
        SDIO_ERRMSG("rp2350_sdio_tx_start failed", g_sdio_error, 0);
        return false;
    }

    sd_callback_t callback = get_stream_callback(src, SDIO_BLOCK_SIZE, false, m_curSector);

    do {
        uint32_t blocks_done;
        g_sdio_error = rp2350_sdio_tx_poll(&blocks_done);

        if (callback)
        {
            callback(g_sd_callback.bytes_start + SDIO_BLOCK_SIZE * blocks_done);
        }
    } while (g_sdio_error == SDIO_BUSY);

    if (g_sdio_error != SDIO_OK)
    {
        SDIO_ERRMSG("SdioCard::writeData failed", g_sdio_error, m_curSector);
        return false;
    }

    m_curSector++;
    return true;
}

bool SdioCard::writeStop()
{
    return stopTransmission(true);
}

bool SdioCard::erase(uint32_t firstSector, uint32_t lastSector)
{
    SDIO_ERRMSG("SdioCard::erase() not implemented", 0, 0);
    return false;
}

bool SdioCard::cardCMD6(uint32_t arg, uint8_t* status) {
    SDIO_DBGMSG("cardCMD6", arg, 0);

    if (m_curState != IDLE_STATE)
    {
        stopTransmission(true);
    }

    uint32_t reply;
    if (!checkReturnOk(rp2350_sdio_command_u32(CMD6, arg, &reply, SDIO_FLAG_STOP_CLK))) // SWITCH_FUNCTION
    {
        SDIO_ERRMSG("CMD6 failed", arg, reply);
        rp2350_sdio_stop();
        return false;
    }

    rp2350_sdio_rx_start(status, 1, 64);

    do {
        g_sdio_error = rp2350_sdio_rx_poll(NULL);
    } while (g_sdio_error == SDIO_BUSY);

    rp2350_sdio_stop();
    SDIO_WAIT_US(1000); // Wait for function switch to complete

    SDIO_DBGMSG("CMD6 response status", reply, status[16]);
    uint32_t new_group1_mode = g_sdio_tmp_buf[16] & 0x0F;

    if (g_sdio_error != SDIO_OK)
    {
        SDIO_ERRMSG("CMD6 response read failure", arg, g_sdio_error);
        return false;
    }
    else if (reply & 0x80)
    {
        SDIO_ERRMSG("CMD6 reports CARD_STATUS_SWITCH_ERROR", arg, reply);
        return false;
    }
    else
    {
        SDIO_DBGMSG("CMD6 switch success", arg, reply);
        return true;
    }
}

bool SdioCard::readSCR(scr_t* scr) {
    SDIO_ERRMSG("SdioCard::readSCR() not implemented", 0, 0);
    return false;
}

/* Writing and reading, with progress callback */

bool SdioCard::writeSector(uint32_t sector, const uint8_t* src)
{
    if (((uint32_t)src & 3) != 0)
    {
        // Buffer is not aligned, need to memcpy() the data to a temporary buffer.
        memcpy(g_sdio_tmp_buf, src, sizeof(g_sdio_tmp_buf));
        src = (uint8_t*)g_sdio_tmp_buf;
    }

    SDIO_DBGMSG("writeSector", sector, src);

    if (!writeStart(sector) || !writeData(src))
    {
        stopTransmission(true);

        // Retry with the write_single_sector code, which is more fault-tolerant
        bool success = false;
        for (int i = 0; i < SDIO_MAX_RETRYCOUNT; i++)
        {
            SDIO_DBGMSG("Retrying write_single_sector", sector, i);
            if (write_single_sector(this, sector, src))
            {
                success = true;
                break;
            }
        }

        if (!success)
        {
            SDIO_ERRMSG("SdioCard::writeSector failed", sector, 0);
            return false;
        }
    }

    return true;
}

bool SdioCard::writeSectors(uint32_t sector, const uint8_t* src, size_t n)
{
    while (n > SDIO_MAX_BLOCKS_PER_REQ)
    {
        // Split large requests
        if (!writeSectors(sector, src, SDIO_MAX_BLOCKS_PER_REQ)) return false;
        n -= SDIO_MAX_BLOCKS_PER_REQ;
        src += SDIO_MAX_BLOCKS_PER_REQ * SDIO_BLOCK_SIZE;
    }

    if (n > 1 && ((uint32_t)src & 3) == 0 && sector + n < g_sdio_sector_count)
    {
        SDIO_DBGMSG("writeSectors multi-block", sector, n);

        // This is same as writeData() except we issue multi-sector transfer
        // to the lower layer.
        if (writeStart(sector) &&
            checkReturnOk(rp2350_sdio_tx_start(src, n, SDIO_BLOCK_SIZE)))
        {
            sd_callback_t callback = get_stream_callback(src, SDIO_BLOCK_SIZE * n, false, sector);

            do {
                uint32_t blocks_done;
                g_sdio_error = rp2350_sdio_tx_poll(&blocks_done);

                if (callback)
                {
                    callback(g_sd_callback.bytes_start + SDIO_BLOCK_SIZE * blocks_done);
                }
            } while (g_sdio_error == SDIO_BUSY);

            if (g_sdio_error == SDIO_OK)
            {
                m_curSector += n;
                return true;
            }
            else
            {
                SDIO_ERRMSG("writeSectors multi-block failed", sector, g_sdio_error);
                stopTransmission(true);
                // Fall through to retry sector-by-sector
            }
        }
    }

    // Unaligned read or end-of-drive read, execute sector-by-sector
    // Or if the multi-block read failed for some reason.
    SDIO_DBGMSG("writeSectors single block", sector, 0);

    // Unaligned write, execute sector-by-sector
    for (size_t i = 0; i < n; i++)
    {
        if (!writeSector(sector + i, src + 512 * i))
        {
            return false;
        }
    }
    return true;
}

bool SdioCard::readSector(uint32_t sector, uint8_t* dst)
{
    uint8_t *real_dst = dst;
    if (((uint32_t)dst & 3) != 0)
    {
        // Buffer is not aligned, need to memcpy() the data from a temporary buffer.
        dst = (uint8_t*)g_sdio_tmp_buf;
    }

    SDIO_DBGMSG("readSector", sector, dst);

    if (!readStart(sector) || !readData(dst))
    {
        stopTransmission(true);

        // Retry with the read_single_sector code, which is more fault-tolerant
        bool success = false;
        for (int i = 0; i < SDIO_MAX_RETRYCOUNT; i++)
        {
            SDIO_DBGMSG("Retrying read_single_sector", sector, i);
            if (read_single_sector(this, sector, dst))
            {
                success = true;
                break;
            }
        }
        
        if (!success)
        {
            SDIO_ERRMSG("SdioCard::readSector failed", sector, 0);
            return false;
        }
    }

    m_curSector = prefetchStart(this, m_curSector);

    if (dst != real_dst)
    {
        memcpy(real_dst, g_sdio_tmp_buf, sizeof(g_sdio_tmp_buf));
    }

    return true;
}

bool SdioCard::readSectors(uint32_t sector, uint8_t* dst, size_t n)
{
    while (n > SDIO_MAX_BLOCKS_PER_REQ)
    {
        // Split large requests
        if (!readSectors(sector, dst, SDIO_MAX_BLOCKS_PER_REQ)) return false;
        n -= SDIO_MAX_BLOCKS_PER_REQ;
        dst += SDIO_MAX_BLOCKS_PER_REQ * SDIO_BLOCK_SIZE;
    }

    if (n > 1 && ((uint32_t)dst & 3) == 0 && sector + n < g_sdio_sector_count && readStart(sector))
    {
        SDIO_DBGMSG("readSectors multi-block", sector, n);

        while (!prefetchIsEmpty() && n > 0)
        {
            sd_callback_t callback = get_stream_callback(dst, SDIO_BLOCK_SIZE, true, sector);
            prefetchRead(this, dst);

            if (callback)
            {
                callback(g_sd_callback.bytes_start + SDIO_BLOCK_SIZE);
            }

            dst += SDIO_BLOCK_SIZE;
            n -= 1;
            sector++;

            while (prefetchIsEmpty() && prefetchBusy(this))
            {
                prefetchProcess(this);
            }
        }

        if (n == 0)
        {
            SDIO_DBGMSG("readSectors satisfied from prefetch", 0, 0);
            m_curSector = prefetchStart(this, m_curSector);
            return true;
        }

        SDIO_DBGMSG("readSectors start receiving", sector, n);

        // This is same as readData() except we issue multi-sector transfer
        // to the lower layer.
        if (checkReturnOk(rp2350_sdio_rx_start(dst, n, SDIO_BLOCK_SIZE)))
        {
            sd_callback_t callback = get_stream_callback(dst, SDIO_BLOCK_SIZE * n, true, sector);

            do {
                uint32_t blocks_done;
                g_sdio_error = rp2350_sdio_rx_poll(&blocks_done);
        
                if (callback)
                {
                    callback(g_sd_callback.bytes_start + SDIO_BLOCK_SIZE * blocks_done);
                }
            } while (g_sdio_error == SDIO_BUSY);

            if (g_sdio_error == SDIO_OK)
            {
                m_curSector += n;
                m_curSector = prefetchStart(this, m_curSector);
                return true;
            }
            else
            {
                SDIO_ERRMSG("readSectors multi-block failed", sector, g_sdio_error);
                stopTransmission(true);
                // Fall through to retry sector-by-sector
            }
        }
    }

    // Unaligned read or end-of-drive read, execute sector-by-sector
    // Or if the multi-block read failed for some reason.
    SDIO_DBGMSG("readSectors single block", sector, 0);
    
    for (size_t i = 0; i < n; i++)
    {
        if (!readSector(sector + i, dst + 512 * i))
        {
            return false;
        }
    }

    return true;
}

#endif /* SDIO_USE_SDFAT */
