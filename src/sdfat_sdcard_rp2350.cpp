/** 
 * ZuluSCSI™ - Copyright (c) 2022-2025 Rabbit Hole Computing™
 * Copyright (c) 2024 Tech by Androda, LLC
 * 
 * ZuluSCSI™ firmware is licensed under the GPL version 3 or any later version. 
 * 
 * https://www.gnu.org/licenses/gpl-3.0.html
 * ----
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

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

#define checkReturnOk(call) ((g_sdio_error = (call)) == SDIO_OK ? true : logSDError(__LINE__))
static bool logSDError(int line)
{
    g_sdio_error_line = line;
    SDIO_ERRMSG("SDIO error on line", line, g_sdio_error);
    return false;
}

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
        status = rp2350_sdio_command_u32(CMD8, 0x1AA, &reply, SDIO_FLAG_NO_LOGMSG); // SEND_IF_COND
        
        if (status == SDIO_OK && reply == 0x1AA)
        {
            break;
        }
    }

    if (reply != 0x1AA || status != SDIO_OK)
    {
        SDIO_DBGMSG("No response to CMD8 SEND_IF_COND", status, reply);
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

bool SdioCard::readStart(uint32_t sector)
{
    if (m_curState == READ_STATE && m_curSector == sector)
    {
        SDIO_DBGMSG("Continuing multi-sector read", sector, 0);
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

    if (dst != real_dst)
    {
        memcpy(real_dst, g_sdio_tmp_buf, sizeof(g_sdio_tmp_buf));
    }

    return true;
}

bool SdioCard::readSectors(uint32_t sector, uint8_t* dst, size_t n)
{
    if (n > 1 && ((uint32_t)dst & 3) == 0 && sector + n < g_sdio_sector_count)
    {
        SDIO_DBGMSG("readSectors multi-block", sector, n);

        // This is same as readData() except we issue multi-sector transfer
        // to the lower layer.
        if (readStart(sector) &&
            checkReturnOk(rp2350_sdio_rx_start(dst, n, SDIO_BLOCK_SIZE)))
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
