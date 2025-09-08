/*
 * Copyright (C) 2015  University of Alberta
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * @file nand_m79a_lld.c
 * @author Tharun Suresh
 * @date 2021-12-29
 *
 * @brief Low level NAND Driver Layer
 *
 * This layer contains Low-level driver functions for reading and writing
 * to M79a NAND Flash via SPI.
 */

#include <stdint.h>
#include "core/nand_m79a_lld.h"

static NAND_ReturnType __Status_Reg_2_ReturnType(uint8_t status_reg);

/**
 * @brief Initializes the NAND. Steps: Reset device and check for correct device IDs.
 * @note  This function must be called first when powered on.
 *
 * @param[in] None
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Init(void) {
    return Ret_Success;
}

/******************************************************************************
 *                              Status Operations
 *****************************************************************************/

/**
 * @brief Sends command to reset the NAND Flash chip.
 * @note Transaction length: 1 byte; Returns success when Flash is ready for further instructions.
 *
 * @param[in] None
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Reset(void) {
#if 0
    uint8_t command = SPI_NAND_RESET;
    SPI_Params transmit = {.buffer = &command, .length = 1};

    NAND_SPI_ReturnType SPI_Status = NAND_SPI_Send(&transmit);
    NAND_Wait(T_POR); // wait for T_POR = 1.25 ms after reset
#else
    NAND_SPI_ReturnType SPI_Status = SPI_OK;
#endif

    if (SPI_Status != SPI_OK) {
        return Ret_ResetFailed;
    } else {
        // wait until OIP bit resets again (Flash is ready for further instructions)
        return NAND_Wait_Until_Ready();
    }
}

/**
 * @brief Waits until device is ready for further instructions
 * @note Waits until OIP bit in the Status Register resets again, indicating
 *  that the NAND Flash is ready for further instructions. If OIP = 1,
 *  operation is ongoing, i.e. device is busy.
 *
 *  Assumption: the device keeps outputting the status register contents
 *  until another command is issued. This is shown in pages 17 and 31.
 *  TODO: confirm with an oscilloscope.
 *
 * @param[in] None
 * @return NAND_ReturnType
 */

#define MAX_ATTEMPTS 10000

NAND_ReturnType NAND_Wait_Until_Ready(void) {
#if 0
    uint8_t timeout_counter = 0;

    /* SPI Transaction set up for NAND_SPI_Receive */
    uint8_t data_rx = 0;

    NAND_ReturnType ret;
    /* if busy, keep polling for until reaching max_attempts. if still busy, return busy */
    do {
        NAND_Get_Features(SPI_NAND_STATUS_REG_ADDR, &data_rx);
        ret = __Status_Reg_2_ReturnType(data_rx);
        timeout_counter += 1;
    } while (ret == Ret_NANDBusy);
    return ret;
#endif
    return Ret_Success;
}


/******************************************************************************
 *                              Feature Operations
 *****************************************************************************/

/**
 * @brief Returns Ret_NANDBusy if there are any operations in progress.
 * @note
 *      Sends command to read status register bit 1 (OIP).
 *      Transaction length: 3 bytes (2 to transmit, 1 to receive)
 *
 * @param[in] None
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Check_Busy(void) {
#if 0
    uint8_t status_reg;

    NAND_Get_Features(SPI_NAND_STATUS_REG_ADDR, &status_reg);
    if (CHECK_OIP(status_reg)) { // if OIP bit is set
        return Ret_NANDBusy;
    } else {
        return Ret_Success;
    }
#else
    return Ret_Success;
#endif
}

/******************************************************************************
 *                              Read Operations
 *****************************************************************************/

NAND_ReturnType NAND_Cache_Read(uint16_t col, uint16_t length, uint8_t *buffer) {
#if 0
    /* Command 3: READ FROM CACHE. See datasheet page 18 for details */
    uint8_t command_cache_read[4] = {SPI_NAND_READ_CACHE_X1, BYTE_1(col), BYTE_0(col), DUMMY_BYTE};

    SPI_Params tx_cache_read = {.buffer = command_cache_read, .length = 4};
    SPI_Params rx_cache_read = {.buffer = buffer, .length = length};
    // TODO: Check if we can read just 2048 data bits per page. Datasheet shows 2176 bytes including the spare
    // locations.

    if (NAND_SPI_SendReceive(&tx_cache_read, &rx_cache_read) != SPI_OK) {
        return Ret_ReadFailed;
    }
#endif
    return Ret_Success;
}

NAND_ReturnType NAND_Page_Load(uint32_t paddr) {
#if 0
    /* PAGE READ. See datasheet page 16 for details */
    uint8_t command_page_read[4] = {SPI_NAND_PAGE_READ, BYTE_2(paddr), BYTE_1(paddr), BYTE_0(paddr)};

    SPI_Params tx_page_read = {.buffer = command_page_read, .length = 4};

    if (NAND_SPI_Send(&tx_page_read) != SPI_OK) {
        return Ret_ReadFailed;
    }
#endif
    /* Command 2: Wait for data to be loaded into cache */
    return NAND_Wait_Until_Ready();
}

/**
 * @brief Read bytes stored in a page.
 * @note Command sequence:
 *          1) Send page read command to read data from page to cache
 *          2) Wait until OIP bit resets in status register
 *          3) Read data from cache
 *
 * @param addr[in]      Pointer to PhysicalAddrs struct
 * @param length[in]    Number of bytes to read
 * @param buffer[out]   Pointer to contents read from page
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Page_Read(PhysicalAddrs *addr, uint16_t length, uint8_t *buffer) {
    NAND_ReturnType status;

    if (length > PAGE_DATA_SIZE) {
        return Ret_ReadFailed;
    }
    uint32_t plane = addr->block & 1;

    /* Command 1: PAGE READ. See datasheet page 16 for details */
    uint32_t row = 0;
    row = (0x7ff & addr->block) << 6;
    row |= (0x3f & addr->page);

    if ((status = NAND_Page_Load(row)) != Ret_Success) {
        return status;
    }

    /* Command 3: READ FROM CACHE. See datasheet page 18 for details */
    uint32_t col = addr->column | (plane << 12);
    return NAND_Cache_Read(col, length, buffer);
}

/******************************************************************************
 *                              Write Operations
 *****************************************************************************/

/* TODO:
 * The first spare area location in each bad block contains the bad-block mark (0x00).
 * System software should initially check the first spare area location (byte 2048) for non-FFh data on
 * the first page of each block prior to performing any program or erase operations on the
 * NAND Flash device.
 */
/**
 * @brief Write data to a page.
 * @note Command sequence:
 *          1) WRITE ENABLE
 *          2) PROGRAM LOAD : load data into cache register
 *          3) PROGRAM EXECUTE : transfers data from cache to main array and waits until OIP bit is cleared
 *          4) WRITE DISABLE
 *
 * @param addr[in]      Pointer to PhysicalAddrs struct
 * @param length[in]    Number of bytes to write
 * @param buffer[in]    Pointer to contents read from page
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Page_Program(PhysicalAddrs *addr, uint16_t length, uint8_t *buffer) {
    if (length > PAGE_DATA_SIZE) {
        return Ret_WriteFailed;
    }

    /* Command 1: WRITE ENABLE */
    __write_enable();

    /* Command 2: PROGRAM LOAD. See datasheet page 30 for details */
    uint32_t plane = addr->block & 1;

    uint32_t col = addr->column | (plane << 12);
    uint8_t command_load[3] = {SPI_NAND_PROGRAM_LOAD_X1, BYTE_1(col), BYTE_0(col)};

#if 0
    SPI_Params tx_cmd = {.buffer = command_load, .length = 3};
    SPI_Params tx_data = {.buffer = buffer, .length = length};

    // TODO: Check if we can write just 2048 data bits per page. Datasheet shows 2176 bytes including the spare
    // locations.

    if (NAND_SPI_Send_Command_Data(&tx_cmd, &tx_data) != SPI_OK) {
        return Ret_WriteFailed;
    }
#endif

    /* Command 3: PROGRAM EXECUTE. See datasheet page 31 for details */
    uint32_t row = 0;
    row = (0x7ff & addr->block) << 6;
    row |= (0x3f & addr->page);

#if 0
    uint8_t command_exec[4] = {SPI_NAND_PROGRAM_EXEC, BYTE_2(row), BYTE_1(row), BYTE_0(row)};

    SPI_Params exec_cmd = {.buffer = command_exec, .length = 4};

    if (NAND_SPI_Send(&exec_cmd) != SPI_OK) {
        return Ret_Failed;
    }
#endif

    /* Command 3: wait for device to be ready again */
    NAND_ReturnType status = NAND_Wait_Until_Ready();

    /* Command 4: WRITE DISABLE */
    __write_disable();
    if (status != Ret_Success) {
        NAND_Reset();
    }
    return status;
}

/******************************************************************************
 *                              Erase Operations
 *****************************************************************************/

/* TODO:
 * The first spare area location in each bad block contains the bad-block mark (0x00).
 * System software should initially check the first spare area location (byte 2048) for non-FFh data on
 * the first page of each block prior to performing any program or erase operations on the
 * NAND Flash device.
 *
 */

/**
 * @brief Erases an entire block (136 KB) at a time.
 * @note Command sequence:
 *          1) WRITE ENABLE
 *          2) BLOCK ERASE
 *          3) Wait for OIP bit to clear
 *          4) WRITE DISABLE
 *
 * @param addr[in]  Pointer to struct with block number
 * @return NAND_ReturnType
 */
NAND_ReturnType NAND_Block_Erase(PhysicalAddrs *addr) {

    /* Command 1: WRITE ENABLE */
    __write_enable();
    NAND_ReturnType status = NAND_Wait_Until_Ready();
    if (status != Ret_Success) {
        return Ret_Failed;
    }
    /* Command 2: BLOCK ERASE. See datasheet page 35 for details */

    // TODO: datasheet simply specifies block address. assuming that's the 11-bit
    // block number padded with dummy bits. check.
    uint32_t block = (0x7ff & addr->block) << 6;

#if 0
    uint8_t command[4] = {SPI_NAND_BLOCK_ERASE, BYTE_2(block), BYTE_1(block), BYTE_0(block)};

    SPI_Params tx_cmd = {.buffer = command, .length = 4};
    if (NAND_SPI_Send(&tx_cmd) != SPI_OK) {
        return Ret_Failed; // I/O failure
    }
#endif

    /* Command 3: wait for device to be ready again */
    status = NAND_Wait_Until_Ready();
    if (status != Ret_Success) {
        NAND_Reset();
    }
    /* Command 4: WRITE DISABLE */
    __write_disable();
    return status;
}

/*
 * False: Block is good
 * True: Block is bad
 */
bool NAND_is_Bad_Block(int block) {
    PhysicalAddrs addr = {0};
    addr.block = block;
    addr.column = 2048;
    uint8_t marker = 0xFF;
    NAND_Page_Read(&addr, sizeof(marker), &marker);
    return marker != 0xFF;
}

NAND_ReturnType NAND_Mark_Bad_Block(int block) {
    PhysicalAddrs addr = {0};
    addr.block = block;
    addr.column = 2048;
    uint8_t marker = 0x00;
    // It seems that sometimes we can write a couple bits to a bad block
    // Really all the marker needs to be is not 0xFF
    // The datasheet seems confident that this will work
    return NAND_Page_Program(&addr, sizeof(marker), &marker);
}

/**
 * THIS WILL ERASE THE BAD BLOCK TABLE. Probably a bad idea, but it's your funeral
 */
NAND_ReturnType NAND_Erase_All() {
    PhysicalAddrs addr = {0};
    for (int i = 0; i < NUM_BLOCKS; i++) {
        addr.block = i;
        NAND_Block_Erase(&addr);
    }
    return Ret_Success;
}

/******************************************************************************
 *                              Move Operations
 *****************************************************************************/

NAND_ReturnType NAND_Copy_Block(PhysicalAddrs *src, PhysicalAddrs *dst) { return Ret_Success; }

/******************************************************************************
 *                              Lock Operations
 *****************************************************************************/

/******************************************************************************
 *                              Internal Functions
 *****************************************************************************/

static NAND_ReturnType __Status_Reg_2_ReturnType(uint8_t status_reg) {
    // DBG_PUT("0x%x\r\n", status_reg);
    //  Check all the non-ecc statuses
    if (status_reg & NAND_CRBSY) {
        return Ret_NANDBusy;
    }
    if (status_reg & NAND_PF) {
        return Ret_WriteFailed;
    }
    if (status_reg & NAND_EF) {
        return Ret_EraseFailed;
    }
    if (status_reg & NAND_OIP) {
        return Ret_NANDBusy;
    }
    return Ret_Success;
}

/**
 * @brief Enable writing to NAND.
 *
 * @param[in] None
 * @return NAND_SPI_ReturnType
 */
NAND_SPI_ReturnType __write_enable(void) {
#if 0
    uint8_t command = SPI_NAND_WRITE_ENABLE;
    SPI_Params transmit = {.buffer = &command, .length = 1};
    return NAND_SPI_Send(&transmit);
#else
    return SPI_OK;
#endif
}

/**
 * @brief Disable writing to NAND
 *
 * @param[in] None
 * @return NAND_SPI_ReturnType
 */
NAND_SPI_ReturnType __write_disable(void) {
#if 0
    uint8_t command = SPI_NAND_WRITE_DISABLE;
    SPI_Params transmit = {.buffer = &command, .length = 1};
    return NAND_SPI_Send(&transmit);
#else
    return SPI_OK;
#endif
}
