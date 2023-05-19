/*
 * sst26_flash.c
 *
 *  Created on: Mar 17, 2023
 *      Author: Fuguru
 */

#include <sst26_flash.h>

#ifdef DEBUG_FLAG
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <utils/uartstdio.h>
#endif

#define FLASH_SPI 3
#define ERR_NO_ERROR 0
#define ERR_INIT_FAIL 1
#define ERR_FAIL 2
#define NULL 0


uint32_t localWrite(uint32_t in_addr, uint8_t* buf, uint32_t in_length, uint8_t wait)
{
    uint32_t rtn;

    if ((in_addr + in_length) > SST_26VF032B_SIZE) {
        return ERR_FAIL;
    }

    rtn = SST_writeBuffer(in_addr, buf, in_length);

    return rtn;
}

uint32_t localErase(uint32_t in_addr, uint32_t in_length, uint8_t wait)
{
    uint32_t rtn;

    if ((in_addr + in_length) > SST_26VF032B_SIZE)
        return ERR_FAIL;

    SST_waitFlashReady();

    rtn = SST_sendCmdBlockErase(in_addr, in_length);
//    rtn = SST_sendCmdSectorErase(in_addr, in_length);

    if (rtn != 0) {return rtn;}

    if (wait)
    {
        SST_waitFlashReady();
    }

    return rtn;
}

uint32_t localRead(uint32_t in_addr, uint8_t* buf, uint32_t in_length, uint8_t wait)
{
    uint32_t rtn;

    if ((in_addr + in_length) > SST_26VF032B_SIZE) {
        return ERR_FAIL;
    }

    SST_waitFlashReady();

    // SST_26VF032B_SIZE-1 = 0x3FFFFF
    uint8_t spi_cmd_data[4];
    spi_cmd_data[0] = READ;
    spi_cmd_data[1] = (( in_addr & 0xFF0000) >> 16);
    spi_cmd_data[2] = (( in_addr & 0x00FF00) >> 8);
    spi_cmd_data[3] =  ( in_addr & 0x0000FF);
    // transmit the command, keeping chip select active so the data can be
    // clocked in immediately after command.
    rtn = SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 4, SPI_HOLD_ACTIVE);
    rtn |= SPI_xfer(FLASH_SPI, 0, buf, in_length, SPI_HOLD_CLR);

    return rtn;
}

//-----------------------------------------------------------------------------
// Private Defines and Macros
//-----------------------------------------------------------------------------

// sst status register defines.
#define STATUS_REGISTER_BUSY    (0x81)  // 0b10000001 Checks both BUSY flags

// sst mfr and device ids.
// JWM updated
#define SST_MFR_ID      (0xBF)
#define SST_MEM_TYPE    (0x26)
#define SST_DEV_ID      (0x42)

#define DUMMY_BYTE      (0x00)


//-----------------------------------------------------------------------------
// Private Function Prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Public Function Definitions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Clear the flash status register to remove the write protection
//-----------------------------------------------------------------------------

/// JWM: Updated
//-----------------------------------------------------------------------------
// \brief   clear the flash status register to remove the write protection.
//
//
//
// \param   spi_cs_hold_e in_cs_hold
//             SPI_HOLD_NONE - do nothing with CSHOLD of SPIDAT1.
//             SPI_HOLD_ACTIVE - keep cs active after completing transfer.
//             SPI_HOLD_CLR - hold cs until the last byte, then clear.
//
// \neturn  uint32_t
//    ERR_NO_ERROR - input in bounds, byte transmitted.
//    ERR_INVALID_PARAMETER - null pointers.

uint32_t SST_enableFlashWrite(void)
{
    uint32_t rtn = ERR_NO_ERROR;
    uint8_t spi_data[1];  // 1 byte

    // send the write enable command
    spi_data[0] = WREN;
    rtn = SPI_xfer(FLASH_SPI, spi_data, NULL, 1, SPI_HOLD_CLR);
    if (rtn != ERR_NO_ERROR) {
        return (rtn);
    }

    // confirm that the write enable command (0x06) was output
    #ifdef DEBUG_FLAG
    UARTprintf("spi_data write enable tx:\t%02X\n", spi_data[0]);
    #endif

    // Executes global block protection unlock
    spi_data[0] = ULBPR;
    rtn = SPI_xfer(FLASH_SPI, spi_data, NULL, 1, SPI_HOLD_CLR);
    if (rtn != ERR_NO_ERROR) {
        return (rtn);
    }

    // confirm correct message was sent 0x98)
    #ifdef DEBUG_FLAG
    UARTprintf("spi_data write status reg tx:\t%02X\n", spi_data[0]);
    #endif

    // send the read status reg command, poll BUSY (bit 0 or 7) 1=write in progress
    SST_waitFlashReady();

    // confirm the status register was written to
    #ifdef DEBUG_FLAG
    UARTprintf("spi_data rx:\t%02X  %02X\n", spi_data[0], spi_data[1]);
    #endif

    #ifdef DEBUG_FLAG
    UARTprintf("spi flash write protection disabled!\n");
    #endif
    return (ERR_NO_ERROR);
}

//-----------------------------------------------------------------------------
// verify device id.
// SST Manufacture's ID = 0xBF, Memory Type = 0x25, and Device ID  = 0x4A
/// JWM: Updated
//-----------------------------------------------------------------------------
uint32_t SST_verifyDeviceID(void)
{
   uint32_t rtn;
   uint8_t spi_data[4];

   spi_data[0] = JEDEC_ID;
   spi_data[1] = DUMMY_BYTE;
   spi_data[2] = DUMMY_BYTE;
   spi_data[3] = DUMMY_BYTE;

   #ifdef DEBUG_FLAG
       UARTprintf("spi_data tx:\t%02X %02X %02X %02X\n", spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
   #endif

   rtn = SPI_xfer(FLASH_SPI, spi_data, spi_data, 4, SPI_HOLD_CLR);

   if (rtn != ERR_NO_ERROR)
   {
      #ifdef DEBUG_FLAG
          UARTprintf("send read id cmd error:\t%d\n", rtn);
      #endif
      return (ERR_FAIL);
   }

   // verify device id.
   #ifdef DEBUG_FLAG
       UARTprintf("spi_data rx:\t%02X %02X %02X %02X\n", spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
   #endif

   if ((spi_data[1] != SST_MFR_ID) ||
       (spi_data[2] != SST_MEM_TYPE) ||
       (spi_data[3] != SST_DEV_ID))
   {
      #ifdef DEBUG_FLAG
      UARTprintf("spi flash id does not match expected id!\n");
      #endif
      return (ERR_FAIL);
   }

   return (ERR_NO_ERROR);
}

//-----------------------------------------------------------------------------
// Send Write Enable Command
/// JWM: No change needed
//-----------------------------------------------------------------------------
uint32_t SST_writeEnable()
{
    uint32_t rtn = ERR_NO_ERROR;
    uint8_t spi_data[1];

    spi_data[0] = WREN;
    rtn = SPI_xfer(FLASH_SPI, spi_data, NULL, 1, SPI_HOLD_CLR);
    return rtn;
}

//-----------------------------------------------------------------------------
// Send Write Disable Command
// JWM: No change needed
//-----------------------------------------------------------------------------
uint32_t SST_writeDisable()
{
    uint32_t rtn;
    uint8_t spi_data[1];

    // send the write disable command
    spi_data[0] = WRDI;
    rtn = SPI_xfer(FLASH_SPI, spi_data, NULL, 1, SPI_HOLD_CLR);
    return (rtn);
}

//-----------------------------------------------------------------------------
// Send Read Status Register
//-----------------------------------------------------------------------------
uint8_t SST_sendCmdReadStatus()
{
    uint8_t spi_data[2];
    spi_data[0] = RDSR;
    spi_data[1] = 0;
    SPI_xfer(FLASH_SPI, spi_data, spi_data, 2, SPI_HOLD_CLR);

    return spi_data[1];
}

//-----------------------------------------------------------------------------
// Send Read Configuration Register
//-----------------------------------------------------------------------------
uint8_t SST_sendCmdReadConfig()
{
    uint8_t spi_data[2];
    spi_data[0] = RDCR;
    spi_data[1] = 0;
    SPI_xfer(FLASH_SPI, spi_data, spi_data, 2, SPI_HOLD_CLR);

#ifdef DEBUG_FLAG
    UARTprintf("configuration register:\t%02X\n\tIOC: %d\n\tBPNV: %d\n\tWPEN: %d\n",
               spi_data[1], spi_data[1]&0x02, spi_data[1]&0x08, spi_data[1]&0x80);
#endif

    return spi_data[1];
}
//-----------------------------------------------------------------------------
// wait until spi flash is not busy.
// JWM: Updated
//-----------------------------------------------------------------------------
uint8_t SST_waitFlashReady()
{
    uint8_t u8_status = STATUS_REGISTER_BUSY;
    
    u8_status = SST_sendCmdReadStatus();
    while (u8_status & STATUS_REGISTER_BUSY)
    {
        // send the read status reg command and wait until device is ready
        u8_status = SST_sendCmdReadStatus();
    }
    return u8_status;
}


//-----------------------------------------------------------------------------
// send the read command and address bytes.
// JWM: Updated
//-----------------------------------------------------------------------------
uint32_t SST_sendCmdRead(uint32_t in_addr)
{
    //sst_address_t sst_address;
    uint8_t spi_cmd_data[4];

    // SST_26VF032B_SIZE-1 = 0x3FFFFF
    spi_cmd_data[0] = READ;
    spi_cmd_data[1] = (( in_addr & 0xFF0000) >> 16);
    spi_cmd_data[2] = (( in_addr & 0x00FF00) >> 8);
    spi_cmd_data[3] =  ( in_addr & 0x0000FF);

    // transmit the command, keeping chip select active so the data can be
    // clocked in immediately after command.
    return SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 4, SPI_HOLD_ACTIVE);
}


//-----------------------------------------------------------------------------
// send the block erase command and address bytes.
// JWM: Need to update
//-----------------------------------------------------------------------------
uint32_t SST_sendCmdBlockErase(uint32_t in_addr, uint32_t in_length)
{
    uint32_t rtn;
    // Confirm in_addr and in_length is a multiple of 64KB after first 64KB
    if (in_addr >= SPIFLASH_64K_BLOCK_SIZE 
        && !((in_addr % SPIFLASH_64K_BLOCK_SIZE)==0)
        && !((in_length % SPIFLASH_64K_BLOCK_SIZE)==0))
    {
        return ERR_FAIL;
    }

    // sst_address_t sst_address;
    uint8_t spi_cmd_data[4];

    // Increment address based on memory map, assumes address is a multiple of 64KB
    uint32_t inc = 0;
    if (in_addr == SST26VF_8K_BLOCK_TOP_START) {
        // Address is 0
        inc = SPIFLASH_8K_BLOCK_SIZE;
    } else if (in_addr < SST26VF_32K_BLOCK_BOT_START) {
        // Address is within one of the 62x 64KB blocks
        inc = SPIFLASH_64K_BLOCK_SIZE;
    } else {
        // Address is at the last 64KB, starts at the last 32KB block
        inc = SPIFLASH_32K_BLOCK_SIZE;
    }

    uint32_t last_addr = in_addr + in_length;
    while (in_addr < last_addr)
    {
        // Enable the flash to write the erase command
        SST_writeEnable();

        spi_cmd_data[0] = ERASE_BLOCK;
        spi_cmd_data[1] = (( in_addr & 0xFF0000) >> 16);
        spi_cmd_data[2] = (( in_addr & 0x00FF00) >> 8);
        spi_cmd_data[3] =  ( in_addr & 0x0000FF);
        rtn  = SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 4, SPI_HOLD_CLR);
        if (rtn != ERR_NO_ERROR)
        {
            return (rtn);
        }

        // make sure the spi flash is not busy.
        SST_waitFlashReady();

        in_addr += inc;

        // Increment address based on memory map
        switch(in_addr)
        {
            case SST26VF_32K_BLOCK_TOP_START:
                // We've reached the first 32KB block
                inc = SPIFLASH_32K_BLOCK_SIZE;
                break;
            case SST26VF_64K_BLOCK_TOP_START:
                // We've reached the 1st 64KB block
                inc = SPIFLASH_64K_BLOCK_SIZE;
                break;
            case SST26VF_32K_BLOCK_BOT_START:
                // We've reached the last 32KB block
                inc = SPIFLASH_32K_BLOCK_SIZE;
                break;
            case SST26VF_8K_BLOCK_BOT_START:
                // We've reached the last set of 8KB blocks
                inc = SPIFLASH_8K_BLOCK_SIZE;
                break;
        }
    }

    return (rtn);
}


//-----------------------------------------------------------------------------
// send the sector erase command and address bytes.
// JWM: Need to update
//-----------------------------------------------------------------------------
uint32_t SST_sendCmdSectorErase(uint32_t in_addr, uint32_t in_length)
{
    uint32_t rtn;
    // Confirm in_addr is a multiple of 4KB
    if (!((in_addr % SPIFLASH_SECTOR_SIZE)==0))
    {
        return ERR_FAIL;
    }
    // Confirm in_length is a multiple of 4KB
    if (!((in_length % SPIFLASH_SECTOR_SIZE)==0))
    {
        return ERR_FAIL;
    }

    // sst_address_t sst_address;
    uint8_t spi_cmd_data[4];

    // Increment address based on memory map, assumes address is a multiple of 64KB
    uint32_t inc = SPIFLASH_SECTOR_SIZE;

    // Enable the flash to write the erase command

    uint32_t last_addr = in_addr + in_length;
    while (in_addr < last_addr)
    {
        SST_writeEnable();

        spi_cmd_data[0] = ERASE_SECTOR;
        spi_cmd_data[1] = (( in_addr & 0xFF0000) >> 16);
        spi_cmd_data[2] = (( in_addr & 0x00FF00) >> 8);
        spi_cmd_data[3] =  ( in_addr & 0x0000FF);
        rtn  = SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 4, SPI_HOLD_CLR);
        if (rtn != ERR_NO_ERROR)
        {
            spi_cmd_data[0] = NOP;
            rtn  = SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 1, SPI_HOLD_CLR);
            return rtn;
        }

        // make sure the flash is not busy.
        SST_waitFlashReady();
        in_addr += inc;
    }

    spi_cmd_data[0] = NOP;
    rtn  = SPI_xfer(FLASH_SPI, spi_cmd_data, NULL, 1, SPI_HOLD_CLR);

    return rtn;
}

//-----------------------------------------------------------------------------
// send the page-write command and address bytes with data and length
//
//-----------------------------------------------------------------------------
uint32_t SST_writePage(uint32_t in_addr, uint8_t* buf, uint32_t len)
{
    uint32_t rtn;

    // Ensure the address is valid
    if ((in_addr + len) > SST_26VF032B_SIZE) {
        return ERR_FAIL;
    }

    // Ensure that the supplied data is no larger than the page size
    if (len > SPIFLASH_PAGE_SIZE) {
        return ERR_FAIL;
    }

#ifdef DEBUG_FLAG
    UARTprintf("spi write to:\t%04X\tlength:\t%d\n", in_addr, len);
#endif

    // send the write enable command
    SST_writeEnable();

    // Send page program command, 24-bit address
    uint8_t cmd_list[4];
    cmd_list[0] = PP;
    cmd_list[1] = (in_addr >> 16) & 0xFF;
    cmd_list[2] = (in_addr >> 8) & 0xFF;
    cmd_list[3] = (in_addr) & 0xFF;

    // set lower 8 bits to 0 to avoid wrapping
    if (len == SPIFLASH_PAGE_SIZE) {
        cmd_list[3] = 0;
    }

    rtn = SPI_xfer(FLASH_SPI, cmd_list, NULL, 4, SPI_HOLD_ACTIVE);
    rtn |= SPI_xfer(FLASH_SPI, buf, NULL, len, SPI_HOLD_CLR);
    SST_waitFlashReady(); // when writing 256B, delay for >1.25ms

    return (rtn);
}

//-----------------------------------------------------------------------------
// Public API to write a buffer to flash with starting address
//
//-----------------------------------------------------------------------------
uint32_t SST_writeBuffer(uint32_t in_addr, uint8_t* buf, uint32_t len)
{
    uint32_t rtn;

    // If the data is only on one page we can go fast
    if (len <= SPIFLASH_PAGE_SIZE) {
        return SST_writePage(in_addr, buf, len);
    }

    // Block spans multiple pages
    uint32_t bytes_written = 0;
    uint32_t buf_offset = 0;
    uint32_t last_addr = in_addr + len;
    while (in_addr < last_addr)
    {
        // Figure out how many bytes need to be written to this page
        uint32_t bytes_to_write = SPIFLASH_PAGE_SIZE - (in_addr % SPIFLASH_PAGE_SIZE);

        // Write the current page
        rtn = SST_writePage(in_addr, (buf + buf_offset), bytes_to_write);
        if (rtn != ERR_NO_ERROR) {
            return (rtn);
        }

        bytes_written += bytes_to_write;

        // Adjust address and len, and buffer offset
        in_addr += bytes_to_write;
        buf_offset += bytes_to_write;
    }

    return rtn;
}

uint32_t SST_sendCmdChipErase()
{
    uint32_t rtn;
    uint8_t spi_data[1];

    SST_waitFlashReady();
    SST_writeEnable();

    spi_data[0] = CHIP_ERASE;
    rtn = SPI_xfer(FLASH_SPI, spi_data, NULL, 1, SPI_HOLD_CLR);

    SST_waitFlashReady(); // check WREN flag
    return (rtn);
}

uint32_t SST_sendCmdReadSID(uint32_t in_addr, uint8_t* buf, uint32_t len)
{
    uint32_t rtn;

    // If the data is only on one page we can go fast
    if (len > 2048) {
        return ERR_FAIL;
    }

    // Send page program command, 24-bit address
    uint8_t cmd_list[4];
    cmd_list[0] = RSID;
    cmd_list[1] = (in_addr>>8)  & 0xFF;
    cmd_list[2] = (in_addr)     & 0xFF;
    cmd_list[3] = DUMMY_BYTE;

    rtn = SPI_xfer(FLASH_SPI, cmd_list, NULL, 4, SPI_HOLD_ACTIVE);
    rtn |= SPI_xfer(FLASH_SPI, NULL, buf, len, SPI_HOLD_CLR);

    return rtn;
}

uint32_t SST_sendCmdReadBlockProtectionReg(uint8_t* buf, uint32_t len)
{
    uint32_t rtn;
    if (len<1)
    {
        return ERR_FAIL;
    }
    //
    buf[0] = RBPR;
    rtn = SPI_xfer(FLASH_SPI, buf, buf, len, SPI_HOLD_CLR);
    SST_waitFlashReady(); // check WREN flag
    return (rtn);
}


uint32_t SST_sendCmdReadSFDP(uint32_t in_addr, uint8_t* buf, uint32_t len)
{
    uint32_t rtn;
    uint8_t spi_data[8];

    in_addr = 0;

    // If the data is only on one page we can go fast
    if (len > 2048) {
        return ERR_FAIL;
    }

    // Send page program command, 24-bit address
    uint8_t cmd_list[5];
    cmd_list[0] = SFDP;
    cmd_list[1] = (in_addr>>16) & 0xFF;
    cmd_list[2] = (in_addr>>8)  & 0xFF;
    cmd_list[2] = (in_addr)     & 0xFF;
    cmd_list[3] = DUMMY_BYTE;

    rtn = SPI_xfer(FLASH_SPI, cmd_list, NULL, 5, SPI_HOLD_ACTIVE);
    // Read SFDP Header
    rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 8, SPI_HOLD_ACTIVE);
    in_addr += 8; // advance read address

    // Confirm SFDP Signature=50444653H
#ifdef DEBUG_FLAG
    UARTprintf("SFDP Signature:\t%02X %02X %02X %02X\n", spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
#endif

    if ((spi_data[0] != 0x53) ||
        (spi_data[1] != 0x46) ||
        (spi_data[2] != 0x44) ||
        (spi_data[3] != 0x50))
    {
#ifdef DEBUG_FLAG
       UARTprintf("SFDP Signature didn't match expected id!\n");
#endif
       return (ERR_FAIL);
    }

#ifdef DEBUG_FLAG
    UARTprintf("\tSFDP Revision:\t%02X %02X\n", spi_data[5], spi_data[4]);
    UARTprintf("\tNPH:\t%d\n", spi_data[6]);
#endif

    // Read SFDP Parameter Header 1-2
    rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 8, SPI_HOLD_ACTIVE);
    in_addr += 8;
    uint32_t PTP1TLen = spi_data[3];
    uint32_t PTP1 = (((uint32_t)spi_data[6])<<16) + (((uint32_t)spi_data[5])<<8) + (uint32_t)spi_data[4];

    // Read SFDP Parameter Header 3-4
    rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 8, SPI_HOLD_ACTIVE);
    in_addr += 8;
    uint32_t PTP2TLen = spi_data[3];
    uint32_t PTP2 = (((uint32_t)spi_data[6])<<16) + (((uint32_t)spi_data[5])<<8) + (uint32_t)spi_data[4];

    // Read SFDP Parameter Header 5-6
    rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 8, SPI_HOLD_ACTIVE);
    in_addr += 8;
    uint32_t PTP3TLen = spi_data[3];
    uint32_t PTP3 = (((uint32_t)spi_data[6])<<16) + (((uint32_t)spi_data[5])<<8) + (uint32_t)spi_data[4];

    // Read until parameter table 1
    while (in_addr < PTP1)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
        in_addr += 4;
    }

    // Read through parameter table 1
    while (in_addr < PTP1 + PTP1TLen*4)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
#ifdef DEBUG_FLAG
        UARTprintf("P-Table 1 0x%04X:\t%02X %02X %02X %02X\n", in_addr, spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
#endif
        in_addr += 4;
    }

    // Read until parameter table 2
    while (in_addr < PTP2)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
        in_addr += 4;
    }

    // Read through parameter table 2
    while (in_addr < PTP2 + PTP2TLen*4)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
#ifdef DEBUG_FLAG
        UARTprintf("P-Table 2 0x%04X:\t%02X %02X %02X %02X\n", in_addr, spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
#endif
        in_addr += 4;
    }

    // Read until parameter table 3
    while (in_addr < PTP3)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
        in_addr += 4;
    }

    // Read through parameter table 3
    while (in_addr < PTP3 + PTP3TLen*4)
    {
        rtn |= SPI_xfer(FLASH_SPI, NULL, spi_data, 4, SPI_HOLD_ACTIVE);
#ifdef DEBUG_FLAG
        UARTprintf("P-Table 3 0x%04X:\t%02X %02X %02X %02X\n", in_addr, spi_data[0], spi_data[1], spi_data[2], spi_data[3]);
#endif
        in_addr += 4;
    }

    rtn |= SPI_xfer(FLASH_SPI, NULL, NULL, 4, SPI_HOLD_CLR);

    return rtn;
}


