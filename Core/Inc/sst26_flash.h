/*
 * sst26_flash.h
 *
 *  Created on: Mar 17, 2023
 *      Author: Fuguru
 */

#ifndef SST26_FLASH_H_
#define SST26_FLASH_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef DEBUG_FLAG
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_nvic.h>
#include <inc/hw_gpio.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/ssi.h>
#endif

typedef enum {
    SPI_HOLD_NONE = 0,
    SPI_HOLD_ACTIVE,
    SPI_HOLD_CLR
} spi_cs_hold_e;

extern uint32_t SPI_xfer(uint32_t ui32Base, uint8_t* inTxData, uint8_t* inRxData, uint32_t inLength, int8_t inFssHold);

//-----------------------------------------------------------------------------
// Public Defines and Macros
//-----------------------------------------------------------------------------
// SST26VF032B spi flash sizes in bytes
#define SST_26VF032B_SIZE           (4194304) // (33,554,432 bits)/(8 bits/byte)  =3D 0x3FFFFF

//  SST26VF032B memory org defines.
#define SPIFLASH_PAGE_SIZE          (256)       //  bytes per page
#define SPIFLASH_64K_BLOCK_SIZE     (65536)     // 64K Block Size in bytes 0x10000
#define SPIFLASH_32K_BLOCK_SIZE     (32768)     // 32K Block Size in bytes 0x08000
#define SPIFLASH_8K_BLOCK_SIZE      (8192)      //  8K Block Size in bytes 0x02000
#define SPIFLASH_CHIP_SIZE          (SST_26VF032B_SIZE) // 4,194,304 bytes
#define SPIFLASH_NUM_PAGES          (SST_26VF032B_SIZE/SPIFLASH_PAGE_SIZE) // 16384
#define SPIFLASH_SECTOR_SIZE        (4096)

// max timeouts for various commands...in microseconds.
#define TIMEOUT_POST_PAGE_WRITE         (5000)    // 5 msec - given page erased
#define TIMEOUT_POST_ERASE_PAGE_PROG    (25000)   // 25 milliseconds
#define TIMEOUT_POST_SEC_ERASE          (25000)   // 25 milliseconds
#define TIMEOUT_POST_CHIP_ERASE         (50000)   // 50 milliseconds

// max timeout for a cmd that is being used...need to change
// if chip erase commands are implemented.
#define TIMEOUT_MAX         (TIMEOUT_POST_SEC_ERASE)

// List of commands supported by the SST26VF032B Flash
#define NOP                 0x00    // No Operation
#define RDSR                0x05    // Read Status Register
#define WRSR                0x01    // Write Status Register
#define RDCR                0x35    // Read configuration register
#define READ                0x03    // Read Memory (20MHz, up to 40MHz)

#define JEDEC_ID            0x9F    // JEDEC ID Read
#define SFDP                0x5A    // Serial Flash Discoverable Parameters

#define WREN                0x06    // Write Enable
#define WRDI                0x04    // Write Disable
#define ERASE_SECTOR        0x20    // Sector Erase 4 KBytes
#define ERASE_BLOCK         0xD8    // Erase Memory Block, 64 KBytes, 32 KBytes or 8 KBytes depending on the location
#define CHIP_ERASE          0xC7    // Full chip erase
#define PP                  0x02    // Page Program
#define RBPR                0x72    // Read block protection reg 1-18
#define ULBPR               0x98    // global block Protection unlock
#define RSID                0x88    // Read Security ID
#define PSID                0xA5    // (One-Time) Program User Security ID

// Memory Map layout - Treat the first and last set of 4x8KB + 1x32KB as a 64KB block
// Top of memory
//  4x 8KB
//  1x 32KB
//  64KB
//  61 more 64KB
//  32KB
//  4x 8KB
#define SST26VF_8K_BLOCK_TOP_START        (0)             // First valid Top 8KB block address
#define SST26VF_32K_BLOCK_TOP_START       (0x008000)      // First valid Top 32KB block address
#define SST26VF_64K_BLOCK_TOP_START       (0x010000)      // First valid 64KB block address
#define SST26VF_32K_BLOCK_BOT_START       (0x3F0000)      // First valid Bottom 32KB block address
#define SST26VF_8K_BLOCK_BOT_START        (0x3F8000)      // First valid Bottom 8KB block address

//-----------------------------------------------------------------------------
// Public Function Prototypes
//-----------------------------------------------------------------------------

uint32_t SST_enableFlashWrite(void);
uint32_t SST_verifyDeviceID(void);
uint32_t SST_writeEnable();
uint32_t SST_writeDisable();
uint8_t SST_waitFlashReady();
uint8_t SST_sendCmdReadStatus();
uint8_t SST_sendCmdReadConfig();

uint32_t SST_sendCmdReadBlockProtectionReg(uint8_t* buf, uint32_t len);

uint32_t SST_sendCmdRead(uint32_t in_addr);
uint32_t SST_writePage(uint32_t in_addr, uint8_t* buf, uint32_t len);
uint32_t SST_writeBuffer(uint32_t in_addr, uint8_t* buf, uint32_t len);

uint32_t SST_sendCmdBlockErase(uint32_t in_addr, uint32_t in_length);
uint32_t SST_sendCmdSectorErase(uint32_t in_addr, uint32_t in_length);
uint32_t SST_sendCmdChipErase();
uint32_t SST_sendCmdReadSID(uint32_t in_addr, uint8_t* buf, uint32_t len);
uint32_t SST_sendCmdReadSFDP(uint32_t in_addr, uint8_t* buf, uint32_t len);

uint32_t localWrite(uint32_t in_addr, uint8_t* buf, uint32_t len, uint8_t wait);
uint32_t localErase(uint32_t in_addr, uint32_t len, uint8_t wait);
uint32_t localRead(uint32_t in_addr, uint8_t* buf, uint32_t in_length, uint8_t wait);


#endif /* SST26_FLASH_H_ */
