/*
 * BootLoader.h
 *
 *  Created on: Jul 16, 2023
 *      Author: Nikhil
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include <stdint.h>

//Macros for Bootloader Commands
//#define <BootloaderCmd>      <Cmd Code>
#define BL_VERSION              0x10

//Version Command
#define BL_GET_VER              0x51

//Bootloader Help Command
#define BL_GET_HELP             0x52

//MCU chip identification number command
#define BL_GET_CID              0x53

//Flash Read protection command
#define BL_GET_RDP_STATUS       0x54

//Jump bootloader specific address command
#define BL_GO_TO_ADDR          0x55

//Mass Erase or Sector Erase of Flash command
#define BL_FLASH_ERASE         0x56

//Write Flash command
#define BL_MEM_WRITE            0x57

//Enable or Disable read/write protect on sector command
#define BL_EN_RW_PROTECT        0x58

//Flash Read command
#define BL_MEM_READ              0x59

//Read all protection status command
#define BL_READ_SECTOR_P_STATUS    0x5A

//Read OTP contents
#define BL_OTP_READ             0x5B

//Disable all sector Read/Write protection
#define BL_DIS_RW_PROTECT       0x5C

//Acknowledge
#define BL_ACK   0xA5
//Not acknowledge
#define BL_NACK  0x7F

//Correc CRC
#define VERIFY_CRC_SUCCESS 0x00
//Wrong CRC
#define VERIFY_CRC_FAIL    0x01

//Valid Addr
#define ADDR_VALID 0x00
//Invalid Addr
#define ADDR_INVALID 0x01

//Invalid Sector
#define INVALID_SECTOR 0x04

//Memory Map of controller
#define BL_SRAM1_BASE    0x20000000U
#define BL_SRAM1_END     0x2001BFFFU
#define BL_SRAM2_BASE    0x2001C000U
#define BL_SRAM2_END     0x2001FFFFU
#define BL_SRAM3_BASE    0x20020000U
#define BL_SRAM3_END     0x2002FFFFU
#define BL_FLASH_BASE    0x08000000U
#define BL_FLASH_END     0x080FFFFFU
#define BL_BKPSRAM_BASE  0x40024000U
#define BL_BKPSRAM_END   0x40024FFFU

//BootLoader Function Pro Types
void blUartReadData(void);
void blJumpUserAppl(void);
void blHandleGetVerCmd(uint8_t * rxBuff);
void blHandleGetHelpCmd(uint8_t *pBuff);
void blHandleGetCIDCmd(uint8_t *pBuff);
void blHandleGetRdpCmd(uint8_t *pBuff);
void blHandleGoCmd(uint8_t *pBuff);
void blHandleFlashEraseCmd(uint8_t *pBuff);
void blHandleMemWriteCmd(uint8_t *pBuff);
void blHandleEnRwProtect(uint8_t *pBuff);
void blHandleMemRead(uint8_t *pBuff);
void blHandleSectorReadProtectStatus(uint8_t *pBuff);
void blHandleReadOtp(uint8_t *pBuff);
void blHandleDisRWProtect(uint8_t *pBuff);

void blSendAck(uint8_t cmdCode,uint8_t followLen);
void blSendNack(void);

uint8_t blVerifyCrc(uint8_t *pData,uint32_t len,uint32_t crcHost);
uint8_t getBLVersion(void);
void blUARTWriteData(uint8_t *pBuff,uint32_t len);

uint16_t getMCUChipID(void);
uint8_t getFlashRdpLevel(void);
uint8_t verifyAddr(uint32_t goAddr);
uint8_t excuteFlashErase(uint8_t secNum,uint8_t numOfSec);
uint8_t excuteMemWrite(uint8_t *pBuff,uint32_t memAddr,uint32_t len);
uint8_t configureFlashSectorRWProtection(uint8_t secDetails,uint8_t protectionMode,uint8_t disable);
uint16_t readOBRProtectionStatus(void);





#endif /* INC_BOOTLOADER_H_ */
