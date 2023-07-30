/*
 * BootLoader.c
 *
 *  Created on: Jul 16, 2023
 *      Author: Nikhil
 */
#include <string.h>
#include "BootLoader.h"
#include "GlobalVar.h"

static uint8_t blRxBuff[200] = {0};

#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000


static uint8_t supportedCMDs[] = {
									BL_GET_VER,
									BL_GET_HELP,
									BL_GET_CID,
									BL_GET_RDP_STATUS,
									BL_GO_TO_ADDR,
									BL_FLASH_ERASE,
									BL_MEM_WRITE,
									BL_READ_SECTOR_P_STATUS
                                  };



void blUartReadData(void)
{
	uint8_t rcvLen = 0;

	while(1)
	{
		memset(blRxBuff,0x00,sizeof(blRxBuff));
		//First only one byte from Host, which is length field of command packet
		HAL_UART_Receive(BL_CMD_UART,blRxBuff,1,HAL_MAX_DELAY);
		rcvLen =  blRxBuff[0];
		HAL_UART_Receive(BL_CMD_UART,&blRxBuff[1],rcvLen,HAL_MAX_DELAY);

		switch(blRxBuff[1])
		{
		case BL_GET_VER:
			blHandleGetVerCmd(blRxBuff);
			break;
		case BL_GET_HELP:
			blHandleGetHelpCmd(blRxBuff);
			break;
		case BL_GET_CID:
			blHandleGetCIDCmd(blRxBuff);
			break;
		case BL_GET_RDP_STATUS:
			blHandleSectorReadProtectStatus(blRxBuff);
			break;
		case BL_GO_TO_ADDR:
			blHandleGoCmd(blRxBuff);
			break;
		case BL_FLASH_ERASE:
			blHandleFlashEraseCmd(blRxBuff);
			break;
		case BL_MEM_WRITE:
			blHandleMemWriteCmd(blRxBuff);
			break;
		case BL_EN_RW_PROTECT:
			blHandleEnRwProtect(blRxBuff);
			break;
		case BL_MEM_READ:
			blHandleReadOtp(blRxBuff);
			break;
		case BL_READ_SECTOR_P_STATUS:
			blHandleSectorReadProtectStatus(blRxBuff);
			break;
		case BL_OTP_READ:
			blHandleReadOtp(blRxBuff);
			break;
		case BL_DIS_RW_PROTECT:
			blHandleDisRWProtect(blRxBuff);
			break;
		default:
			debugMsg("BL Invalid Command\r\n");
			break;

		}
	}
}

void blJumpUserAppl(void)
{
   void (*appResetHandler)(void);
   uint32_t mspVal  = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
   debugMsg("BooltLoader Jump to user application\r\n");

   //this function comes from cmsis
   __set_MSP(mspVal);

   uint32_t resetHandler = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

   appResetHandler = (void*)resetHandler;

   debugMsg("Application reset handler address:%#x\r\n",appResetHandler);

   //jump to reset handler
   appResetHandler();


}

void blHandleGetVerCmd(uint8_t * rxBuff)
{
	uint8_t blVersion;
	//verify check sum of recevied command
	debugMsg("blHandleGetVerCmd\r\n");

	//Total length of command packet
	uint32_t cmdPacketLen =  rxBuff[0] + 1;

	//extract crc32 sent in packet by host
	uint32_t hostCrc = *((uint32_t *)(rxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&rxBuff[0], cmdPacketLen - 4,hostCrc))
	{
		debugMsg("BL Check sum success\r\n");
		//Send acknowledgement
		blSendAck(rxBuff[0],1);
		blVersion = getBLVersion();
		debugMsg("BL Version :%d %#x\r\n",blVersion,blVersion);
		blUARTWriteData(&blVersion,1);
	}
	else
	{
		debugMsg("BL Check Failed\r\n");
		blSendNack();
	}
}

void blHandleGetHelpCmd(uint8_t *pBuff)
{
	debugMsg("blHandleGetHelpCmd\r\n");
	//Get total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//Extract CRC from received packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("Check Sum correct\r\n");
		blSendAck(pBuff[0],sizeof(supportedCMDs));
		blUARTWriteData(supportedCMDs,sizeof(supportedCMDs));
	}
	else
	{
		debugMsg("CRC invalid\r\n");
		blSendNack();
	}
}


void blHandleGetCIDCmd(uint8_t *pBuff)
{
	uint16_t blCIDNum = 0;
	debugMsg("blHandleGetCIDCmd\r\n");

	//Get total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//Extract crc from receive packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen -4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("CRC valid\r\n");
		blSendAck(pBuff[0],2);
		blCIDNum = getMCUChipID();
		debugMsg("BL MCU ID: %d %#x !!\r\n");
		blUARTWriteData((uint8_t *)blCIDNum,2);
	}
	else
	{
		debugMsg("CRC invalid\r\n");
		blSendNack();
	}
}

void blHandleGetRdpCmd(uint8_t *pBuff)
{
	uint8_t rdpLevel = 0x00;
	debugMsg("blHandleGetRdpCmd\r\n");

	//Get total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//Extract  CRC of received packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("Check sum valid\r\n");
		blSendAck(pBuff[0],1);
		rdpLevel = getFlashRdpLevel();
		debugMsg("RDP level:%d %#x\r\n",rdpLevel,rdpLevel);
		blUARTWriteData(&rdpLevel,1);
	}
	else
	{
		debugMsg("CRC invalid\r\n");
		blSendNack();
	}
}

void blHandleGoCmd(uint8_t *pBuff)
{
  uint32_t goAddr = 0;
  uint8_t addrValid = ADDR_VALID;
  uint8_t addrInvalid = ADDR_INVALID;

  debugMsg("blHandleGoCmd\r\n");

  //Get total length of packet
  uint32_t cmdPacketLen = blRxBuff[0] + 1;

  //extract crc of received packet
  uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

  if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
  {
	  debugMsg("Check sum valid\r\n");
	  blSendAck(pBuff[0],1);
	  goAddr =  *((uint32_t *)&pBuff[2]);
	  debugMsg("Go Addr:%#x\r\n");

	  if(verifyAddr(goAddr) == ADDR_VALID)
	  {
		  blUARTWriteData(&addrValid, 1);

			/*jump to "go" address.
			we dont care what is being done there.
			host must ensure that valid code is present over there
			Its not the duty of bootloader. so just trust and jump */

			/* Not doing the below line will result in hardfault exception for ARM cortex M */
			//watch : https://www.youtube.com/watch?v=VX_12SjnNhY

		 // goAddr += 1;
		//  void(*lets_jump)(void) = (void *)goAddr;

		//  debugMsg("Jumping to given address\r\n");

		//  lets_jump();
		  blJumpUserAppl();

	  }
	  else
	  {
		  debugMsg("Go address Invalid\r\n");
		  blUARTWriteData(&addrInvalid, 1);

	  }

  }
}

void blHandleFlashEraseCmd(uint8_t *pBuff)
{
	uint8_t eraseStatus = 0x00;
	debugMsg("blHandleFlashEraseCmd\r\n");

	//Get total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//extract crc of received packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("Valid CRC");
		blSendAck(pBuff[0],1);
		debugMsg("Inital Sector:%d, num of sectors:%d\r\n",pBuff[2],pBuff[3]);

		eraseStatus = excuteFlashErase(pBuff[2],pBuff[3]);

		debugMsg("Flash Erase Status:%#x\r\n",eraseStatus);

		blUARTWriteData(&eraseStatus,1);
	}
	else
	{
		debugMsg("Invalid CRC\r\n");
		blSendNack();
	}
}

void blHandleMemWriteCmd(uint8_t *pBuff)
{

//	uint8_t addrValid = ADDR_VALID;
	uint8_t writeStatus = 0x00;
	uint8_t chkSum = 0, len = 0;
	len = pBuff[0];

	uint8_t payLoadLen = pBuff[6];

	uint32_t memAddr = *((uint32_t *)(&pBuff[2]));

	chkSum = pBuff[len];

	debugMsg("blHandleMemWriteCmd\r\n");

	//total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//extract crc of received packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("Valid CRC\r\n");
		blSendAck(pBuff[0],1);

		debugMsg("mem write address:%#x\r\n",memAddr);

		if(verifyAddr(memAddr) == ADDR_VALID)
	//	if(1)
		{

			writeStatus = excuteMemWrite(&pBuff[7], memAddr,payLoadLen);

			blUARTWriteData(&writeStatus,1);
		}
		else
		{
			debugMsg("Invalid Address\r\n");
			writeStatus = ADDR_INVALID;
			blUARTWriteData(&writeStatus,1);
		}
	}
	else
	{
		debugMsg("Invalid CRC\r\n");
		blSendNack();
	}
}

void blHandleEnRwProtect(uint8_t *pBuff)
{

}

void blHandleMemRead(uint8_t *pBuff)
{
}

void blHandleSectorReadProtectStatus(uint8_t *pBuff)
{
	uint16_t status;
	debugMsg("blHandleSectorReadProtectionStatus\r\n");

	//total length of packet
	uint32_t cmdPacketLen = blRxBuff[0] + 1;

	//extract crc of received packet
	uint32_t rcvCRC = *((uint32_t *)(blRxBuff + cmdPacketLen - 4));

	if(!blVerifyCrc(&blRxBuff[0],cmdPacketLen - 4,rcvCRC))
	{
		debugMsg("Valid CRC\r\n");
		//blSendAck(&pBuff[0],2);
		status = readOBRProtectionStatus();
		debugMsg("nWRP status: %#x\r\n",status);
		blUARTWriteData((uint8_t *)&status,2);
	}
	else
	{
		debugMsg("Invalid CRC\r\n");
		blSendNack();
	}
}

void blHandleReadOtp(uint8_t *pBuff)
{

}

void blHandleDisRWProtect(uint8_t *pBuff)
{

}


void blSendAck(uint8_t cmdCode,uint8_t followLen)
{
	//Send 2 Bytes first byte is ack and Sencond byte is len value
	uint8_t ackBuff[2] = {0};
	ackBuff[0] = BL_ACK;
	ackBuff[1] = followLen;
	HAL_UART_Transmit(BL_CMD_UART,ackBuff,2,HAL_MAX_DELAY);

}

void blSendNack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(BL_CMD_UART,&nack,1,HAL_MAX_DELAY);
}


uint8_t blVerifyCrc(uint8_t *pData,uint32_t len,uint32_t crcHost)
{
	uint8_t ret = VERIFY_CRC_FAIL;

	uint32_t uwCRCVal = 0xFF;
	for(uint32_t i = 0; i < len; i++)
	{
		uint32_t iData = pData[i];
		uwCRCVal = HAL_CRC_Accumulate(&hcrc,&iData, 1);
	}

	__HAL_CRC_DR_RESET(&hcrc);

	if(uwCRCVal == crcHost)
	{
		ret = VERIFY_CRC_SUCCESS;
	}
	return ret;
}

uint8_t getBLVersion(void)
{
	uint8_t ret;
	ret = (uint8_t)BL_VERSION;
	return ret;
}

void blUARTWriteData(uint8_t *pBuff,uint32_t len)
{
	HAL_UART_Transmit(BL_CMD_UART,pBuff,len, HAL_MAX_DELAY);
}

uint16_t getMCUChipID(void)
{
	uint16_t ret = 0;
	ret = (uint16_t)(DBGMCU->IDCODE);
    return ret;
}

uint8_t getFlashRdpLevel(void)
{
	uint8_t ret = 0;

#if 0
	FLASH_OBProgramInitTypeDef obHandle;
	HAL_FLASHEx_OBGetConfig(&obHandle);
	ret = (uint8_t)obHandle.RDPLevel;
#else
	volatile uint32_t *pOBAddr = (uint32_t *)0x1FFFC000;
	ret = (uint8_t)(*pOBAddr >> 8);
#endif

	return ret;
}

uint8_t verifyAddr(uint32_t goAddr)
{
	uint8_t ret = ADDR_INVALID;
	if(goAddr >= BL_SRAM1_BASE && goAddr <= BL_SRAM1_END)
	{
		ret = ADDR_VALID;
	}
	else if(goAddr >= BL_SRAM2_BASE && goAddr <= BL_SRAM2_END)
	{
		ret = ADDR_VALID;
	}
	else if(goAddr >= BL_FLASH_BASE && goAddr <= BL_FLASH_END)
	{
		ret = ADDR_VALID;
	}
	else if(goAddr >= BL_SRAM3_BASE && goAddr <= BL_SRAM3_END)
	{
		ret = ADDR_VALID;
	}
	else if(goAddr >= BL_BKPSRAM_BASE && goAddr <= BL_BKPSRAM_END)
	{
		ret = ADDR_VALID;
	}
	else
	{
		ret = ADDR_VALID;
	}

	return ret;

}

uint8_t excuteFlashErase(uint8_t secNum,uint8_t numOfSec)
{
    //we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	FLASH_EraseInitTypeDef flashEraseHandle;
	uint32_t sectorError;

	HAL_StatusTypeDef status;

	if(numOfSec > 8)
	{
		return INVALID_SECTOR;
	}

	if((numOfSec == 0xFF) || (numOfSec < 7))
	{
		if(numOfSec == (uint8_t)0xff)
		{
			flashEraseHandle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			//Here need to calculate how many sector wants to erase
		    uint8_t remainigSec = 8 - secNum;
		    if(numOfSec > remainigSec)
		    {
		    	numOfSec = remainigSec;
		    }
		    flashEraseHandle.TypeErase = FLASH_TYPEERASE_SECTORS;
		    flashEraseHandle.Sector = secNum;
		    flashEraseHandle.NbSectors = numOfSec;
		}
		flashEraseHandle.Banks = FLASH_BANK_1;

		//Get access to flash
		HAL_FLASH_Unlock();
		flashEraseHandle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t)HAL_FLASHEx_Erase(&flashEraseHandle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}

	return INVALID_SECTOR;

}

uint8_t excuteMemWrite(uint8_t *pBuff,uint32_t memAddr,uint32_t len)
{
  uint8_t status = HAL_OK;
  //get access to flash by unlocking its registers
  HAL_FLASH_Unlock();
  for(uint32_t i = 0; i < len; i++)
  {
	  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,memAddr + i,pBuff[i]);
  }
  HAL_FLASH_Lock();
  return status;debugMsg("Invalid CRC\r\n");
}

uint8_t configureFlashSectorRWProtection(uint8_t secDetails,uint8_t protectionMode,uint8_t disable)
{
	uint8_t ret;
		return ret;
}

uint16_t readOBRProtectionStatus(void)
{
	//this structure is give by ST flash driver to hold the OB(Option Byte) contents....
	FLASH_OBProgramInitTypeDef OBInit;

	//First unlock OB memory access
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&OBInit);
	HAL_FLASH_OB_Lock();

	return (uint16_t)OBInit.WRPSector;
}

void debugMsg(char *format,...)
{
	char str[80];

	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(DEBUG_UART,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
}



