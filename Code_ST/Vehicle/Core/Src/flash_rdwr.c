/*
 * flash_rdwr.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */


#include "flash_rdwr.h"
#include "stm32f4xx_hal_flash.h"


int Convert4BytesToWordData(uint8_t *pInput, uint32_t *pOutBuffer, int InputLength)
{
	int length = 0, h = 0;
	if((InputLength % 4) != 0)
		length = InputLength / 4 + 1;
	else
		length /= 4;
	for(int i = 0; i < length; i++)
	{
		for(int j = 3; j >= 0; j--)
		{
			pOutBuffer[i] += pInput[h] << (8 * j);
			h++;
		}
	}
	return length;
}




void WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr)
{
	for(int i = 0; i < 100; i++)
	{
		pflash->WriteIn32bBuffer[i] = 0;
	}
	pflash->WriteIn32bBuffer[0] = Convert4BytesToWordData(pflash->WriteInBuffer,&pflash->WriteIn32bBuffer[1],pflash->Length);
	//FLASH_Unlock();
	//FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[0]);
	FLASH_BaseAddr += 4;
	for(int i = 0; i < pflash->WriteIn32bBuffer[0]; i++)
	{
		//FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[i + 1]);
		FLASH_BaseAddr += 4;
	}
	//FLASH_Lock();
}

/** @brief  : Read from flash memory
**  @agr    : input and output message
**  @retval : None
**/
void ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr)
{
	int i = 0;
	uint32_t mask;
	int length = (int)(*(uint32_t*)FLASH_BaseAddr); /* 4 first byte hold the length */
	for(int count = 0; count < length; count++)
	{
		mask = 0xFF000000;
		FLASH_BaseAddr += 4;
		for(int j = 3; j >= 0; j--)
		{
			pflash->ReadOutBuffer[i] = (uint8_t)(((*(uint32_t*)FLASH_BaseAddr) & mask) >> (8 * j));
			mask >>= 8;
			i++;
		}
	}
}

/** @brief  : Erase sector flash memory
**  @agr    : None
**  @retval : None
**/
void EraseMemory(uint32_t Flash_Sector)
{
	//FLASH_Unlock();
	//FLASH_EraseSector(Flash_Sector, FLASH_ProgramType_Word);
	//FLASH_Lock();
}




