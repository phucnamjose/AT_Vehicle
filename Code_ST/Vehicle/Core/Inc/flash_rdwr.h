/*
 * flash_rdwr.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#ifndef INC_FLASH_RDWR_H_
#define INC_FLASH_RDWR_H_

#include "def_myself.h"




//#define	           FLASH_ProgramType_Byte	        VoltageRange_1
//#define	           FLASH_ProgramType_HalfWord       VoltageRange_2
//#define	           FLASH_ProgramType_Word           VoltageRange_3
//#define	           FLASH_ProgramType_DoubleWord     VoltageRange_4
#define	           FLASH_PIDPara_BaseAddr           0x08060000  // (4 KBytes) (0x08060000 - 0x08060FFF)
#define	           FLASH_FuzPara_BaseAddr           0x08061000  // (4 Kbytes) (0x08061000 - 0x08061FFF)
#define	           FLASH_GPSPara_BaseAddr           0x08040000  // (128 KBytes)


typedef struct FlashMemory{
	uint32_t 	WriteIn32bBuffer[100];
	uint8_t		ReadOutBuffer[500];
	uint8_t		WriteInBuffer[500];
	int			Length;
	//char  		Message[MESSAGE_ROW][MESSAGE_COL];
} FlashMemory;



/*-------- Flash Memory Embedded functions --------*/
void                    WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr);
void                    ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr);
void                    EraseMemory(uint32_t Flash_Sector);

#endif /* INC_FLASH_RDWR_H_ */
