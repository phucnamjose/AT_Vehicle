/*
 * arduino.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */
#include "arduino.h"
#include "string.h"
#include "stdio.h"

AtSerial_t atSerialPi;
AtSerial_t atSerialMega;

union ArrayToInt _arrayToInt;
union ArrayToLong _arrayToLong;

//#pragma region AT_REGION_CRC_16
uint16_t _checksumPolynomial = 0xA001;
uint16_t _checksumTable[256];

void InitChecksum()
{
    uint16_t value;
    uint16_t temp;
    for (uint16_t i = 0; i < 256; ++i)
    {
        value = 0;
        temp = i;
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (((value ^ temp) & 0x0001) != 0)
            {
                value = (uint16_t)((value >> 1) ^ _checksumPolynomial);
            }
            else
            {
                value >>= 1;
            }
            temp >>= 1;
        }
        _checksumTable[i] = value;
    }
}

uint16_t ComputeChecksum(uint8_t bytes[], uint32_t offset, uint32_t length)
{
    uint16_t maxIndexExclude = offset + length;
    uint16_t crc = 0;
    uint8_t index;
    for (uint32_t i = offset; i < maxIndexExclude; ++i)
    {
        index = (uint8_t)(crc ^ bytes[i]);
        crc = (uint16_t)((crc >> 8) ^ _checksumTable[index]);
    }
    return crc;
}

uint16_t ComputeChecksum2(uint16_t crc, uint8_t b)
{
    return (uint16_t)((crc >> 8) ^ _checksumTable[(uint8_t)(crc ^ b)]);
}

//#pragma endregion


void AtSerial_Init(AtSerial_t *atSerial)
{
    //atSerialMs._serial = _serial;
    atSerial->_serialRecvBytes[0] = CMD_SYNC_BYTE_1;
    atSerial->_serialRecvBytes[1] = CMD_SYNC_BYTE_2;
}



uint8_t AtSerial_ReadCommand(AtSerial_t *atSerial, RingBuffer_t *ringbuff_rx)
{
	uint8_t countLoop = 20;
	//GO TO
loopReadAtSerial:
    if (countLoop == 0)
    {
        return FALSE;
    }
    countLoop--;

    uint8_t b;
    if (ringBuff_PopChar(ringbuff_rx, &b))
    {
        switch (atSerial->_serialReadState)
        {
//#pragma region AT_REGION_Sync_command
        case SRS_SYNC_BYTE_1:
            if (b == CMD_SYNC_BYTE_1)
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_2;
            }
            break;

        case SRS_SYNC_BYTE_2:
            if (b == CMD_SYNC_BYTE_2)
            {
                atSerial->_serialRecvBytesIndex = 2;
                atSerial->_serialReadState = SRS_TCP_CMD_FLAG;
            }
            else if (b == CMD_SYNC_BYTE_1)
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_2;
            }
            else
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_1;
            }
            break;
//#pragma endregion

//#pragma region AT_REGION_Command_and_padding
        case SRS_TCP_CMD_FLAG:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_tcpCommand = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_DATA_LEN_PADDING;
            break;

        case SRS_DATA_LEN_PADDING:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_tcpDataPadding = b;
            atSerial->_tcpDataIndex = SEG_INDEX_DATA_BYTE + atSerial->_tcpDataPadding;
            atSerial->_serialReadState = SRS_DATA_LEN_PLUS_PADDING_BYTE_0;
            break;
//#pragma endregion

//#pragma region AT_REGION_Get_data_len_plus_padding
        case SRS_DATA_LEN_PLUS_PADDING_BYTE_0:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_DATA_LEN_PLUS_PADDING_BYTE_1;
            break;
        case SRS_DATA_LEN_PLUS_PADDING_BYTE_1:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_DATA_LEN_PLUS_PADDING_BYTE_2;
            break;
        case SRS_DATA_LEN_PLUS_PADDING_BYTE_2:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_DATA_LEN_PLUS_PADDING_BYTE_3;
            break;
        case SRS_DATA_LEN_PLUS_PADDING_BYTE_3:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;

            _arrayToLong.array[0] = atSerial->_serialRecvBytes[SEG_INDEX_DATA_LEN_BYTE];
            _arrayToLong.array[1] = atSerial->_serialRecvBytes[SEG_INDEX_DATA_LEN_BYTE + 1];
            _arrayToLong.array[2] = atSerial->_serialRecvBytes[SEG_INDEX_DATA_LEN_BYTE + 2];
            _arrayToLong.array[3] = atSerial->_serialRecvBytes[SEG_INDEX_DATA_LEN_BYTE + 3];
            atSerial->_tcpDataLength = _arrayToLong.integer;

            atSerial->_serialReadState = SRS_CHECKSUM_HEADER_BYTE_0;
            break;
//#pragma endregion

//#pragma region AT_REGION_Check_sum_header
        case SRS_CHECKSUM_HEADER_BYTE_0:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_CHECKSUM_HEADER_BYTE_1;
            break;
        case SRS_CHECKSUM_HEADER_BYTE_1:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;

            atSerial->_serialChecksum = ComputeChecksum(atSerial->_serialRecvBytes, 0, atSerial->_serialRecvBytesIndex);
            if (atSerial->_serialChecksum == 0)
            {
                atSerial->_tcpEndDataSegmentIndex =
                    atSerial->_serialRecvBytesIndex
                    + atSerial->_tcpDataPadding // 0 or 1
                    + atSerial->_tcpDataLength
                    ;

                if (atSerial->_tcpDataLength == 0)
                {
                    atSerial->_serialReadState = SRS_CHECKSUM_DATA_BYTE_0;
                }
                else
                {
                    atSerial->_serialReadState = SRS_DATA;
                }
            }
            else if (b == CMD_SYNC_BYTE_1)
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_2;
            }
            else
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_1;
            }

            break;
//#pragma endregion

//#pragma region AT_REGION_Data
        case SRS_DATA:

            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex++;

            // Da du du lieu
            if (atSerial->_serialRecvBytesIndex >= atSerial->_tcpEndDataSegmentIndex)
            {
                atSerial->_serialReadState = SRS_CHECKSUM_DATA_BYTE_0;
            }
            break;
//#pragma endregion

//#pragma region AT_REGION_Check_sum_data
        case SRS_CHECKSUM_DATA_BYTE_0:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;
            atSerial->_serialReadState = SRS_CHECKSUM_DATA_BYTE_1;
            break;
        case SRS_CHECKSUM_DATA_BYTE_1:
            atSerial->_serialRecvBytes[atSerial->_serialRecvBytesIndex] = b;
            atSerial->_serialRecvBytesIndex += 1;

            atSerial->_serialChecksum = ComputeChecksum(&(atSerial->_serialRecvBytes[0]), SEG_INDEX_DATA_BYTE, atSerial->_tcpDataLength + atSerial->_tcpDataPadding + 2);

            // Du lieu dung
            if (atSerial->_serialChecksum == 0)
            {
                // Signal data arrive
                atSerial->_segmentLength = atSerial->_serialRecvBytesIndex;
                atSerial->_serialReadState = SRS_SYNC_BYTE_1;
                return TRUE;
            }
            else
            {
                if (b == CMD_SYNC_BYTE_1)
                {
                    atSerial->_serialReadState = SRS_SYNC_BYTE_2;
                    atSerial->_serialRecvBytesIndex = 2;
                    break;
                }

                atSerial->_serialReadState = SRS_SYNC_BYTE_1;
            }

            break;
//#pragma endregion

//#pragma region AT_REGION_Default
        default:
            if (b == CMD_SYNC_BYTE_1)
            {
                atSerial->_serialReadState = SRS_SYNC_BYTE_2;
                break;
            }

            atSerial->_serialReadState = SRS_SYNC_BYTE_1;
            break;
//#pragma endregion
        }
    }
    else
    {
        return FALSE;
    }

    goto loopReadAtSerial;
}

void 	AtSerial_PrepareCommand(AtSerial_t *atSerial,
							uint8_t command,
							uint8_t data[],
							uint32_t offset,
							uint32_t lenght)
{
    uint32_t dataLenPadding = lenght % 2;
    uint32_t dataLenPlusPadding = lenght + dataLenPadding;
    uint32_t sendByteCount =
        SEG_INDEX_DATA_BYTE // header
        + dataLenPlusPadding
        + 2 // checksum data
        ;

    atSerial->bufferWrite[0] = CMD_SYNC_BYTE_1;
    atSerial->bufferWrite[1] = CMD_SYNC_BYTE_2;

    atSerial->bufferWrite[SEG_INDEX_CMD_BYTE] = command;
    atSerial->bufferWrite[SEG_INDEX_DATA_PADDING_BYTE] = dataLenPadding;

    atSerial->bufferWrite[SEG_INDEX_DATA_LEN_BYTE] = (lenght & 0xFF);
    atSerial->bufferWrite[SEG_INDEX_DATA_LEN_BYTE + 1] = ((lenght >> 8) & 0xFF);
    atSerial->bufferWrite[SEG_INDEX_DATA_LEN_BYTE + 2] = 0;
    atSerial->bufferWrite[SEG_INDEX_DATA_LEN_BYTE + 3] = 0;

    uint16_t checksumHeader = ComputeChecksum(atSerial->bufferWrite, 0, SEG_INDEX_CHECKSUM_HEADER_BYTE);
    atSerial->bufferWrite[SEG_INDEX_CHECKSUM_HEADER_BYTE] = (checksumHeader & 0xFF);
    atSerial->bufferWrite[SEG_INDEX_CHECKSUM_HEADER_BYTE + 1] = ((checksumHeader >> 8) & 0xFF);

    if (lenght <= SEG_LEN_DATA_MAX)
    {
        atSerial->bufferWrite[SEG_INDEX_DATA_BYTE] = 0;
        memcpy(&(atSerial->bufferWrite[SEG_INDEX_DATA_BYTE + dataLenPadding]), &data[offset], lenght);

        uint16_t checksumData = ComputeChecksum(atSerial->bufferWrite, SEG_INDEX_DATA_BYTE, dataLenPlusPadding);

        atSerial->bufferWrite[sendByteCount - 2] = (checksumData & 0xFF);
        atSerial->bufferWrite[sendByteCount - 1] = ((checksumData >> 8) & 0xFF);
        atSerial->_sendByteCount = sendByteCount;
    }
}


uint8_t	AtSerial_GetCommand(AtSerial_t *atSerial) {
	return atSerial->_tcpCommand;
}

int32_t	AtSerial_GetData(AtSerial_t *atSerial, uint8_t *ptr_des) {
	int32_t index_start, len;

	index_start = atSerial->_tcpDataIndex;
	len			= atSerial->_tcpDataLength;
	memcpy(ptr_des, &(atSerial->_serialRecvBytes[index_start]), len);

	return len;
}

enum_DebugCmd AtSerial_HaldleCommand(uint8_t at_cmd,
									uint8_t *data,
									int32_t lenght,
									mainTaskMail_t *mail) {
	// Move
	if (CMD_MOVE == at_cmd) {
		if (LENGHT_CMD_MOVE == lenght) {
			mail->cmd_code 	= MOVE;
			mail->move		= data[0];
			return MOVE;
		} else
			return CMD_NONE;
	// Hand
	} else if (CMD_HAND == at_cmd) {
		if (LENGHT_CMD_HAND == lenght) {
			mail->cmd_code 	= HAND;
			mail->hand		= data[0];
			return HAND;
		} else
			return CMD_NONE;
	// Set feature
	} else if (CMD_SET_FEATURE == at_cmd) {
		if (LENGHT_CMD_SET_FEATURE == lenght) {
			mail->cmd_code	= SET_FEATURE;
			return SET_FEATURE;
		} else
			return CMD_NONE;
	} else if (CMD_POSITION_INFO == at_cmd) {
		if (lenght >= 32) {
			mail->cmd_code	= POSITION;
			memcpy(&(mail->x), &data[0], 8);
			memcpy(&(mail->y), &data[8], 8);
			memcpy(&(mail->z), &data[16], 8);
			memcpy(&(mail->yaw), &data[24], 8);
			return POSITION;
		} else
			return CMD_NONE;

	// Otherwise
	} else {
		return FORWARD_MSG;
	}

}
