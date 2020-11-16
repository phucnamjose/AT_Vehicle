/*
 * arduino.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */
#include "arduino.h"
#include "string.h"
#include "stdio.h"
#include "tx_dma_manage.h"

AtSerial_t atSerialPi;
AtSerial_t atSerialMega;

ValueUsing_t valueUsingTable;

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

enum_MessageClass_t AtSerial_HaldleComputer(uint8_t at_cmd,
											uint8_t *data,
											int32_t lenght,
											mainTaskMail_t *mail) {
	// Move
	if (CMD_MOVE == at_cmd) {
		if (LENGHT_CMD_MOVE == lenght) {
			mail->cmd_code 	= MOVE_MANUAL;
			mail->move		= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Hand
	} else if (CMD_HAND == at_cmd) {
		if (LENGHT_CMD_HAND == lenght) {
			mail->cmd_code 			= HAND_MANUAL;
			mail->hand_manual		= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Position Information
	} else if (CMD_POSITION_INFO == at_cmd) {
		if (lenght >= 32) {
			memcpy(&(valueUsingTable.position_info.x), &data[0], 8);
			memcpy(&(valueUsingTable.position_info.y), &data[8], 8);
			memcpy(&(valueUsingTable.position_info.z), &data[16], 8);
			memcpy(&(valueUsingTable.position_info.yaw), &data[24], 8);
			valueUsingTable.pos_count = valueUsingTable.pos_count + 1;
			return MSG_ONPY_ME;
		} else
			return MSG_WRONG;

	} else if (CMD_SET_FEATURE == at_cmd) {
		// Care only move speed
		// TODO: ...
		return MSG_FORWARD;
	// Emergency
	} else if (CMD_EMERGENCY_STOP == at_cmd) {
		if (LENGHT_CMD_EMERGENCY_STOP == lenght) {
			mail->cmd_code 	= EMERGENCY;
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Switch mode vehicle
	} else if (CMD_SWITCH_MODE == at_cmd) {
		if (LENGHT_CMD_SWITCH_MODE == lenght) {
			mail->cmd_code 		= MODE_VEHICLE;
			mail->mode_vehicle 	= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Auto move to target point
	} else if (CMD_AUTO_MOVE == at_cmd) {
		if (LENGHT_CMD_AUTO_MOVE == lenght) {
			mail->cmd_code 	= MOVE_AUTO;
			memcpy(&(mail->target_x), &data[2], 8);
			memcpy(&(mail->target_y), &data[10], 8);
			memcpy(&(mail->target_frame[0]), &data[0], lenght);
 			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Auto hand rotate
	} else if (CMD_AUTO_HAND == at_cmd) {
		if (LENGHT_CMD_AUTO_HAND == lenght) {
			mail->cmd_code 		= HAND_AUTO;
			mail->hand_auto		= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Auto rotate vehicle
	} else if (CMD_AUTO_ROTATE == at_cmd) {
		if (LENGHT_CMD_AUTO_ROTATE == lenght) {
			mail->cmd_code 	= AUTO_ROTATE;
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Start auto moving
	} else if (CMD_AUTO_START == at_cmd) {
		if (LENGHT_CMD_AUTO_START == lenght) {
			mail->cmd_code 	= AUTO_START;
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Stop auto moving
	} else if (CMD_AUTO_STOP == at_cmd) {
		if (LENGHT_CMD_AUTO_STOP == lenght) {
			mail->cmd_code 	= AUTO_STOP;
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Otherwise
	} else {
		return MSG_FORWARD;
	}

}

enum_MessageClass_t AtSerial_HaldleArduino(uint8_t at_cmd,
							uint8_t *data,
							int32_t lenght,
							mainTaskMail_t *mail) {
	// Move
	if (CMD_MOVE == at_cmd) {
		if (LENGHT_CMD_MOVE == lenght) {
			mail->cmd_code 	= MOVE_MANUAL;
			mail->move		= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Hand Manual
	} else if (CMD_HAND == at_cmd) {
		if (LENGHT_CMD_HAND == lenght) {
			mail->cmd_code 			= HAND_MANUAL;
			mail->hand_manual		= data[0];
			return MSG_MAIL_TO_MAINTASK;
		} else
			return MSG_WRONG;
	// Worth Feature
	} else if (CMD_SET_FEATURE == at_cmd) {
		if (lenght >= 4) {
			AtSerial_ReadFeatureArduino(data);
		}
		return MSG_FORWARD;

	// Otherwise
	} else {
		return MSG_FORWARD;
	}

}

void	AtSerial_ReadFeatureArduino(uint8_t *data) {
	uint8_t	type_feature = data[0];

	if (CMD_SET_FEATURE_VAT_CAN_1 == type_feature ) {
		valueUsingTable.ls_object_1 = data[3];
	} else if (CMD_SET_FEATURE_VAT_CAN_2 == type_feature) {
		valueUsingTable.ls_object_2 = data[3];
	} else if (CMD_SET_FEATURE_VAT_CAN_3 == type_feature) {
		valueUsingTable.ls_object_3 = data[3];
	} else if (CMD_SET_FEATURE_VAT_CAN_4 == type_feature) {
		valueUsingTable.ls_object_4 = data[3];
	} else if (CMD_SET_FEATURE_VAT_CAN_5 == type_feature) {
		valueUsingTable.ls_object_5 = data[3];
	} else if (CMD_SET_FEATURE_VAT_CAN_6 == type_feature) {
		valueUsingTable.ls_object_6 = data[3];
	}
}

void	AtSerial_SetPowerMotion(uint8_t value) {
	uint8_t data[4];

	data[0] = RELAY_MOTION_POW;
	data[3] = value;
	AtSerial_PrepareCommand(&atSerialMega, CMD_SET_FEATURE, data, 0, 4);
	serial_sendArduinoMega(&(atSerialMega.bufferWrite[0]), atSerialMega._sendByteCount);
}

void	AtSerial_RequestPosition(void) {
	uint8_t data = 1;

	AtSerial_PrepareCommand(&atSerialPi, CMD_POSITION_INFO, &data, 0, 1);
	serial_sendRasberryPi(&(atSerialPi.bufferWrite[0]), atSerialPi._sendByteCount);
}

void	AtSerial_ReportFinishTarget(uint8_t *data) {
	AtSerial_PrepareCommand(&atSerialPi, CMD_AUTO_MOVE, data, 0, LENGHT_CMD_AUTO_MOVE);
	serial_sendRasberryPi(&(atSerialPi.bufferWrite[0]), atSerialPi._sendByteCount);
}

void	AtSerial_ReportSensor(uint8_t nb_of_sensor, uint8_t value) {

}

uint32_t 	AtSerial_GetPositionCount(void) {
	return valueUsingTable.pos_count;
}

void	AtSerial_GetPosition(Position_t *position) {
	memcpy(position, &(valueUsingTable.position_info), sizeof(Position_t));
}
