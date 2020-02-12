/*********************************************************************************
File name:	  CLidarPacketReceiver2.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  lidar packet receiver
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************* File includes **********************************/
#include "CLidarPacketReceiver2.h"

/******************************* Current libs includes ****************************/
#include "CDeviceConnection.h"
#include "CCountDown.h"

/********************************** Name space ************************************/
using namespace everest;
using namespace everest::hwdrivers;

#define LIDAR_PACEKT_HEADER1 0xAA
#define LIDAR_PACEKT_HEADER2 0x00

/***********************************************************************************
Function:     CLidarPacketReceiver2
Description:  The constructor of CLidarPacketReceiver2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::CLidarPacketReceiver2()
{
    m_device_conn = NULL;
    m_log_when_receive_time_over = false;
    reset();
    m_counter = 0;
}

/***********************************************************************************
Function:     CLidarPacketReceiver2
Description:  The destructor of CLidarPacketReceiver2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::~CLidarPacketReceiver2()
{
    if(m_save_fp)
    {
        m_save_fp.close();
    }
}

/***********************************************************************************
Function:     receivePacket
Description:  Receive lidar packet, if return true, it means receive a valid packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver2::receivePacket(CLidarPacket2 *packet)
{
	/* Judge whether serial is connecting */
	if (packet == NULL || m_device_conn == NULL || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN)
	{
		printf("[CLidarPacketReceiver2] receivePacket: connection not open!\n");
		return false;
	}

	/* Read packet */
	m_count_down.setTime((double)m_params.packet_max_time_ms);
	char ch;
	while (1)
	{
		if (m_count_down.isEnd())
		{
			//			printf("[CLidarPacketReceiver2] Receive packet time %5.2f ms is over!\n", m_count_down.getInputTime());
			if (m_log_when_receive_time_over)
			{
				printf("[CLidarPacketReceiver2] Receive packet time is over!\n");
			}
			return false;
		}

		int read_bytes = m_device_conn->read((char *)&ch, 1, 1);
		if (read_bytes == 0)
		{
			continue;
		}
		else if (read_bytes < 0)
		{
			printf("[CLidarPacketReceiver2] finish read data read bytes is %d!\n", read_bytes);
			return false;
		}
		else
		{
			TPacketResult packet_result = readPacket(packet, ch);
			switch (packet_result)
			{
			case PACKET_ING: break;
			case PACKET_SUCCESS:
			{
				reset();
				return true;
			}
			case PACKET_FAILED:
			{
				reset();
				return false;
			}
			}
		}
	}

	printf("[CLidarPacketReceiver2] It should not come to here!\n");
	return false;
}

/***********************************************************************************
Function:     readPacket
Description:  Read packet, if it return ture, it means read complete packet or enter
              erro state
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::TPacketResult CLidarPacketReceiver2::readPacket(CLidarPacket2 *packet, u8 ch)
{
    TPacketResult packet_result = PACKET_ING;
    switch(m_state)
    {
        case STATE_HEADER1: packet_result = processStateHeader1(packet, ch); break;
        case STATE_HEADER2: packet_result = processStateHeader2(packet, ch); break;
        case STATE_LENGHT: packet_result =  processStateLength(packet, ch); break;
        case STATE_ACQUIRE_DATA: packet_result = processStateAcquireData(packet, ch); break;
        default:
            printf("[CLidarPacketReceiver2] Enter erro state %d!\n", m_state);
        break;
    }
    return packet_result;
}

/***********************************************************************************
Function:     processStateHeader1
Description:  Proces state header1
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::TPacketResult CLidarPacketReceiver2::processStateHeader1(CLidarPacket2 *packet, u8 ch)
{
    if(ch == LIDAR_PACEKT_HEADER1)
    {
        packet->reset();
        packet->pushBack(ch);
        m_state = STATE_HEADER2;
        m_count_down.setTime(m_params.packet_wait_time_ms);
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateHeader2
Description:  Proces state header2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::TPacketResult CLidarPacketReceiver2::processStateHeader2(CLidarPacket2 *packet, u8 ch)
{
    if(ch == LIDAR_PACEKT_HEADER2)
    {
        packet->pushBack(ch);
        m_state = STATE_LENGHT;
    }
    else
    {
        reset();
        printf("[CLidarPacketReceiver2] Find erro header2 %d!\n", int(ch));
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateLength
Description:  Process state length
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::TPacketResult CLidarPacketReceiver2::processStateLength(CLidarPacket2 *packet, u8 ch)
{
#if 1
	/* Limit packet length */
	if (ch < 6 || ch > 250)
	{
		reset();
		printf("[CLidarPacketReceiver2] Find erro length is 0x%x!\n", (ch));
		return PACKET_ING;
	}
#endif
	packet->pushBack(ch);

	// Add 2bytes for receive CRC16, sub 3 bytes for header(1bytes) and length(2bytes)
	m_packet_length = (int)ch + 2 - 3;

	m_state = STATE_ACQUIRE_DATA;

	return PACKET_ING;
}

/***********************************************************************************
Function:     processStateAcquireData
Description:  Process state acquire data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver2::TPacketResult CLidarPacketReceiver2::processStateAcquireData(CLidarPacket2 *packet, u8 ch)
{
    m_actual_count++;
    packet->pushBack(ch);
    if(m_actual_count == m_packet_length)
    {
        reset();
		if (packet->verifyCheckSum(packet->getPrototypeCode()))
        //if(packet->verifyCheckSum())
        {
            return PACKET_SUCCESS;
        }
        else
        {
            printf("[CLidarPacketReceiver2] CRC verify wrong!\n");
//            packet->printHex();
            return PACKET_FAILED;
        }
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     reset
Description:  Reset
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacketReceiver2::reset()
{
    m_state = STATE_HEADER1;
    m_actual_count = 0;
    m_packet_length = 0;
}
