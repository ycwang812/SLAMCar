/*********************************************************************************
File name:	  C3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_C3IROBOTICSLIDAR_H
#define EVEREST_LIDAR_C3IROBOTICSLIDAR_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket2.h"
#include "CLidarUnpacket2.h"
#include "CLidarPacketReceiver2.h"
#include "CSimulateSerial2.h"

/******************************* System libs includes *****************************/
#include <vector>

/******************************* Other libs includes ******************************/

namespace everest
{
	namespace hwdrivers
	{
	    struct TLidarScan2
	    {
	        TLidarScan2() : angle(), distance(), signal(){ }

	        void insert(TLidarScan2 &scan)
	        {
                this->angle.insert(this->angle.end(), scan.angle.begin(), scan.angle.end());
                this->distance.insert(this->distance.end(), scan.distance.begin(), scan.distance.end());
                this->signal.insert(this->signal.end(), scan.signal.begin(), scan.signal.end());
	        }

	        void clear()
	        {
	            angle.clear();
	            distance.clear();
	            signal.clear();
	        }

            size_t getSize() {return angle.size();}

	        std::vector<float> angle;
	        std::vector<float> distance;
	        std::vector<int>   signal;

	    };

        enum TLidarGrabResult2
        {
            LIDAR_GRAB_ING2 = 0,
            LIDAR_GRAB_SUCESS2,
            LIDAR_GRAB_ERRO2,
            LIDAR_GRAB_ELSE2
        };

        enum TLidarCommandID2
        {
            I3LIDAR_DISTANCE2 = 0xA9,
            I3LIDAR_HEALTH2   = 0xAB,
            I3LIDAR_LIDAR_SPEED2  = 0xAE,
            I3LIDAR_NEW_DISTANCE2 = 0xAD
        };

		class C3iroboticsLidar2
		{
            public:
                enum TGrabScanState2
                {
                    GRAB_SCAN_FIRST2 = 0,
                    GRAB_SCAN_ELSE_DATA2
                };

                /* Constructor */
                C3iroboticsLidar2();

                /* Destructor */
                ~C3iroboticsLidar2();

                /* Set device connect */
                bool initilize(CDeviceConnection *device_connect);

                /* Get scan data */
                TLidarGrabResult2 getScanData();

                /* Get Lidar Error */
                TLidarError2 getLidarError() { return m_lidar_erro; }

                /* Get lidar scan */
                TLidarScan2& getLidarScan() { return m_lidar_scan; }

                /* Set data with signal*/
                void setDataWithSignal(bool data_with_signal) {m_data_with_signal = data_with_signal;}

                /* Return true if receive lidar speed */
                bool isReceiveLidarSpeed() {bool flag = m_receive_lidar_speed; m_receive_lidar_speed = false; return flag;}

                /* Get Lidar current speed */
                double getLidarCurrentSpeed() {return m_current_lidar_speed;}

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_receiver.enableLogWhenReceiveTimeOvers(state);}

            private:
                /* Analysis packet */
                TLidarGrabResult2 analysisPacket(CLidarPacket2 &lidar_packet);

                /* Analysis tooth scan */
                TLidarGrabResult2 analysisToothScan(CLidarPacket2 &lidar_packet);

                /* Analysis new tooth scan */
                TLidarGrabResult2 analysisNewToothScan(CLidarPacket2 &lidar_packet);

                /* Analysis health info */
                TLidarGrabResult2 analysisHealthInfo(CLidarPacket2 &lidar_packet);

                /* Analysis lidar speed */
                TLidarGrabResult2 analysisLidarSpeed(CLidarPacket2 &lidar_packet);

                /* Reset scan grab */
                void resetScanGrab();

                /* Combine scan */
                TLidarGrabResult2 combineScan(TToothScan2 &tooth_scan);

                /* Return true if tooth scan is first */
                bool isFirstScan(TToothScan2 &tooth_scan);

                /* Grab first scan */
                TLidarGrabResult2 grabFirstScan(TToothScan2 &tooth_scan);

                /* Change tooth scan to lidar scan */
                void toothScan2LidarScan(TToothScan2 &tooth_scan, TLidarScan2 &lidar_scan);

                /* Params */
                struct TParams2
                {
                    /* Constructor */
                    TParams2()
                    {
                        packet_wait_time_ms = 100;
                        tooth_number = 16;
                        scan_time_out_ms = 150;
                    }

                    size_t packet_wait_time_ms;
                    size_t scan_time_out_ms;
                    int tooth_number;
                };

            private:
                CDeviceConnection       *m_device_connect;
                CLidarPacketReceiver2    m_receiver;
                TParams2                 m_params;
                TLidarScan2              m_lidar_scan;
                TLidarError2             m_lidar_erro;
                TGrabScanState2          m_grab_scan_state;
                int                     m_grab_scan_count;
                float                   m_last_scan_angle;
                CLidarPacket2            m_packet;
                TToothScan2              m_remainder_tooth_scan;
                bool                    m_remainder_flag;
                bool                    m_data_with_signal;

                bool                    m_receive_lidar_speed;
                double                  m_current_lidar_speed;

                CCountDown              m_data_count_down;
		};
	}
}

#endif


