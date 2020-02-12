/*********************************************************************************
File name:	  CLidarUnpacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-06
Description:  3irobotics Lidar unpacket
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_CLIDARUNPACKET_H
#define EVEREST_LIDAR_CLIDARUNPACKET_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket2.h"

/******************************* System libs includes *****************************/
#include <vector>

/******************************* Other libs includes ******************************/

namespace everest
{
	namespace hwdrivers
	{
	    struct TToothScan2
	    {
            TToothScan2() : offset_valid(false), offset_angle(0.0), lidar_speed(-1.0), angle(0.0), distance(), signal(){ }

            float getAngle() { return angle; }
            size_t getSize() { return distance.size(); }

            bool                  offset_valid;
            float                 offset_angle;     // unit: degree
            float                 lidar_speed;      // unit: lidar speed
            float                 angle;            // unit: degree
            std::vector<float>    distance;         // unit: meter
            std::vector<int>      signal;           // range: 0 - 255
	    };

        enum TLidarError2
        {
            LIDAR_ERROR_NONE = 0,
            LIDAR_ERROR_CCD,
            LIDAR_ERROR_CALIBRATION_PARAMS,
            LIDAR_ERROR_LD_OPTICAL_POWER_PARAMS,
            LIDAR_ERROR_UNKNOW
        };

		class CLidarUnpacket
		{
            public:
                /* Constructor */
                CLidarUnpacket();

                /* Destructor */
                ~CLidarUnpacket();

                /* Lidar unpacket */
                static TToothScan2 unpacketLidarScan(CLidarPacket2 &packet);

                /* Lidar unpacket */
                static TToothScan2 unpacketLidarScan2(CLidarPacket2 &packet);

                /* Lidar unpacket */
                static TToothScan2 unpacketNewLidarScanHasSingal(CLidarPacket2 &packet);

                /* Lidar unpacket */
                static TToothScan2 unpacketNewLidarScanNoSingal(CLidarPacket2 &packet);

                /* Health info unpacket */
                static TLidarError2 unpacketHealthInfo(CLidarPacket2 &packet);

                /* Lidar speed */
                static int unpacketLidarSpeed(CLidarPacket2 &packet);
		};
	}
}

#endif


