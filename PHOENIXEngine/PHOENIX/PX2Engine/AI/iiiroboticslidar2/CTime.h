/**********************************************************************************
File name:	  CTime.h
Author:       Kimbo
Version:      V1.5.0
Date:	 	  2016-4-25
Description:  Time class
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_CTIME_H_
#define EVEREST_LIDAR_CTIME_H_


/********************************** System includes *******************************/
#include <stdint.h>
#include <string>

#define INVALID_TIMESTAMP (0)

namespace everest
{
    namespace hwdrivers
    {
        typedef uint64_t TTimeStamp2;
		struct TTimeParts
		{
			uint16_t	year;	/** The year */
			uint8_t		month;  /** Month (1-12) */
			uint8_t		day;    /** Day (1-31) */
			uint8_t		hour;   /** Hour (0-23) */
			uint8_t		minute; /** Minute (0-59) */
			double		second; /** Seconds (0.0000-59.9999) */
			uint8_t		day_of_week; /** Day of week (1:Sunday, 7:Saturday) */
			int			daylight_saving;
		};

        class CTime
        {
            public:
                /* Constructor */
                CTime();

                /* Destructor */
                ~CTime();

                /* Add time */
                static TTimeStamp2 addTime(TTimeStamp2 time, double time_ms);

                /* Get real time */
                static TTimeStamp2 getRealTime();

                /* Transfer time to string */
                static std::string timeToString(TTimeStamp2 &time);

                /* Get cpu time */
                static TTimeStamp2 getCpuTime();

                /* Time_t to stamp */
                static TTimeStamp2 time_tToTimestamp(const time_t &t);

                /* Get time difference, unit is s */
                static double timeDifference( const TTimeStamp2 &t1, const TTimeStamp2 &t2 );

                /* Get time string */
                static std::string getTimeString();

                /* Timestamp to parts */
                static void timestampToParts( TTimeStamp2 t, TTimeParts &p , bool localTime);

                /* timestamp to time_t */
                static double timestampTotime_t( const TTimeStamp2  &t );

            private:
        };
    }
}

#endif


