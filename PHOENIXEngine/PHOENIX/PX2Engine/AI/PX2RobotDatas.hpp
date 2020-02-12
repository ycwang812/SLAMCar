// PX2RobotDatas.hpp

#ifndef PX2ROBOTDATAS_HPP
#define PX2ROBOTDATAS_HPP

#include "PX2CorePre.hpp"
#include "PX2APoint.hpp"
#include "PX2Object.hpp"

namespace PX2
{

	class PX2_ENGINE_ITEM RobotMapDataStruct
	{
	public:
		RobotMapDataStruct();
		~RobotMapDataStruct();

		int MapWidth;
		int MapHeight;
		float MapResolution;
		int IndexX;
		int IndexY;
		APoint CurPos;
		float CurAngle;
	};

	class PX2_ENGINE_ITEM RobotToDoStruct 
	{
	public:
		RobotToDoStruct();
		~RobotToDoStruct();

		// 0 none
		// 1 Speed
		// 2 gotarget
		// 3 clearpathfinder
		// 4 savemap
		// 5 loadmap
		// 6 setselfdrawobst
		int Type;

		float LeftSpeed;
		float RightSpeed;
		float HeadDegree;

		APoint TargetPos;
		
		float Range;
		int Value;
	};

	class PX2_ENGINE_ITEM RobotMapData : public Object
	{
		PX2_DECLARE_RTTI;
		PX2_DECLARE_NAMES;
		PX2_DECLARE_STREAM(RobotMapData);

	public:
		RobotMapData();
		virtual ~RobotMapData();

		enum MapObstType
		{
			MOT_OBJST = 0,
			MOT_OBJST_LIDAR = 1,
			MOT_NONE = 10,
			MOT_ROBOTPOS = 100,
			MOT_CANGO = 200,
			MOT_MAX_TYPE
		};

		void Initlize(int mapWidth, int mapHeight, float resolution);

	public:
		std::vector<unsigned char> _ExtendLarger(
			const std::vector<unsigned char> &mapping, int mapWidth,
			int mapHeight, int equalValue);
		void _SetMapUsingData(std::vector<unsigned char> &maping, int mapWidth,
			int mapHeight, int x, int y, int val);

		RobotMapDataStruct MapStruct;
		std::vector<unsigned char> Map2DOrigin;

		std::vector<unsigned char> SelfDrawMapData2D;
		std::vector<unsigned char> PathFinderLineDraw;
	};

	PX2_REGISTER_STREAM(RobotMapData);
	typedef Pointer0<RobotMapData> RobotMapDataPtr;

}

#endif