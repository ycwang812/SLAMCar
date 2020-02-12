// PX2RobotExt.hpp

#ifndef PX2ROBOTEXT_HPP
#define PX2ROBTOEXT_HPP

// this file is in AI Path,but shoud putted in slampath to persurt quick compile in visual studio

#include "PX2Robot.hpp"
#include "PX2RobotExtData.hpp"
#include "PX2RobotEx_Pre.hpp"

namespace PX2
{
	
	class SLAM_DLL_ITEM RobotExt : public Robot
	{
	public:
		RobotExt();
		virtual ~RobotExt();

		static RobotExt *New();
		static void Delete(RobotExt* robot);

		virtual void Update(float appseconds, float elpasedSeconds);

		virtual void GoTarget(const APoint &targetPos, PathType type = PT_A);
		virtual void ClearPathFinder();

		virtual bool SaveMap(const std::string &filename);
		virtual bool LoadMap(const std::string &filename, 
			const APoint &initPos = APoint::ORIGIN, float curAngle = 0.0f);

		virtual void SetObstMapValueAtPos(const APoint &pos, float range = 0.1f, int val = 1.0f);

	private:
		virtual void _UpdateTransform(float elapsedSeconds);
		void _UpdateVirtualRobot1(float elaplseSeconds);
		std::vector<Vector2f > GetNearObst(float radius = 1.5f);
		virtual bool _IsInRightDirection(const AVector &dir);
		virtual void _UpdateAdjustDirection(const AVector &dir, 
			float elapsedSeconds);

		std::vector<float> DynamicWindowApproach(RobotState rState, const Vector2f &target,
			std::vector<Vector2f> &obstacle, std::vector<std::vector<RobotState> > &outRobotStates, 
			bool &isHasSpeedPlan);
		RobotState Motion(RobotState curState, float leftSpeed, float rightSpeed, float elapsedSeconds);
		std::vector<RobotState> GenerateTraj(RobotState initState, float leftSpeed, float rightSpeed);
		std::vector<float> CreateDW(RobotState curState);
		float CalcClearance(RobotState rState, std::vector<Vector2f> &obsts);
		float CalcBreakingDist(float velo);
		float CalcHeading(RobotState rState, const Vector2f &goal);

		RobotState _GetCurRobotState();
		void _RunSpeed(float elaplseSeconds);
		void _Stop();

		void _UpdateLidarDataToMapData();
		virtual void _UpdateMapDataToPathFinder();
		void _UpdateTex();
		int _Roundle1(float number);
		std::vector<APoint> _AdjustPoints(const std::vector<APoint> &poses);

		RobotState mRobotState;
		float mRunSpeedTimer;
		float mSendRobotToDoTimer;

		std::vector<Vector2f> mMap2DPoses;
	};

}

#endif