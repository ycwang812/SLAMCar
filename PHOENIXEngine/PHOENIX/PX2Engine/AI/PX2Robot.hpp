// PX2Robot.hpp

#ifndef PX2SLAM_HPP
#define PX2SLAM_HPP

#include "PX2CorePre.hpp"
#include "PX2Singleton_NeedNew.hpp"
#include "PX2RobotDatas.hpp"
#include "PX2HMatrix.hpp"
#include "PX2UIFPicBox.hpp"
#include "PX2AIPath.hpp"
#include "PX2Arduino.hpp"

namespace PX2
{

	class VoxelSection;
	class AIAgent;
	class Plugin;
	class PathingGraph;
	class LiDar;

	class PX2_ENGINE_ITEM Robot : public Singleton<Robot>
	{
	public:
		Robot();
		virtual ~Robot();

		static Robot *New();
		static void Delete(Robot* robot);

		virtual void Update(float appseconds, float elpasedSeconds);

		void SetNeedUpdateTexture(bool isNeed);
		bool IsNeedUpdateTexture() const;

		// config
		void SetMaxSpeed(float maxSpeed);
		float GetMaxSpeed() const;
		void SetMinSpeed(float minSpeed);
		float GetMinSpeed() const;

		void SetMaxAccelerate(float accele);
		float GetMaxAccelerate() const;

		void SetPredictTime(float predictTime);
		float GetPredictTime() const;

		void SetRadius(float radius);
		float GetRadius() const;

		void SetLidarOffset(float offsetDist);
		float GetLidarOffset() const;

		void SetLidarIngoreRadius(float radius);
		float GetLidarIngoreRadius() const;

		void SetMotoRate(int rate);
		int GetMotoRate() const;

		void SetWheelRadius(float radius);
		float GetWheelRadius() const;
		// end config

	protected:
		float mMaxSpeed;
		float mMinSpeed;
		float mMaxAccelerate;
		float mPredictTime;
		float mRadius;
		float mLidarIngoreRadius;
		float mLidarOffset;
		int mMotoRate;
		float mWheelRadius;

		// direction
	public:		
		const APoint &GetPosition() const;
		const AVector &GetDirection() const;
		const AVector &GetUp() const;
		const AVector &GetRight() const;
		const HMatrix &GetMatrix() const;

		// lidar slam2d
	public:
		static void InitlizeSlam2D();
		static void TernimateSlam2D();

		enum RoleType
		{
			RT_MASTER,
			RT_CONNECTOR,
			RT_MASTER_ONLY_SENDLIDAR,
			RT_CONNECTOR_CALCULATE,
			RT_MAX_TYPE
		};
		void SetRoleType(RoleType rtType);
		RoleType GetRoleType() const;

		void SetVoxelSection(VoxelSection *vs);
		VoxelSection *GetVoxelSection();

		// lidar
		void CreateLidar();
		bool LidarOpen(const std::string &portIP, int baudratePort);
		LiDar *GetLidar();

		// arduino
		void SetArduino(Arduino *arduino);
		Arduino *GetArduino();
		bool IsArduinoConnected() const;

		void SetSpeedLeft(float speed);
		void SetSpeedRight(float speed);
		void LeftRunSpeed(float speed);
		void RightRunSpeed(float speed);

		// adjust
		float GetOffsetDegree() const;
		float GetYawCrossValue() const;

		// slam
		void InitSlamMap(int mapWidth, int mapHeight, float resolution,
			bool clearCurMapData);
		bool IsSlamMapInited() const;
		void SetSlam2DMap(std::vector<unsigned char> &map);
		void SetSelfDraw(const std::vector<unsigned char> &map);
		int GetMapWidth() const;
		int GetMapHeight() const;
		float GetResolution() const;

		void SetSlam2DPosition(const APoint &pos, float angle);
		
		float GetSlam2DDegree() const;

		void SetSlamMapUpdate(bool update);
		bool IsSlamMapUpdate() const;

		Texture2D *GetTextureMap();
		UIFPicBox *GetUIFPicBoxMap();

		// map load save
		virtual bool SaveMap(const std::string &filename);
		virtual bool LoadMap(const std::string &filename, const APoint &initPos = APoint::ORIGIN, float curAngle=0.0f);
		RobotMapData &GetInitMapData();
		void ClearInitMapData();
		RobotMapData *GetCurMapData();

		// 0 force set obst
		// 200 default do nothing
		// 201 force set space can go
		virtual void SetObstMapValueAtPos(const APoint &pos, float range = 0.1f, int val=1.0f);
		void SetPathFinderLineValueAtPos(const APoint &pos, float range = 0.1f, int val = 1.0f);
		
		void SetMapValueAtPos(std::vector<unsigned char> &datas, const APoint &pos, int val);

	protected:
		virtual void _UpdateTransform(float elapsedSeconds);
		void _RecreateMapTex(int mapWidth, int mapHeight);
		virtual void _UpdateMapDataToPathFinder();
		void _SetGraphValue(int x, int y, float val);
		void _UpdateRobotTex(UIFPicBox *picBox, const APoint &pos, float rot);
		void _SetPosVal(std::vector<unsigned char> &datas, const APoint &pos, 
			float range, int value);

		RoleType mRoleType;
		ArduinoPtr mArduino;

		bool mIsNeedUpdateTexture;

		LiDar *mLiDar;
		VoxelSection *mVoxelSection;

		bool mIsSlamMapInited;
		RobotMapData mMapDataInit;
		RobotMapDataPtr mMapDataPtr;

		Texture2DPtr mTextureMap;
		UIFPicBoxPtr mPicBoxMap;

		std::vector<unsigned char> mMap2DLidarData;

		bool mIsSlamMapDataChanged;

		bool mIsSlamMapDoUpdate;

		float mOffsetDegree;

		UIFPicBoxPtr mRobotTex;
		UIFPicBoxPtr mRobotTexSlam;
		int mRobotMapX;
		int mRobotMapY;
		APoint m2DSlamPos;
		float m2DSlamAngle;

		static Plugin *mSlamPlugin;

		// common
	public:
		APoint mPosition;
		float mRotationRad;

		AVector mDirection;
		AVector mUp;
		AVector mRight;
		HMatrix mMatrix;

		AVector &GetAxisDirection();
		HMatrix &GetAxisRotMatrix();
		
	private:
		bool mIsHasAxis;

		AVector mAxisDirection;
		AVector mAxisUp;
		AVector mAxisRight;
		HMatrix mAxisRotMatrix;

		// path
	public:
		PathingGraph *GetPathingGraph();

		enum PathType
		{
			PT_A,
			PT_RRT,
			PT_MAX_TYPE
		};
		virtual void GoTarget(const APoint &targetPos, PathType type= PT_A);
		void AdjustToDirection(const AVector &dir);
		virtual void ClearPathFinder();

		AIAgent *GetAIAgent();

	public_internal:
		void _SetAIAgent(AIAgent *agent);

	protected:
		virtual bool _IsInRightDirection(const AVector &dir);
		virtual void _UpdateAdjustDirection(const AVector &dir, float elapsedSeconds);

		PathingGraphPtr mPathGraph;
		PathPlanPtr mCurPathPlan;
		APoint mGoTargetPos;
		APoint mGoingPos;

		bool mIsAdjustToDirection;
		AVector mAdjustToDirection;

		float mLeftSpeed;
		float mRightSpeed;
		bool mIsNeedSendSpeed;

		AIAgent *mAgent;
	};

#include "PX2Robot.inl"

#define PX2_ROBOT Robot::GetSingleton()

}

#endif