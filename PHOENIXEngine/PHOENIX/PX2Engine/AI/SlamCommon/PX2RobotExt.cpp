// PX2RobotExt.cpp

#if defined ROBOTEXT_SLAMGMAPPING
#include "PX2SlamGMappingManager.hpp"
#endif

#include "PX2RobotExt.hpp"
#include "PX2AISteeringPath.hpp"
#include "PX2VoxelSection.hpp"
#include "PX2AILiDar.hpp"
#include "PX2AIAgent.hpp"
#include "PX2Arduino.hpp"
#include "PX2Renderer.hpp"
#include "PX2RobotExtData.hpp"
#include "PX2Log.hpp"
#include "PX2Application.hpp"
using namespace PX2;

//----------------------------------------------------------------------------
RobotExt *RobotExt::New()
{
	RobotExt *obj = new0 RobotExt();
	return obj;
}
//----------------------------------------------------------------------------
void RobotExt::Delete(RobotExt* robot)
{
	PX2_LOG_INFO("RobotExt Delete 0");

#if defined ROBOTEXT_SLAMGMAPPING
	PX2_SlamGMappingM.IsThreadRunning = false;
#endif

	PX2_LOG_INFO("RobotExt Delete 1");

	delete0(robot);

	PX2_LOG_INFO("RobotExt Delete 1.5");

	Robot::Set(0);

	PX2_LOG_INFO("RobotExt Delete 2");
}
//----------------------------------------------------------------------------
const float DT = 0.1f;
//----------------------------------------------------------------------------
RobotExt::RobotExt()
{
	mRunSpeedTimer = 0.0f;
	mSendRobotToDoTimer = 0.0f;

	mMaxSpeed = 0.2f;
	mRadius = 0.2f;
	mPredictTime = 2.0f;

	mMaxAccelerate = 0.2f;
	mRadius = 0.16f;
}
//----------------------------------------------------------------------------
RobotExt::~RobotExt()
{
}
//----------------------------------------------------------------------------
void RobotExt::GoTarget(const APoint &targetPos, PathType type)
{
	if (mMapDataPtr)
	{
		memset(&(mMapDataPtr->PathFinderLineDraw[0]), 0,
			(int)mMapDataPtr->PathFinderLineDraw.size());

		mIsSlamMapDataChanged = true;
	}

	mGoTargetPos = targetPos;

	std::vector<PathingNode*> vec;
	AISteeringPath aiAgentPath;
	std::vector<APoint> targetPoints;

	int posIndex = 0;
	APoint firstGoPos;

	if (type == PT_A)
	{
		mCurPathPlan = mPathGraph->FindPath(mPosition, mGoTargetPos);
		if (mCurPathPlan)
		{
			PathingNodeList list = mCurPathPlan->m_path;

			std::list<PathingNode*>::iterator it = list.begin();
			for (; it != list.end(); it++)
			{
				PathingNode *node = *it;

				APoint pos = node->GetPos();
				targetPoints.push_back(pos);

				posIndex++;

				if (2 == posIndex)
				{
					firstGoPos = pos;
				}
			}

			mCurPathPlan->ResetPath();
		}
	}

	targetPoints = _AdjustPoints(targetPoints);

	for (int i = 0; i < (int)targetPoints.size(); i++)
	{
		const APoint &pos = targetPoints[i];
		aiAgentPath.AddWayPoint(pos);
		SetPathFinderLineValueAtPos(pos, 0.05f, 200);
	}

	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		if (mAgent)
		{
			mAgent->SetPath(aiAgentPath);
		}

		if (posIndex >= 2)
		{
			AVector direction = firstGoPos - mPosition;
			float dist = direction.Normalize();
			if (dist > 0.0f)
			{
				AdjustToDirection(direction);
			}
		}
	}
	else
	{
		EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
		if (cnt)
		{
			APoint tgp = targetPos;

			RobotToDoStruct roToDo;
			roToDo.TargetPos = tgp;
			roToDo.Type = 2;
			cnt->SendRobotToDo(roToDo);
		}
	}
}
//----------------------------------------------------------------------------
void RobotExt::ClearPathFinder()
{
	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		if (mAgent)
		{
			AISteeringPath aiAgentPath;
			mAgent->SetPath(aiAgentPath);
		}

		if (mMapDataPtr)
		{
			memset(&(mMapDataPtr->PathFinderLineDraw[0]), 0,
				(int)mMapDataPtr->PathFinderLineDraw.size());

			mIsSlamMapDataChanged = true;
		}
	}
	else if (RT_CONNECTOR == mRoleType)
	{
		EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
		if (cnt)
		{
			RobotToDoStruct roToDo;
			roToDo.Type = 3;
			cnt->SendRobotToDo(roToDo);
		}
	}
}
//----------------------------------------------------------------------------
bool RobotExt::SaveMap(const std::string &filename)
{
	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		_UpdateMapDataToPathFinder();

		return true;
	}
	else if (RT_CONNECTOR == mRoleType)
	{
		EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
		if (cnt)
		{
			RobotToDoStruct roToDo;
			roToDo.Type = 4;
			cnt->SendRobotToDo(roToDo);
			return true;
		}
	}

	return true;
}
//----------------------------------------------------------------------------
bool RobotExt::LoadMap(const std::string &filename,
	const APoint &initPos, float curAngle)
{
	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		return true;
	}
	else
	{
		EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
		if (cnt)
		{
			RobotToDoStruct roToDo;
			roToDo.Type = 5;
			cnt->SendRobotToDo(roToDo);
			return true;
		}
	}

	return true;
}
//----------------------------------------------------------------------------
void RobotExt::SetObstMapValueAtPos(const APoint &pos, float range, int val)
{
	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		if (!mMapDataPtr)
			return;

		if (0 == mMapDataPtr->SelfDrawMapData2D.size())
			return;

		_SetPosVal(mMapDataPtr->SelfDrawMapData2D, pos, range, val);

		mIsSlamMapDataChanged = true;
	}
	else if (RT_CONNECTOR == mRoleType)
	{
		EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
		if (cnt)
		{
			RobotToDoStruct stru;
			stru.Type = 6;
			stru.TargetPos = pos;
			stru.Range = range;
			stru.Value = val;
			cnt->SendRobotToDo(stru);
		}
	}
}
//----------------------------------------------------------------------------
std::vector<APoint> RobotExt::_AdjustPoints(const std::vector<APoint> &poses)
{
	std::vector<APoint> posesRet;
	int fromSize = (int)poses.size();
	for (int i = 0; i < fromSize; i++)
	{
		if (i < fromSize - 1)
		{
			APoint curPos = poses[i];
			posesRet.push_back(curPos);

			AVector diff = poses[i+1] - curPos;
			float length = diff.Normalize();
			if (length > 1.0f)
			{
				int numPos = (int)length;
				for (int j = 1; j <= numPos; j++)
				{
					APoint tPos = curPos + diff * j;
					posesRet.push_back(tPos);
				}
			}
		}
		else
		{
			posesRet.push_back(poses[i]);
		}
	}

	return posesRet;
}
//----------------------------------------------------------------------------
void RobotExt::Update(float appseconds, float elpasedSeconds)
{
	Robot::Update(appseconds, elpasedSeconds);

	if (RT_MASTER == mRoleType || RT_CONNECTOR_CALCULATE == mRoleType)
	{
		if (mIsAdjustToDirection)
		{
			if (!_IsInRightDirection(mAdjustToDirection))
			{
				_UpdateAdjustDirection(mAdjustToDirection, elpasedSeconds);
			}
			else
			{
				mIsAdjustToDirection = false;
				_Stop();
			}
		}

#if defined ROBOTEXT_SLAMGMAPPING
		PX2_SlamGMappingM.SetGuessPos(mPosition.X(), mPosition.Y(), mRotationRad);
#endif
	}

	if (RT_MASTER == mRoleType)
	{
		if (!mIsAdjustToDirection)
			_UpdateVirtualRobot1(elpasedSeconds);
	}
	else if (RT_CONNECTOR == mRoleType)
	{
		if (mIsNeedSendSpeed)
		{
			//mSendRobotToDoTimer += elpasedSeconds;
			//if (mSendRobotToDoTimer > 0.2f)
			{
				//_RunSpeed(elpasedSeconds);

				// send run
				EngineClientConnector *cnt = PX2_APP.GetEngineClientConnector();
				if (cnt)
				{
					RobotToDoStruct roToDo;
					roToDo.Type = 1;
					roToDo.LeftSpeed = mLeftSpeed;
					roToDo.RightSpeed = mRightSpeed;
					cnt->SendRobotToDo(roToDo);
				}

				mSendRobotToDoTimer = 0.0f;
			}

			mIsNeedSendSpeed = false;
		}
	}
	else if (RT_MASTER_ONLY_SENDLIDAR == mRoleType)
	{
		//if (!mIsAdjustToDirection)
		//	_UpdateVirtualRobot1(elpasedSeconds);
	}
	if (RT_CONNECTOR_CALCULATE == mRoleType)
	{
		if (!mIsAdjustToDirection)
			_UpdateVirtualRobot1(elpasedSeconds);
	}

	if (RT_MASTER == mRoleType || RT_CONNECTOR == mRoleType ||
		RT_CONNECTOR_CALCULATE == mRoleType)
	{
		_UpdateLidarDataToMapData();

		// to do: arm
		if (mIsNeedUpdateTexture)
			_UpdateTex();
	}
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateTransform(float elapsedSeconds)
{
	PX2_UNUSED(elapsedSeconds);

	// Slam2D
	mPosition = m2DSlamPos;
	mRotationRad = m2DSlamAngle;

	mMapDataPtr->MapStruct.CurPos = mPosition;
	mMapDataPtr->MapStruct.CurAngle = mRotationRad;

	mDirection = AVector(-Mathf::Sin(mRotationRad), Mathf::Cos(mRotationRad), 0.0f);
	mRight = mDirection.Cross(AVector::UNIT_Z);
	mUp = AVector::UNIT_Z;
	mMatrix = HMatrix(mRight, mDirection, mUp, mPosition, true);

	if (mLiDar)
		mLiDar->SetCurTransform(mMatrix);

	_UpdateRobotTex(mRobotTex, mPosition, mRotationRad);
	_UpdateRobotTex(mRobotTexSlam, m2DSlamPos, m2DSlamAngle);
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateVirtualRobot1(float elaplseSeconds)
{
	if (!mAgent)
		return;

	AISteeringPath &steerPath = mAgent->GetSteeringBehavior()->GetPath();

	std::vector<std::vector<RobotState> > outRobotStates;

	int selectIndex = -1;
	if (!steerPath.Finished() && mCurPathPlan)
	{
		// run
		APoint curWayPoint = steerPath.CurrentWaypoint();
		Vector2f curWayPoint2 = curWayPoint.To2();

		float nearRadius = GetRadius()*4.0f;
		std::vector<Vector2f> obsts = GetNearObst(nearRadius);

		RobotState ste = _GetCurRobotState();
		bool isHasSpeedPlan = false;
		std::vector<float> selectPlan = DynamicWindowApproach(ste,
			curWayPoint2, obsts, outRobotStates, isHasSpeedPlan);

		if (isHasSpeedPlan)
		{
			float leftSpeed = selectPlan[0];
			float rightSpeed = selectPlan[1];
			selectIndex = selectPlan[2];

			mLeftSpeed = leftSpeed;
			mRightSpeed = rightSpeed;
		}
		else
		{	
			mLeftSpeed = 0.0;
			mRightSpeed = 0.0;
	
			// turn round
			// AVector wayDir = curWayPoint - GetPosition();
			// wayDir.Normalize();
			// AVector dir = GetDirection();
			// AVector cross = dir.Cross(wayDir);
			// float maxSpeed = mAgent->GetMaxSpeed();
			// if (cross.Z() >= 0.0f) 
			// { // go right
				// mLeftSpeed = maxSpeed * 0.5f;
				// mLeftSpeed = -maxSpeed * 0.5f;
			// }
			// else
			// { // go left
				// mLeftSpeed = -maxSpeed * 0.5f;
				// mLeftSpeed = maxSpeed * 0.5f;
			// }
		}
	}
	else
	{
		// stop
		if (mCurPathPlan)
		{
			_Stop();

			mCurPathPlan = 0;
		}
	}

	_RunSpeed(elaplseSeconds);

	// draw paths
	for (int i = 0; i < (int)outRobotStates.size(); i++)
	{
		std::vector<RobotState> states = outRobotStates[i];
		if (states.size() > 1)
		{
			Float4 addColor = Float4::GREEN;
			if (selectIndex == i)
				addColor = Float4::RED;

			for (int j = 0; j < (int)states.size() - 1; j++)
			{
				RobotState roS0 = states[j];
				RobotState roS1 = states[j + 1];

				APoint pos0(roS0.PosX, roS0.PosY, 0.1f);
				APoint pos1(roS1.PosX, roS1.PosY, 0.1f);

				//EngineSceneCanvas::GetSingleton().AddDebugLine(pos0, pos1,
				//	addColor);
			}
		}
	}
}
//----------------------------------------------------------------------------
void RobotExt::_RunSpeed(float elaplseSeconds)
{
	float runTime = 0.0f;
	bool isRunedArduino = false;
	if (mArduino && mArduino->IsInitlized())
	{
		mRunSpeedTimer += elaplseSeconds;
		if (mRunSpeedTimer > 0.1f)
		{
			mPosition = m2DSlamPos;
			mRotationRad = m2DSlamAngle;
			mRobotState = _GetCurRobotState();

			LeftRunSpeed(mLeftSpeed);
			RightRunSpeed(mRightSpeed);

			runTime = mRunSpeedTimer;

			isRunedArduino = true;

			mRunSpeedTimer = 0.0f;
		}
	}
	else
	{
		runTime = elaplseSeconds;
	}

	if (runTime > 0.0f && (mLeftSpeed != 0.0f || mRightSpeed!=0.0f))
	{
		RobotState rs = Motion(mRobotState, mLeftSpeed, mRightSpeed,
			runTime);
		mRobotState = rs;
		mPosition = APoint(rs.PosX, rs.PosY, 0.0f);
		mRotationRad = rs.Orientation;
		mDirection = AVector(-Mathf::Sin(mRotationRad), Mathf::Cos(mRotationRad), 0.0f);
		mDirection.Normalize();
	}
}
//----------------------------------------------------------------------------
void RobotExt::_Stop()
{
	mArduino->Stop();
	mLeftSpeed = 0;
	mRightSpeed = 0;
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateLidarDataToMapData()
{
	if (!mLiDar || mMap2DLidarData.empty())
		return;

	int mapWidth = mMapDataPtr->MapStruct.MapWidth;
	int mapHeight = mMapDataPtr->MapStruct.MapHeight;
	float reso = mMapDataPtr->MapStruct.MapResolution;
	float allHeight = mapHeight * reso;
	float ingroeRadius = GetLidarIngoreRadius();

	int allSize = mapWidth*mapHeight;
	memset(&mMap2DLidarData[0], RobotMapData::MOT_NONE, allSize * sizeof(char));

	float ingoreLarge = GetRadius()*4.0f;

	mMap2DPoses.clear();
	std::vector<RslidarDataComplete> lidarDatas = mLiDar->GetLiDarData();
	for (int i = 0; i < (int)lidarDatas.size(); i++)
	{
		RslidarDataComplete lidarData = lidarDatas[i];

		int signal = lidarData.signal;
		float angle = lidarData.angle;
		float distance = lidarData.distance;

		if (distance > ingroeRadius && distance <ingoreLarge)
		{
			float rad = angle * Mathf::DEG_TO_RAD;
			float offsetX = Mathf::Cos(rad) * distance;
			float offsetY = Mathf::Sin(rad) * distance;

			APoint pos(offsetX, offsetY, 0.0f);
			APoint posLast = mMatrix * pos;

			SetMapValueAtPos(mMap2DLidarData, posLast, RobotMapData::MOT_OBJST);
			mMap2DPoses.push_back(Vector2f(posLast.X(), posLast.Y()));
		}
	}

#if defined _DEBUG
#else
	//RobotMapData rmd;
	//mMap2DLidarData = rmd._ExtendLarger(mMap2DLidarData, mapWidth, mapHeight,
	//	RobotMapData::MOT_OBJST);
#endif
}
//----------------------------------------------------------------------------
int RobotExt::_Roundle1(float number)
{
	return (int)((number > 0.0f) ? floor(number + 0.5f) : ceil(number - 0.5f));
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateMapDataToPathFinder()
{
	int mapWidth = mMapDataPtr->MapStruct.MapWidth;
	int mapHeight = mMapDataPtr->MapStruct.MapHeight;
	int mapAllSize = mapWidth * mapHeight;
	float resolution = mMapDataPtr->MapStruct.MapResolution;

	RobotMapData rmd;
	std::vector<unsigned char> map1 = rmd._ExtendLarger(mMapDataPtr->Map2DOrigin,
		mMapDataPtr->MapStruct.MapWidth,
		mMapDataPtr->MapStruct.MapHeight,
		RobotMapData::MOT_OBJST);

	std::vector<unsigned char> selfDraw0 = rmd._ExtendLarger(mMapDataPtr->SelfDrawMapData2D,
		mMapDataPtr->MapStruct.MapWidth,
		mMapDataPtr->MapStruct.MapHeight,
		RobotMapData::MOT_OBJST);
	std::vector<unsigned char> selfDraw1 = rmd._ExtendLarger(selfDraw0,
		mMapDataPtr->MapStruct.MapWidth,
		mMapDataPtr->MapStruct.MapHeight,
		RobotMapData::MOT_OBJST);

	// mask
	std::vector<unsigned char> maskData;
	maskData.resize(mapAllSize);
	memset(&maskData[0], 0, maskData.size());

	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			int i = y*mapWidth + x;

			bool isHasObst = false;

			int mapObstValue = map1[i];
			if (RobotMapData::MOT_OBJST == mapObstValue)
			{
				isHasObst = true;
			}

			int selfDrawValue = selfDraw1[i];
			if (RobotMapData::MOT_OBJST == selfDrawValue)
			{
				isHasObst = true;
			}
			else if (RobotMapData::MOT_NONE == selfDrawValue)
			{
				// do nothing
			}
			else if (RobotMapData::MOT_CANGO == selfDrawValue)
			{
				isHasObst = false;
			}

			int gw = mPathGraph->GetMapWidth();
			int gh = mPathGraph->GetMapHeight();

			float xPercent = (float)x / (float)mapWidth;
			float yPercent = (float)y / (float)mapHeight;
			float fXIndex = xPercent * (float)gw;
			float fYIndex = yPercent * (float)gh;

			int xIndex = _Roundle1(fXIndex);
			int yIndex = _Roundle1(fYIndex);

			if (isHasObst)
			{
				_SetGraphValue(xIndex, yIndex, 1000.0f);
				_SetGraphValue(xIndex - 1, yIndex, 1000.0f);
				_SetGraphValue(xIndex + 1, yIndex, 1000.0f);
				_SetGraphValue(xIndex, yIndex - 1, 1000.0f);
				_SetGraphValue(xIndex, yIndex + 1, 1000.0f);

				_SetGraphValue(xIndex - 1, yIndex - 1, 1000.0f);
				_SetGraphValue(xIndex + 1, yIndex + 1, 1000.0f);
				_SetGraphValue(xIndex - 1, yIndex + 1, 1000.0f);
				_SetGraphValue(xIndex + 1, yIndex - 1, 1000.0f);

				maskData[i] = 1;
			}
			else
			{
				_SetGraphValue(xIndex, yIndex, PATHING_DEFAULT_ARC_WEIGHT);

				maskData[i] = 0;
			}
		}
	}

	mPathGraph->SetMapData(maskData, mapWidth, mapHeight, resolution);
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateTex()
{
	if (!mIsSlamMapDataChanged)
		return;

	int mapWidth = mMapDataPtr->MapStruct.MapWidth;
	int mapHeight = mMapDataPtr->MapStruct.MapHeight;
	float resolution = mMapDataPtr->MapStruct.MapResolution;

	//if (mVoxelSection)
	//	mVoxelSection->SetAllBlocks(0);

	RobotMapData rmd;
	std::vector<unsigned char> map2DUsing;

	if (!mMapDataPtr->Map2DOrigin.empty())
	{
#if defined _DEBUG
		map2DUsing = mMapDataPtr->Map2DOrigin;
#else
		//map2DUsing = rmd._ExtendLarger(mMapDataPtr->Map2DOrigin,
		//	mapWidth, mapHeight, RobotMapData::MOT_OBJST);
		map2DUsing = mMapDataPtr->Map2DOrigin;
#endif
	}

	std::vector<unsigned char> selfDraw1 = mMapDataPtr->SelfDrawMapData2D;

	int mapAllSize = mapWidth * mapHeight;
	int robotindex = 0;
	int x = 0;
	int y = 0;
	unsigned char *dst = (unsigned char*)mTextureMap->GetData(0);
	int width = mTextureMap->GetWidth();
	int height = mTextureMap->GetHeight();
	int indexTex = 0;
	unsigned char *pixel = 0;
	int index = 0;
	bool isHasObst = false;
	int mapVal = 0;
	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			indexTex = y *mapWidth + x;
			pixel = dst + indexTex * 4;

			index = (mapHeight - 1 - y) *mapWidth + x;

			*(pixel + 0) = 0;	// b
			*(pixel + 1) = 0;	// g
			*(pixel + 2) = 0;	// r
			*(pixel + 3) = 255;

			isHasObst = false;
			mapVal = RobotMapData::MOT_CANGO;
			if (!map2DUsing.empty())
			{
				mapVal = map2DUsing[index];

				if (RobotMapData::MOT_ROBOTPOS == mapVal)
				{
					// robot pos
					robotindex = index;

					*(pixel + 0) = 255;	// b
					*(pixel + 1) = 255;	// g
					*(pixel + 2) = 255;	// r
					*(pixel + 3) = 255;

					mRobotMapX = x;
					mRobotMapY = y;
					mMapDataPtr->MapStruct.IndexX = x;
					mMapDataPtr->MapStruct.IndexY = y;
				}
				else if (RobotMapData::MOT_NONE == mapVal)
				{
					// none
					*(pixel + 0) = 0;	// b
					*(pixel + 1) = 0;	// g
					*(pixel + 2) = 0;	// r
					*(pixel + 3) = 255;

					// set empty
				}
				else if (RobotMapData::MOT_CANGO == mapVal)
				{
					// space can go
					*(pixel + 0) = 0;
					*(pixel + 1) = 255;
					*(pixel + 2) = 0;
					*(pixel + 3) = 255;

					// set space can go
				}
				else if (RobotMapData::MOT_OBJST == mapVal)
				{
					// obst
					*(pixel + 0) = 0;
					*(pixel + 1) = 0;
					*(pixel + 2) = 255;
					*(pixel + 3) = 255;

					isHasObst = true;
				}
				else
				{
					// will never go here
					*(pixel + 0) = (unsigned char)mapVal;
					*(pixel + 1) = (unsigned char)mapVal;
					*(pixel + 2) = (unsigned char)mapVal;
					*(pixel + 3) = 255;
				}
			}

			if ((int)selfDraw1.size() > 0)
			{
				mapVal = selfDraw1[index];
				if (RobotMapData::MOT_OBJST == mapVal)
				{
					*(pixel + 0) = 255;
					*(pixel + 1) = 255;
					*(pixel + 2) = 255;
					*(pixel + 3) = 255;

					isHasObst = true;
				}
				else if (RobotMapData::MOT_CANGO == mapVal)
				{
					*(pixel + 0) = 0;
					*(pixel + 1) = 255;
					*(pixel + 2) = 0;
					*(pixel + 3) = 255;

					isHasObst = false;
				}

				if (mMap2DLidarData[index] == RobotMapData::MOT_OBJST)
				{
					*(pixel + 0) = 255;
					*(pixel + 1) = 255;
					*(pixel + 2) = 0;
					*(pixel + 3) = 255;
				}

				//if (isHasObst)
				//{
					//float xPos = x * resolution - mapWidth * 0.5f * resolution;
					//float yPos = mapHeight * 0.5f * resolution - y*resolution;

					//APoint targetPos(xPos, yPos, 0.0f);
					//APoint targetPos1(xPos, yPos, 0.05f);
					//APoint targetPos2(xPos, yPos, 0.1f);

					// obst
					//if (mVoxelSection)
					//{
					//	mVoxelSection->SetBlock(targetPos, 1);
					//	mVoxelSection->SetBlock(targetPos1, 1);
					//	mVoxelSection->SetBlock(targetPos2, 1);
					//}
				//}

			}

			if ((int)mMapDataPtr->PathFinderLineDraw.size() > 0)
			{
				mapVal = mMapDataPtr->PathFinderLineDraw[index];
				if (0 != mapVal)
				{
					*(pixel + 0) = 0;
					*(pixel + 1) = 0;
					*(pixel + 2) = (unsigned char)mapVal;
					*(pixel + 3) = (unsigned char)255;
				}
			}
		}
	}

	if (mTextureMap && mapWidth > 0 && mapHeight > 0)
		Renderer::UpdateAll(mTextureMap, 0);

	mIsSlamMapDataChanged = false;
}
//----------------------------------------------------------------------------
RobotState RobotExt::Motion(RobotState curState, float leftSpeed,
	float rightSpeed, float elapsedSeconds)
{
	RobotState afterMoveState;

	float rotSpeed = rightSpeed - leftSpeed;
	float rotDist = rotSpeed * elapsedSeconds;
	float rotRad = (rotDist / mRadius);

	afterMoveState.Orientation = curState.Orientation + rotRad;
	afterMoveState.LeftSpeed = leftSpeed;
	afterMoveState.RightSpeed = rightSpeed;

	float speed = (leftSpeed + rightSpeed) / 2.0f;

	Vector2f direction(-Mathf::Sin(afterMoveState.Orientation), Mathf::Cos(afterMoveState.Orientation));
	direction.Normalize();
	Vector2f posCur = Vector2f(curState.PosX, curState.PosY) +
		direction * speed * elapsedSeconds;

	afterMoveState.PosX = posCur.X();
	afterMoveState.PosY = posCur.Y();

	return afterMoveState;
}
//----------------------------------------------------------------------------
std::vector<RobotState> RobotExt::GenerateTraj(RobotState initState,
	float leftSpeed, float rightSpeed)
{
	RobotState tempState = initState;
	std::vector<RobotState> trajectories;
	float time = 0;
	trajectories.push_back(initState);
	while (time < mPredictTime)
	{
		tempState = Motion(tempState, leftSpeed, rightSpeed, DT);
		trajectories.push_back(tempState);
		time += DT;
	}

	return trajectories;
}
//----------------------------------------------------------------------------
std::vector<float> RobotExt::CreateDW(RobotState curState)
{
	std::vector<float> dw(4);
	float tmpMinLeftSpeed = curState.LeftSpeed - mMaxAccelerate*DT;
	float tmpMaxLeftSpeed = curState.LeftSpeed + mMaxAccelerate*DT;

	float tmpMinRightSpeed = curState.RightSpeed - mMaxAccelerate*DT;
	float tmpMaxRightSpeed = curState.RightSpeed + mMaxAccelerate*DT;

	dw[0] = Mathf::Clamp(tmpMinLeftSpeed, mMinSpeed, mMaxSpeed);
	dw[1] = Mathf::Clamp(tmpMaxLeftSpeed, mMinSpeed, mMaxSpeed);

	dw[2] = Mathf::Clamp(tmpMinRightSpeed, mMinSpeed, mMaxSpeed);
	dw[3] = Mathf::Clamp(tmpMaxRightSpeed, mMinSpeed, mMaxSpeed);

	return dw;
}
//----------------------------------------------------------------------------
RobotState RobotExt::_GetCurRobotState()
{
	RobotState state;
	state.PosX = mPosition.X();
	state.PosY = mPosition.Y();
	state.LeftSpeed = mLeftSpeed;
	state.RightSpeed = mRightSpeed;
	state.Orientation = mRotationRad;

	return state;
}
//----------------------------------------------------------------------------
float RobotExt::CalcHeading(RobotState rState, const Vector2f &goal)
{
	AVector diff = APoint(goal.X(), goal.Y(), 0.0f)
		- APoint(rState.PosX, rState.PosY, 0.0f);
	float diffLength = diff.Normalize();
	if (diffLength < 0.01f)
		return 180.0f;

	AVector dir = AVector(-Mathf::Sin(rState.Orientation),
		Mathf::Cos(rState.Orientation), 0.0f);
	dir.Normalize();

	float dotVal = diff.Dot(dir);

	float rad = Mathf::ACos(dotVal);
	float degree = rad * Mathf::RAD_TO_DEG;

	return 180.0f - degree;
}
//----------------------------------------------------------------------------
float RobotExt::CalcClearance(RobotState rState, std::vector<Vector2f> &obs)
{
	float dist = 1000.0f;
	float distTemp;
	int numObs = (int)obs.size();
	for (int i = 0; i < numObs; i++)
	{
		Vector2f diff = Vector2f(rState.PosX, rState.PosY) - obs[i];
		float length = diff.Length();

		distTemp = length - mRadius;

		if (distTemp < dist)
		{
			dist = distTemp;
		}
	}

	if (dist >= mRadius)
	{
		dist = mRadius;
	}

	return dist;
}
//----------------------------------------------------------------------------
float RobotExt::CalcBreakingDist(float velo)
{
	float stopDist = 0;
	while (velo > 0)
	{
		stopDist = stopDist + velo*DT;
		velo = velo - mMaxAccelerate*DT;
	}

	return stopDist;
}
//----------------------------------------------------------------------------
std::vector<Vector2f > RobotExt::GetNearObst(float radius)
{
	return mMap2DPoses;

	if (mMap2DLidarData.empty())
		return std::vector<Vector2f >();

	std::vector<Vector2f> obsts;

	if (mMapDataPtr)
	{
		int mapWidth = mMapDataPtr->MapStruct.MapWidth;
		int mapHeight = mMapDataPtr->MapStruct.MapHeight;

		float halfMapSize = mapWidth * 0.5f;
		float resolution = mMapDataPtr->MapStruct.MapResolution;

		float fxPosStart = mPosition.X() - radius;
		int xPosStart = (int)(fxPosStart / resolution);
		xPosStart += halfMapSize;
		xPosStart = Math<int>::Clamp(xPosStart, 0, mapWidth);

		float fxPosEnd = mPosition.X() + radius;
		int xPosEnd = (int)(fxPosEnd / resolution);
		xPosEnd += halfMapSize;
		xPosEnd = Math<int>::Clamp(xPosEnd, 0, mapWidth);

		float fyPosStart = mPosition.Y() - radius;
		int yPosStart = (int)(fyPosStart / resolution);
		yPosStart = halfMapSize + yPosStart;
		yPosStart = Math<int>::Clamp(yPosStart, 0, mapHeight);

		float fyPosEnd = mPosition.Y() + radius;
		int yPosEnd = (int)(fyPosEnd / resolution);
		yPosEnd = halfMapSize + yPosEnd;
		yPosEnd = Math<int>::Clamp(yPosEnd, 0, mapHeight);

		float centerX = mPosition.X() / resolution;
		float centerY = mPosition.Y() / resolution;

		for (int y = yPosStart; y < yPosEnd; y++)
		{
			for (int x = xPosStart; x < xPosEnd; x++)
			{
				float xPos = (x - halfMapSize) * resolution;
				float yPos = (y - halfMapSize) * resolution;

				int i = y * mapWidth + x;
				int mapVal = mMap2DLidarData[i];
				int mapVal2 = mMapDataPtr->SelfDrawMapData2D[i];

				bool isHasObst = false;
				if (RobotMapData::MOT_OBJST == mapVal)
				{
					isHasObst = true;
				}
				if (RobotMapData::MOT_OBJST == mapVal2)
				{
					isHasObst = true;
				}

				if (isHasObst)
				{
					obsts.push_back(Vector2f(xPos, yPos));
				}
			}
		}
	}

	return obsts;
}
//----------------------------------------------------------------------------
bool RobotExt::_IsInRightDirection(const AVector &dir)
{
	float valDot = dir.Dot(mDirection);
	float degree = Mathf::ACos(valDot) * Mathf::RAD_TO_DEG;
	if (degree < 15.0f)
	{
		return true;
	}

	return false;
}
//----------------------------------------------------------------------------
void RobotExt::_UpdateAdjustDirection(const AVector &dir, 
	float elapsedSeconds)
{
	float valDot = dir.Dot(mDirection);
	float degree = Mathf::ACos(valDot) * Mathf::RAD_TO_DEG;
	if (degree < 15.0f)
	{
	}
	else
	{
		if (mDirection != AVector::ZERO)
		{
			float maxSpeed = mMaxSpeed * 0.3f;

			AVector right = mDirection.Cross(dir);
			if (right.Z() > 0.0f)
			{
				mLeftSpeed = -maxSpeed;
				mRightSpeed = maxSpeed;
			}
			else
			{
				mLeftSpeed = maxSpeed;
				mRightSpeed = -maxSpeed;
			}

			_RunSpeed(elapsedSeconds);
		}
	}
}
//----------------------------------------------------------------------------
std::vector<float> RobotExt::DynamicWindowApproach(RobotState rState,
	const Vector2f &target, std::vector<Vector2f> &obstacle,
	std::vector<std::vector<RobotState> > &outRobotStates,
	bool &isHasSpeedPlan)
{
	// 0:leftMin, 1:leftMax, 2:rightMin, 3:rightMax
	std::vector<float> velocityAndOmegaRange = CreateDW(rState);

	std::vector<EvaluationPara> evalParas;
	float sumHeading = 0;
	float sumClearance = 0;
	float sumSpeed = 0;

	float SAMPLING_SPEED = 0.008f;

	for (float l = velocityAndOmegaRange[0]; l < velocityAndOmegaRange[1]; l += SAMPLING_SPEED)
	{
		for (float r = velocityAndOmegaRange[2]; r < velocityAndOmegaRange[3]; r += SAMPLING_SPEED)
		{
			std::vector<RobotState> trajectories = GenerateTraj(rState, l, r);

			RobotState stateBack = trajectories.back();

			float speed = (l + r) / 2.0f;

			float tempClearance = CalcClearance(stateBack, obstacle);
			float stopDist = CalcBreakingDist(speed);
			stopDist = GetRadius() * 0.7f;

			if (tempClearance > stopDist)
			{
				outRobotStates.push_back(trajectories);

				EvaluationPara tempEvalPara;
				tempEvalPara.Index = (int)outRobotStates.size() - 1;

				tempEvalPara.DiffHeading = CalcHeading(stateBack, target);

				tempEvalPara.Clearance = tempClearance;
				tempEvalPara.Leftspped = l;
				tempEvalPara.RightSpeed = r;
				tempEvalPara.Speed = speed;

				sumSpeed += speed;
				sumHeading += tempEvalPara.DiffHeading;
				sumClearance += tempClearance;

				evalParas.push_back(tempEvalPara);
			}
		}
	}

	float selectedLeftSpeed = 0;
	float selectedRightSpeed = 0;
	int selectIndex = -1;

	std::vector<float> selVelocity;
	selVelocity.resize(3);

	if (evalParas.empty())
	{
		/*_*/
		isHasSpeedPlan = false;
	}
	else
	{
		float G = 0;
		EvaluationPara selectEva;
		for (std::vector<EvaluationPara>::iterator i = evalParas.begin(); i < evalParas.end(); i++)
		{
			float smoothHeading = i->DiffHeading / sumHeading;
			float smoothClearance = i->Clearance / sumClearance;
			float smoothVelocity = i->Speed / sumSpeed;

			const float WEIGHT_HEADING = 0.05f;
			const float WEIGHT_CLEARANCE = 0.2f;
			//const float WEIGHT_VELOCITY = 0.065f;
			const float WEIGHT_VELOCITY = 0.055f;

			float tempG = WEIGHT_HEADING*smoothHeading + WEIGHT_CLEARANCE*smoothClearance + WEIGHT_VELOCITY*smoothVelocity;
			(*i).LastValue = tempG;

			if (tempG > G)
			{
				G = tempG;
				selectedLeftSpeed = i->Leftspped;
				selectedRightSpeed = i->RightSpeed;
				selectIndex = (*i).Index;

				selectEva = *i;
			}
		}

		isHasSpeedPlan = true;
	}

	selVelocity[0] = selectedLeftSpeed;
	selVelocity[1] = selectedRightSpeed;
	selVelocity[2] = (float)selectIndex;

	return selVelocity;
}
//----------------------------------------------------------------------------