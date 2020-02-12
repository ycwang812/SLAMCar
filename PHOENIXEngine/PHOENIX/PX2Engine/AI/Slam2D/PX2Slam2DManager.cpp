// PX2Slam2DManager.cpp

#include "PX2Slam2DManager.hpp"
#include "PX2AILiDar.hpp"
#include "PX2Thread.hpp"
#include "PX2Project.hpp"
#include "PX2Robot.hpp"
#include "PX2Application.hpp"
#include "PX2DynLibManager.hpp"
#include "PX2Robot.hpp"
#include "PX2ScopedCS.hpp"
#include <iostream>
using namespace PX2;
using namespace std;

//#define PX2_SLAM2D_USE_OPENCV

#if defined PX2_SLAM2D_USE_OPENCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
//opencv_core340d.lib
//opencv_highgui340d.lib
//opencv_imgproc340d.lib
#endif

double start_mapAngle(0.0), end_mapAngle(0.0);
int first_step(-1), last_step(-1);
int multiecho_mode(0);
unsigned int speed(0), cluster_count(1);
bool get_intensities(false), get_new(false), verbose(false);

#if defined PX2_SLAM2D_USE_OPENCV
int _ShowType = 0; // 0 out, 1 internal
#else
int _ShowType = 1; // 0 out, 1 internal
#endif

bool showLiDar = false;

//----------------------------------------------------------------------------
static buffer TheBuf;
#if defined PX2_SLAM2D_USE_OPENCV
IplImage* LaserImage;
IplImage* LaserImage2;
#endif
static float curUseYSendToMap = 0.0f;
static float curY = 0.0f;
static bool isSlamAlreadyRuned = false;
//----------------------------------------------------------------------------
#if defined PX2_SLAM2D_USE_OPENCV
// 显示地图的线程last modify
void showMap()
{
	// new opencv image used for show laser points
	LaserImage = cvCreateImage(cvSize(1024, 1024), IPL_DEPTH_8U, 1);
	cvNamedWindow("Slam2D", 1);

	std::vector<unsigned char> map;
	unsigned char * pPixel = NULL;
	int x, y;
	int robotindex;
	int robotindexUse;
	float mapAngleMapUpdated = 0.0f;

	float offsetDegree = robot->GetOffsetDegree();
	const AVector &moveDir = robot->GetMoveDirection();

	while (Slam2DManager::IsRun)
	{
		int indexX = 0;
		int indexY = 0;
		float mapAngle = 0.0f;
		mgr->mHector->getLastMatchPoseMapIndex(indexX, indexY, mapAngle);
		float usemapAngle = mapAngle + offsetDegree;

		mgr->mHector->getMap(map, mapAngleMapUpdated);
		float usemapAngleMapUpdated = mapAngleMapUpdated;// +offsetDegree*Mathf::DEG_TO_RAD;

		size_t size = map.size();

		for (size_t i = 0; i < size; i++)
		{
			x = (i+1024) % 1024;
			y = (1023 - i / 1024);
			pPixel = (unsigned char*)LaserImage->imageData + y * LaserImage->widthStep + x;
			// this is not robot coordinate or not updated point
			if (map[i] != 100 && map[i] != 10)
			{
				*pPixel = map[i];
			}
			else if (map[i] == 100)
			{
				robotindex = i;
			}
			else
			{
				*pPixel = 100;
			}
		}

		// mapmapAngle
		robotindexUse = indexY * 1024 + indexX;
		int robotX = robotindexUse % 1024;
		int robotY = 1023 - robotindexUse / 1024;
		cvCircle(LaserImage, cvPoint(robotX, robotY), 3, cvScalar(255.0), -1, 8, 0);
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-usemapAngle) * 20),
			robotY + (int)(sin(-usemapAngle) * 20)), cvScalar(255.0));

		// use mapAngle send to map
		float yRad = curUseYSendToMap * Mathf::DEG_TO_RAD;
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-yRad) * 20),
			robotY + (int)(sin(-yRad) * 20)), cvScalar(255.0));

		// cur axis mapAngle
		float yRad1 = curY * Mathf::DEG_TO_RAD;
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-yRad1) * 35),
			robotY + (int)(sin(-yRad1) * 35)), cvScalar(255.0));

		float yRad1Adjust = (curY + robot->mDegreeAdjust) * Mathf::DEG_TO_RAD;
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-yRad1Adjust) * 30),
			robotY + (int)(sin(-yRad1Adjust) * 30)), cvScalar(5.0));

		// cur move mapAngle
		if (moveDir != AVector::ZERO)
		{
			cvLine(LaserImage, cvPoint(robotX, robotY),
				cvPoint(robotX + moveDir.X() * 50.0f, robotY + moveDir.Y() * 50.0f),
				cvScalar(20.0));
		}

		// first move degree
		float firstAxisY = robot->mFirstMoveDirectionAxisY;
		float firsY = firstAxisY * Mathf::DEG_TO_RAD;
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-firsY) * 60),
			robotY + (int)(sin(-firsY) * 150)), cvScalar(20.0f));

		// move degree
		float moveDegree = robot->GetMoveDirectionDegree() + robot->mFirstMoveDirectionAxisY;
		float movRad = moveDegree * Mathf::DEG_TO_RAD;
		cvLine(LaserImage, cvPoint(robotX, robotY),
			cvPoint(robotX + (int)(cos(-movRad) * 70),
			robotY + (int)(sin(-movRad) * 250)), cvScalar(10.0f));

		cvShowImage("Slam2D", LaserImage);
		cvWaitKey(10);
	}
}
//----------------------------------------------------------------------------
void showLaserDataInPic(std::vector<sData> & data)
{
	cvZero(LaserImage2);
	cvShowImage("Slam2D2", LaserImage2);
	cvWaitKey(2);

	int x, y;
	double rho;
	unsigned char * pPixel = 0;
	int halfWidth = 512;
	int halfHeight = 512;

	for (int i = 0; i < data.size(); i++)
	{
		float theta = data[i].angle *Mathf::DEG_TO_RAD;
		rho = data[i].distance;

		x = (int)rho*cos(theta) + 512;
		y = (int)-rho*sin(theta) + 512;

		if (x >= 0 && x < 1024 && y >= 0 && y < 1024)
		{
			pPixel = (unsigned char*)LaserImage2->imageData +
				y*LaserImage2->widthStep + x;
			*pPixel = 255;
		}
	}

	cvCircle(LaserImage2, cvPoint(512, 512), 3, cvScalar(255), -1, 8, 0);
	cvLine(LaserImage2, cvPoint(512, 512), cvPoint(550, 512), cvScalar(255));
	cvLine(LaserImage2, cvPoint(512, 512), cvPoint(512, 474), cvScalar(255));

	cvShowImage("Slam2D2", LaserImage2);
	cvWaitKey(2);
}
void _ThreadCallback(void*)
{
	showMap();
}
#endif
Slam2DManager *mgr = 0;
Mutex sRobotMutex;
Robot::RoleType sRobotRoleType = Robot::RT_MAX_TYPE;
void _ThreadProcessDataCallback(void*)
{
	while (Slam2DManager::IsRun)
	{
		std::vector<sData> data = TheBuf.getSData();
		if (data.size() > 0)
		{
			std::vector<_sData> dataContainer;
			{
				ScopedCS cs(&mgr->mMutexHector);
				mgr->mHector->scanCallback(data, false, dataContainer);
			}
			isSlamAlreadyRuned = true;

#if defined PX2_SLAM2D_USE_OPENCV
			if (showLiDar)
				showLaserDataInPic(dataContainer);
#endif
		}
	}
}
//----------------------------------------------------------------------------
bool Slam2DManager::IsRun = false;
//----------------------------------------------------------------------------
Slam2DManager::Slam2DManager() :
mThread(0),
mIsSlamMapInited(false),
mHector(0)
{
	mgr = Slam2DManager::GetSingletonPtr();
}
//----------------------------------------------------------------------------
Slam2DManager::~Slam2DManager()
{
}
//----------------------------------------------------------------------------
void Slam2DManager::Initlize()
{
	PX2_LOG_INFO("Slam2DManager::Initlize 0");

	PX2_EW.ComeIn(this);

	IsRun = true;

#if defined (__LINUX__) && defined (PX2_SLAM2D_USE_OPENCV)
	DynLibManager::GetSingleton().Load("libopencv_core.so");
	DynLibManager::GetSingleton().Load("libopencv_highgui.so");
	DynLibManager::GetSingleton().Load("libopencv_imgproc.so");
#endif

#if defined PX2_SLAM2D_USE_OPENCV
	if (showLiDar)
	{
		LaserImage2 = cvCreateImage(cvSize(1024, 1024), IPL_DEPTH_8U, 1);//new opencv image used for show laser points
		cvNamedWindow("Slam2D2", 1);
	}

	if (0 == _ShowType)
	{
		// new opencv image used for show laser points
		LaserImage = cvCreateImage(cvSize(1024, 1024), IPL_DEPTH_8U, 1);
		cvNamedWindow("Slam2D", 1);

		mThread = new0 Thread();
		mThread->Start(_ThreadCallback);
	}
#endif

	mHector = new HectorMapping();

	mThreadProcessData = new0 Thread();
	mThreadProcessData->Start(_ThreadProcessDataCallback);

	PX2_LOG_INFO("Slam2DManager::Initlize");
}
//----------------------------------------------------------------------------
void Slam2DManager::Terminate()
{
	sRobotRoleType = Robot::RT_MAX_TYPE;
	Slam2DManager::IsRun = false;
	System::SleepSeconds(0.2f);

	if (mThread)
	{
		//mThread->Join();
		delete0(mThread);
	}

	if (mThreadProcessData)
	{
		//mThreadProcessData->Join();
		delete0(mThreadProcessData);
	}

	if (mHector)
	{
		delete(mHector);
		mHector = 0;
	}

	PX2_EW.GoOut(this);
}
//----------------------------------------------------------------------------
static float timer = 0.0f;
static int curRDCIndex = 0;
RslidarDataComplete _GetRDC(float degree,
	std::vector<RslidarDataComplete> &lData)
{
	for (int i = curRDCIndex; i < (int)lData.size(); i++)
	{
		int nextI = i + 1;
		if (nextI >(int)lData.size() - 1)
			nextI = i;

		if (lData[i].angle <= degree && degree < lData[nextI].angle)
		{
			curRDCIndex = i;
			break;
		}
	}

	return lData[curRDCIndex];
}
//----------------------------------------------------------------------------
void Slam2DManager::_UpdateInit()
{
	int hectorMapSize =mgr->mHector->getMapSize();
	float reso =mgr->mHector->getResolution();
	Robot *robot = Robot::GetSingletonPtr();
	if (robot)
	{
		sRobotRoleType = robot->GetRoleType();

		if (!mIsSlamMapInited)
		{
			robot->InitSlamMap(hectorMapSize, hectorMapSize, reso, true);
			mIsSlamMapInited = true;
		}

		LiDar *liDar = robot->GetLidar();
		if (liDar)
		{
			mgr->mHector->setUpdateMap(robot->IsSlamMapUpdate());

			Robot::RoleType rt = robot->GetRoleType();
			if ((Robot::RT_MASTER == rt &&liDar->IsOpened())
				|| Robot::RT_CONNECTOR_CALCULATE == rt)
			{
				if (!robot->GetInitMapData().Map2DOrigin.empty())
				{
					TheBuf.clear();

					{
						ScopedCS cs(&mMutexHector);

						if (mHector)
						{
							delete mHector;
							mHector = 0;
						}

						mHector = new HectorMapping();
					}

					APoint curPos = robot->GetInitMapData().MapStruct.CurPos;
					float curAngle = robot->GetInitMapData().MapStruct.CurAngle;
					Eigen::Vector3f hitPos = Eigen::Vector3f(curPos[0], curPos[1], curAngle);
					mgr->mHector->setMap(robot->GetInitMapData().Map2DOrigin, hitPos);
					robot->ClearInitMapData();
					isSlamAlreadyRuned = true;
				}
			}
		}
	}
}
//----------------------------------------------------------------------------
void Slam2DManager::Update(double appSeconds, double elapsedSeconds)
{
	if (!Project::GetSingletonPtr())
		return;

	_UpdateInit();

	int hectormapsize =mgr->mHector->getMapSize();
	float reso =mgr->mHector->getResolution();
	Robot *robot = Robot::GetSingletonPtr();
	if (robot)
	{
		LiDar *liDar = robot->GetLidar();
		if (liDar)
		{
			curRDCIndex = 0;
			std::vector<RslidarDataComplete> lData = liDar->GetLiDarData();
			int size = lData.size();

			LiDarSlamData lsd0;
			lsd0.Datas.resize(size);
			for (int i = 0; i < (int)lData.size(); i++)
			{
				lsd0.Datas[i].angle = lData[i].angle;
				lsd0.Datas[i].signal = (uint8_t)lData[i].signal;
				lsd0.Datas[i].distance = lData[i].distance * 1000.0f;
			}

			//#define PX2_SLAM2D_USEAXIS 1
#if defined PX2_SLAM2D_USEAXIS

			float yawCrossValue = robot->GetYawCrossValue();

			LiDarSlamDatas.push_back(lsd0);

			int index = -1;
			bool doBreak = false;
			for (int i = 0; i < (int)LiDarSlamDatas.size(); i++)
			{
				LiDarSlamData &lsd = LiDarSlamDatas[i];

				std::vector<Arduino::AxisObj> &axisObjects = PX2_ARDUINO.GetAxisObjs();
				if (axisObjects.size() > 2)
				{
					for (int j = 0; j < (int)axisObjects.size() - 1; j++)
					{
						if (axisObjects[j].TStamp <= lsd.TStamp &&
							lsd.TStamp <= axisObjects[j + 1].TStamp)
						{
							index = i;
							doBreak = true;

							float yaw = (axisObjects[j].Y + axisObjects[j + 1].Y)*0.5f;
							curY = PX2_ARDUINO.GetCurAxisObj().Y + offsetDegree;

#if defined PX2_SLAM2D_USEAXIS_YAW
							if (Mathf::FAbs(axisObjects[j].AZ) < 15 && Mathf::FAbs(axisObjects[j + 1].AZ) < 15)
							{
								for (int i = 0; i < (int)lsd.Datas.size(); i++)
								{
									lsd.Datas[i].angle = lsd.Datas[i].angle + yaw * yawCrossValue;
								}
								buf.put(lsd.Datas);
								curUseYSendToMap = yaw + offsetDegree;
								robot->SetCurMapUsingAxisWithOffset(curUseYSendToMap);
							}

#else
							for (int i = 0; i < (int)lsd.Datas.size(); i++)
							{
								lsd.Datas[i].angle = lsd.Datas[i].angle;
							}
							buf.put(lsd.Datas);
							curUseYSendToMap = yaw + offsetDegree;
							robot->SetCurMapUsingAxisWithOffset(curUseYSendToMap);
#endif
						}
					}
				}
				else
				{
					LiDarSlamDatas.clear();
				}

				if (doBreak)
				{
					break;
				}
			}

			if (index >= 0)
			{
				std::vector<LiDarSlamData> datas;
				for (int i = index + 1; i < (int)LiDarSlamDatas.size(); i++)
				{
					datas.push_back(LiDarSlamDatas[i]);
				}
				LiDarSlamDatas = datas;
			}
			// end refresh
#else
			if (Robot::RT_MASTER == sRobotRoleType || Robot::RT_CONNECTOR_CALCULATE == sRobotRoleType)
			{
				if (lsd0.Datas.size() > 0)
				{
					TheBuf.put(lsd0.Datas);
				}
			}
#endif

			if (1 == _ShowType)
			{
				std::vector<unsigned char> map;
				float mapAngleMap = 0.0f;
				mgr->mHector->getMap(map, mapAngleMap);

				robot->SetSlam2DMap(map);
			}

			if (isSlamAlreadyRuned)
			{
				int indexX = 0;
				int indexY = 0;
				float angle = 0;
				mgr->mHector->getLastMatchPoseMapIndex(indexX, indexY, angle);
				int hector_slam_mapSize =mgr->mHector->getMapSize();
				float reso =mgr->mHector->getResolution();
				float posX = ((float)indexX - hector_slam_mapSize*0.5f) * reso;
				float posY = ((float)indexY - hector_slam_mapSize*0.5f) * reso;
				robot->SetSlam2DPosition(APoint(posX, posY, 0.0f), angle);
			}
		}
	}
	else
	{
		sRobotRoleType = Robot::RT_MAX_TYPE;
	}
}
//----------------------------------------------------------------------------