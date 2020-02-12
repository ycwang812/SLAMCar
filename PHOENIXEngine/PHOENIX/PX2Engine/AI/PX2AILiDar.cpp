// PX2AILiDar.cpp

#include "PX2AILiDar.hpp"
#include "C3iroboticsLidar2.h"
#include "C3iroboticsLidar.h"
#include "PX2Log.hpp"
#include "PX2LidarSerialConnection.hpp"
#include "PX2Renderer.hpp"
#include "PX2StringHelp.hpp"
#include "PX2System.hpp"
#include "PX2ScopedCS.hpp"
#include "PX2GraphicsRoot.hpp"
#include "PX2GraphicsEventType.hpp"
#include "PX2NetClientConnector.hpp"
#include "PX2AIES.hpp"
#include "PX2WRLidar.hpp"
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
#include "rptypes.h"
#include "rplidar_cmd.h"
#include "rplidar_driver.h"
#include "hal/locker.h"
using namespace rp::standalone::rplidar;
#endif
using namespace PX2;
using namespace everest::hwdrivers;


//----------------------------------------------------------------------------
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
rplidar_response_device_info_t rplidarInfo;
std::vector<RplidarScanMode> modeVec_;
_u16 usingScanMode_ = 0;
#endif
void _ThreadLiDarReadCallback(void* ld)
{
	LiDar *lidar = (LiDar*)(ld);
	Mutex &mutex = lidar->GetMutexOpenClose();

	while (lidar && lidar->IsOpened())
	{
		ScopedCS cs(&mutex);
		lidar->GetSlamData();
	}
}
//----------------------------------------------------------------------------
LiDar::LiDar():
mDeviceConnectin(0),
mLiDarIII(0),
mLiDarIII3(0),
mLiDarWR(0),
mIsCurTransformLost(false),
mIsTransformUpdate(false),
mIsConnected(false),
mIsHasDataNew(false),
mIsSupportControlMoto(false),
mIsNeedUpdateTexture(true)
{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
	mLidar_drv = 0;
#endif

	mOffsetDegree = -90.0f;

	mLiDarType = LT_III;
	mLiDarIII = new everest::hwdrivers::C3iroboticsLidar2();
	mLiDarIII3 = new everest::hwdrivers::C3iroboticsLidar();
	mClientConnector = new0 ClientConnector(5);
	mLiDarWR = new0 WRLidar();

	mThread = new0 Thread();
	
	int width = 256;
	int height = 256;

	// leidar
	Texture::Format formatLidar = Texture::TF_A8R8G8B8;
	mTextureLiDar = new0 Texture2D(formatLidar, width, height, 1);
	char *pDestLiDar = mTextureLiDar->GetData(0);
	int offsetDstLidar = 0;
	for (int row = 0; row < width; ++row)
	{
		for (int col = 0; col < height; ++col)
		{
			pDestLiDar[offsetDstLidar + 0] = 0; // b
			pDestLiDar[offsetDstLidar + 1] = 100; // g
			pDestLiDar[offsetDstLidar + 2] = 0; // r
			pDestLiDar[offsetDstLidar + 3] = 255;

			offsetDstLidar += 4;
		}
	}
	mPicBoxLiDar = new0 UIFPicBox();
	mPicBoxLiDar->GetUIPicBox()->SetTexture(mTextureLiDar);
	mPicBoxLiDar->GetUIPicBox()->GetMaterialInstance()
		->GetMaterial()->GetPixelShader(0, 0)->SetFilter(0,
		Shader::SF_NEAREST);

	mMatrix.MakeIdentity();
	mMatrixRot.MakeIdentity();

	ComeInEventWorld();
}
//----------------------------------------------------------------------------
LiDar::~LiDar()
{
	PX2_LOG_INFO("Delete LiDar0");

	GoOutEventWorld();

	PX2_LOG_INFO("Delete LiDar1");

	Close();

	PX2_LOG_INFO("Delete LiDar2");

	System::SleepSeconds(1.0f);

	PX2_LOG_INFO("Delete LiDar3");

	//mThread->Join();
	mThread = 0;

	PX2_LOG_INFO("Delete LiDar4");

	// III
	if (mLiDarIII)
	{
		delete(mLiDarIII);
		mLiDarIII = 0;
	}
	if (mLiDarIII3)
	{
		delete(mLiDarIII3);
		mLiDarIII3 = 0;
	}
	
	PX2_LOG_INFO("Delete LiDar5");

	if (mDeviceConnectin)
	{
		delete(mDeviceConnectin);
		mDeviceConnectin = 0;
	}

	PX2_LOG_INFO("Delete LiDar6");

	if (mLiDarWR)
	{
		mLiDarWR->Stop();
		delete0(mLiDarWR);
		mLiDarWR = 0;
	}

	PX2_LOG_INFO("Delete LiDar7");

#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
	if (mLidar_drv)
	{
		mLidar_drv->stop();
		rp::standalone::rplidar::RPlidarDriver::DisposeDriver(mLidar_drv);
	}
#endif

	PX2_LOG_INFO("Delete LiDar8");

	if (mClientConnector)
	{
		mClientConnector->Disconnect();
		delete0(mClientConnector);
	}

	PX2_LOG_INFO("Delete LiDar9");
}
//----------------------------------------------------------------------------
Mutex &LiDar::GetMutexOpenClose()
{
	return mMutexOpenClose;
}
//----------------------------------------------------------------------------
LiDar *LiDar::New()
{
	return new0 LiDar();
}
//----------------------------------------------------------------------------
void LiDar::SetLiDarType(LiDarType type)
{
	mLiDarType = type;
	mIsConnected = false;
	mIsHasDataNew = false;
}
//----------------------------------------------------------------------------
LiDar::LiDarType LiDar::GetLiDarType() const
{
	return mLiDarType;
}
//----------------------------------------------------------------------------
static char *m_receiveBuffer;
static int m_numberOfBytesInReceiveBuffer = 0;
//----------------------------------------------------------------------------
void _LiDarClientConnectorRecvCallback1(uint8_t* buffer, uint32_t numOfBytes)
{
	bool beVerboseHere = false;

	uint32_t remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
	uint32_t bytesToBeTransferred = numOfBytes;
	if (remainingSpace < numOfBytes)
	{
		bytesToBeTransferred = remainingSpace;
	}
	else
	{
	}

	if (bytesToBeTransferred > 0)
	{
		// Data can be transferred into our input buffer
		memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
		m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;
	}
	else
	{
		// There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
		// Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
		// we clear the input buffer here.
		m_numberOfBytesInReceiveBuffer = 0;
	}
}
void _LiDarClientConnectorRecvCallback(ClientConnector *cn, const std::string &recvData)
{
	_LiDarClientConnectorRecvCallback1((uint8_t*)recvData.c_str(),
		(uint32_t)recvData.length());
}
//----------------------------------------------------------------------------
void LiDar::SetNeedUpdateTexture(bool up)
{
	mIsNeedUpdateTexture = up;
}
//----------------------------------------------------------------------------
bool LiDar::IsNeedUpdateTexture() const
{
	return mIsNeedUpdateTexture;
}
//----------------------------------------------------------------------------	
bool LiDar::Open(const std::string &portIP, int baudratePort)
{
	mPortIP = portIP;
	mBaudratePort = baudratePort;

	std::string lastPort = portIP;
	transform(lastPort.begin(), lastPort.end(), lastPort.begin(), ::tolower);

	mIsHasDataNew = false;
	mIsSupportControlMoto = false;

	if (LT_III == mLiDarType)
	{
		if (mLiDarIII)
		{
			if (mDeviceConnectin)
			{
				mDeviceConnectin->close();
				delete(mDeviceConnectin);
			}

			SerialConnection *cnt = new SerialConnection();
			cnt->SetParam(portIP, baudratePort);
			mDeviceConnectin = cnt;
			if (mDeviceConnectin->openSimple())
			{
				if (mDeviceConnectin)
				{
					if (mLiDarIII->initilize(mDeviceConnectin))
					{
						mThread->Start(_ThreadLiDarReadCallback, this);
						mIsConnected = true;

						Event *ent = PX2_CREATEEVENTEX(AIES, LiDarOpen);
						ent->SetDataStr0(mPortIP);
						PX2_EW.BroadcastingLocalEvent(ent);

						return true;
					}
				}
			}
		}

		return false;
	}
	else if (LT_III3 == mLiDarType)
	{
		if (mLiDarIII3)
		{
			if (mDeviceConnectin)
			{
				mDeviceConnectin->close();
				delete(mDeviceConnectin);
			}

			SerialConnection *cnt = new SerialConnection();
			cnt->SetParam(portIP, baudratePort);
			mDeviceConnectin = cnt;
			if (mDeviceConnectin->openSimple())
			{
				if (mDeviceConnectin)
				{
					if (mLiDarIII3->initilize(mDeviceConnectin))
					{
						TLidarError retvalue;
						retvalue = mLiDarIII3->setLidarWorkMode(HIGHSPEED_SCAN);
						if (retvalue == EXECUTE_SUCCESS)
						{
							PX2_LOG_INFO("Lidar %s", "setLidarWorkMode suc");
						}

						System::SleepSeconds(2.0);

						mLiDarIII3->setLidarRotationlSpeed(15);

						mThread->Start(_ThreadLiDarReadCallback, this);
						mIsConnected = true;

						Event *ent = PX2_CREATEEVENTEX(AIES, LiDarOpen);
						ent->SetDataStr0(mPortIP);
						PX2_EW.BroadcastingLocalEvent(ent);

						return true;
					}
				}
			}
		}

		return false;
	}
	else if (LT_RP == mLiDarType)
	{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
		if (!mLidar_drv)
			mLidar_drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

#if defined _WIN32 || defined WIN32
		std::size_t it = lastPort.find("com");
		if (it != std::string::npos)
		{
			std::string subStr = lastPort.substr(it + 3, portIP.size() - it - 3);
			int numVal = StringHelp::StringToInt(subStr);
			if (numVal >= 10)
			{
				lastPort = std::string("\\\\.\\") + portIP;
			}
		}
#endif

		PX2_LOG_INFO("rplidar try to connect %s at baud %d", portIP.c_str(), baudratePort);

		if (IS_FAIL(mLidar_drv->connect(portIP.c_str(), baudratePort)))
		{
			PX2_LOG_INFO("rplidar connect failed");

			return false;
		}

		PX2_LOG_INFO("rplidar connectted");

#if defined _DEBUG
		_InitLiDar(false);
		_InitLiDar1();
#else
		PX2_GR.SendGeneralEvent("LIDARINIT", 1.0f);
#endif

#endif
	}
	else if (LT_SICK == mLiDarType)
	{
		mClientConnector->Disconnect();

		mClientConnector->AddRawRecvCallback(_LiDarClientConnectorRecvCallback);
		mClientConnector->ConnectB(portIP, baudratePort);
		if (CONNSTATE_CONNECTED == mClientConnector->GetConnectState())
		{
			PX2_LOG_INFO("Connected!");

			// 启动是1，关闭是0
			std::string buf("<\x02sEN LMDscandata 1\x03\0");
			mClientConnector->GetSocket().SendBytes(buf.c_str(), buf.length());
		}
	}
	else if (LT_WR == mLiDarType)
	{
		mLiDarWR->Start(portIP, baudratePort);
		
		mThread->Start(_ThreadLiDarReadCallback, this);
		
		mIsConnected = true;
	}

	return true;
}
//----------------------------------------------------------------------------
void LiDar::SetOffsetDegree(float offsetDegree)
{
	mOffsetDegree = offsetDegree;
}
//----------------------------------------------------------------------------
bool LiDar::_CheckDeviceHealth(int * errorCode)
{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
	int errcode = 0;
	bool ans = false;

	do 
	{
		if (!mIsConnected) {
			errcode = RESULT_OPERATION_FAIL;
			break;
		}

		rplidar_response_device_health_t healthinfo;
		if (IS_FAIL(mLidar_drv->getHealth(healthinfo)))
		{
			errcode = RESULT_OPERATION_FAIL;
			break;
		}

		if (healthinfo.status != RPLIDAR_STATUS_OK) 
		{
			errcode = healthinfo.error_code;
			break;
		}

		ans = true;
	} while (0);

	if (errorCode) 
		*errorCode = errcode;
	return ans;

#else
	return false;
#endif
}
//----------------------------------------------------------------------------
bool LiDar::IsOpened() const
{
	if (LT_III == mLiDarType)
	{
		if (mDeviceConnectin)
		{
			return CDeviceConnection::STATUS_OPEN == mDeviceConnectin->getStatus();
		}
	}
	else
	{
		return mIsConnected;
		// this will stop
		//return _CheckDeviceHealth();
	}

	return false;
}
//----------------------------------------------------------------------------
void LiDar::Close()
{
	ScopedCS cs(&mMutexOpenClose);

	if (LT_III == mLiDarType)
	{
		if (mDeviceConnectin)
		{
			mDeviceConnectin->close();
			delete(mDeviceConnectin);
			mDeviceConnectin = 0;

			Event *ent = PX2_CREATEEVENTEX(AIES, LiDarClose);
			PX2_EW.BroadcastingLocalEvent(ent);
		}
	}
	else if (LT_RP == mLiDarType)
	{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
		if (mIsConnected) 
		{
			mLidar_drv->stop();

			Event *ent = PX2_CREATEEVENTEX(AIES,LiDarClose);
			PX2_EW.BroadcastingLocalEvent(ent);
		}
#endif
	}
	else if (LT_WR == mLiDarType)
	{
		mLiDarWR->Stop();

		Event *ent = PX2_CREATEEVENTEX(AIES, LiDarClose);
		PX2_EW.BroadcastingLocalEvent(ent);
	}

	mIsHasDataNew = false;
	mIsSupportControlMoto = false;
}
//----------------------------------------------------------------------------
void LiDar::SetCurSlam3DTransformLost(bool lost)
{
	mIsCurTransformLost = lost;
}
//----------------------------------------------------------------------------
void LiDar::SetCurTransform(HMatrix &mat)
{
	mMatrix = mat;
	AVector right;
	AVector direction;
	AVector up;
	APoint pos;

	mMatrix.GetRow(0, right);
	mMatrix.GetRow(1, direction);
	mMatrix.GetRow(2, up);
	mMatrix.GetRow(3, pos);

	mMatrixRot = HMatrix(right, direction, up, APoint::ORIGIN, true);

	mCurPos = pos;
	mCurDirection = direction;

	mIsTransformUpdate = true;
}
//----------------------------------------------------------------------------
const APoint &LiDar::GetCurPos() const
{
	return mCurPos;
}
//----------------------------------------------------------------------------
const AVector &LiDar::GetCurDirection() const
{
	return mCurDirection;
}
//----------------------------------------------------------------------------
void LiDar::GetSlamData()
{
	if (!mIsConnected)
		return;

	std::vector<RslidarDataComplete> lidarDataThread;

	bool isHanNewData = false;
	if (LT_III == mLiDarType)
	{
		if (!mLiDarIII || !mDeviceConnectin)
			return;

		int status = mDeviceConnectin->getStatus();
		if (CDeviceConnection::STATUS_OPEN != status)
			return;

		TLidarGrabResult2 result = mLiDarIII->getScanData();
		switch (result)
		{
		case LIDAR_GRAB_ING:
		{
			break;
		}
		case LIDAR_GRAB_SUCESS:
		{
			TLidarScan2 lidar_scan = mLiDarIII->getLidarScan();
			size_t lidar_scan_size = lidar_scan.getSize();

			lidarDataThread.resize(lidar_scan_size);
			RslidarDataComplete one_lidar_data;
			for (size_t i = 0; i < lidar_scan_size; i++)
			{
				one_lidar_data.signal = (uint8_t)lidar_scan.signal[i];
				one_lidar_data.angle = 360.0 - lidar_scan.angle[i];
				one_lidar_data.distance = lidar_scan.distance[i];
				lidarDataThread[i] = one_lidar_data;
			}

			isHanNewData = true;

			break;
		}
		case LIDAR_GRAB_ERRO:
		{
			break;
		}
		case LIDAR_GRAB_ELSE:
		{
			PX2_LOG_INFO("[Main] LIDAR_GRAB_ELSE!\n");
			break;
		}
		}
	}
	else if (LT_III3 == mLiDarType)
	{
		if (!mLiDarIII3 || !mDeviceConnectin)
			return;

		int status = mDeviceConnectin->getStatus();
		if (CDeviceConnection::STATUS_OPEN != status)
			return;

		mLiDarIII3->ThreadCall();

		CLidarDynamicScan dyn = mLiDarIII3->getLidarDynamicScan();

		TLidarGrabResult result = dyn.getGrabResult();
		switch (result)
		{
		case LIDAR_GRAB_ING:
		{
			break;
		}
		case LIDAR_GRAB_SUCESS:
		{
			size_t lidar_scan_size = dyn.getSize();

			lidarDataThread.resize(lidar_scan_size);
			RslidarDataComplete one_lidar_data;
			for (size_t i = 0; i < lidar_scan_size; i++)
			{
				one_lidar_data.signal = (uint8_t)255;

				if (0<=i && i<lidar_scan_size)
					one_lidar_data.angle = 360.0 - dyn.m_angle[i];

				if (0 <= i && i<lidar_scan_size)
					one_lidar_data.distance = dyn.m_distance[i];

				if (0<=i && i<(int)lidar_scan_size)
					lidarDataThread[i] = one_lidar_data;
			}

			isHanNewData = true;

			break;
		}
		case LIDAR_GRAB_ERRO:
		{
			break;
		}
		case LIDAR_GRAB_ELSE:
		{
			PX2_LOG_INFO("[Main] LIDAR_GRAB_ELSE!\n");
			break;
		}
		}
	}
	else if (LT_RP == mLiDarType)
	{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
		if (!mLidar_drv)
			return;

		rplidar_response_measurement_node_t nodes[8192];
		size_t cnt = (sizeof(nodes) / sizeof(nodes[0]));

		if (IS_OK(mLidar_drv->grabScanData(nodes, cnt, 0)))
		{
			lidarDataThread.clear();
			for (int i = 0; i < (int)cnt; i++)
			{
				RslidarDataComplete one_lidar_data;

				one_lidar_data.signal = (nodes[i].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
				one_lidar_data.angle = (nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f + mOffsetDegree;
				one_lidar_data.distance = (float)(nodes[i].distance_q2 / 4.0f * 0.001f);

				one_lidar_data.angle = 360.0 - one_lidar_data.angle;

				lidarDataThread.push_back(one_lidar_data);
			}

			if (!lidarDataThread.empty())
			{
				isHanNewData = true;
			}
		}
#endif
	}
	else if (LT_WR == mLiDarType)
	{
		lidarDataThread = mLiDarWR->GetRDCS();
		isHanNewData = true;
	}

	//std::sort(lidarDataThread.begin(), lidarDataThread.end(), RslidarDataComplete::LessThan);

	if (isHanNewData)
	{
		ScopedCS cs(&mMutex);
		mLidarDatTimestamp = Timestamp();
		SetLiData(lidarDataThread);
		isHanNewData = false;
	}
}
//----------------------------------------------------------------------------
void LiDar::OnEvent(Event *ent)
{
	if (GraphicsES::IsEqual(ent, GraphicsES::GeneralString))
	{
		std::string str = ent->GetDataStr0();
		if (str == "LIDARINIT")
		{
			_InitLiDar(true);
		}
		else if (str == "LIDARINIT1")
		{
			_InitLiDar1();
		}
	}
}
//----------------------------------------------------------------------------
void LiDar::_InitLiDar(bool sendMsg)
{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
	// retrieve the devinfo
	u_result ans = mLidar_drv->getDeviceInfo(rplidarInfo);
	if (IS_FAIL(ans))
	{

	}

	modeVec_.clear();
	mLidar_drv->getAllSupportedScanModes(modeVec_);

	_u16 typicalMode;
	mLidar_drv->getTypicalScanMode(typicalMode);
	usingScanMode_ = typicalMode;

	mLidar_drv->checkMotorCtrlSupport(mIsSupportControlMoto);

	_CheckDeviceHealth();
	mLidar_drv->startMotor();

	if (sendMsg)
		PX2_GR.SendGeneralEvent("LIDARINIT1", 1.0f);
#endif
}
//----------------------------------------------------------------------------
void LiDar::_InitLiDar1()
{
#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
	mLidar_drv->startScanExpress(0, RPLIDAR_CONF_SCAN_COMMAND_STD);
	mThread->Start(_ThreadLiDarReadCallback, this);

	mIsConnected = true;

	Event *ent = PX2_CREATEEVENTEX(AIES, LiDarOpen);
	ent->SetTimeDelay(0.5f);
	ent->SetDataStr0(mPortIP);
	PX2_EW.BroadcastingLocalEvent(ent);
#endif
}
//----------------------------------------------------------------------------
void LiDar::Update(float appSeconds, float elapsedSeconds)
{
	PX2_UNUSED(elapsedSeconds);

	std::vector<RslidarDataComplete> lidarDatas;
	{
		ScopedCS cs(&mMutex);
		lidarDatas = mLiDarData;
	}
	mLiDarDataReturn = lidarDatas;

	if (mClientConnector && LT_SICK==mLiDarType)
	{
		mClientConnector->Update(appSeconds);
	}

	if (mLiDarWR && LT_WR == mLiDarType)
	{
		mLiDarWR->Update(appSeconds, elapsedSeconds);
	}

	// update cur tex
	if (mIsHasDataNew && mIsNeedUpdateTexture)
	{
		int texWidth = mTextureLiDar->GetWidth();
		int texHeight = mTextureLiDar->GetHeight();
		char* pDestLidar = mTextureLiDar->GetData(0);

		// clear
		int offsetDst = 0;
		for (int row = 0; row < texWidth; ++row)
		{
			for (int col = 0; col < texHeight; ++col)
			{
				pDestLidar[offsetDst + 0] = 0; // b
				pDestLidar[offsetDst + 1] = 0; // g 
				pDestLidar[offsetDst + 2] = 0; // r
				pDestLidar[offsetDst + 3] = 255;
				offsetDst += 4;
			}
		}

		// update
		float maxDist = 15.0f;
		for (int i = 0; i < (int)lidarDatas.size(); i++)
		{
			RslidarDataComplete lidarData = lidarDatas[i];

			int signal = lidarData.signal;
			float angle = lidarData.angle;
			float distance = lidarData.distance;

			float rad = angle * Mathf::DEG_TO_RAD;
			float offsetX = Mathf::Cos(rad) * distance / maxDist;
			float offsetY = Mathf::Sin(rad) * distance / maxDist;
			float u = 0.5f + 0.5f * offsetX;
			float v = 0.5f + 0.5f * offsetY;
			int posU = (int)(texWidth * u);
			if (posU < 0)
				continue;
			if (posU > texWidth-1) 
				continue;

			int posV = texHeight - (int)(texHeight * v);
			if (posV < 0)
				continue;
			if (posV > texHeight-1)
				continue;

			int destPos = (posV * texWidth + posU)*4;
			pDestLidar[destPos + 0] = 0; // b
			pDestLidar[destPos + 1] = 0; // g 
			pDestLidar[destPos + 2] = 255; // r
			pDestLidar[destPos + 3] = 255;
		}

		Renderer::UpdateAll(mTextureLiDar, 0);
	}

	mIsHasDataNew = false;
}
//----------------------------------------------------------------------------
void LiDar::SetLiData(const std::vector<RslidarDataComplete> &datas)
{
	mLiDarData = datas;
	mIsHasDataNew = true;
}
//----------------------------------------------------------------------------
std::vector<RslidarDataComplete> LiDar::GetLiDarData()
{
	return mLiDarDataReturn;
}
//----------------------------------------------------------------------------
Timestamp LiDar::GetLiDarDataTimestamp()
{
	return mLidarDatTimestamp;
}
//----------------------------------------------------------------------------
Texture2D *LiDar::GetTextureLidar()
{
	return mTextureLiDar;
}
//----------------------------------------------------------------------------
UIFPicBox *LiDar::GetUIFPicBoxLidar()
{
	return mPicBoxLiDar;
}
//----------------------------------------------------------------------------