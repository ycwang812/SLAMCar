// PX2AILiDar.hpp

#ifndef PX2AILIDAR_HPP
#define PX2AILIDAR_HPP

#include "PX2CorePre.hpp"
#include "PX2Object.hpp"
#include "PX2UIFPicBox.hpp"
#include "PX2Memory.hpp"
#include "PX2Thread.hpp"
#include "PX2Timestamp.hpp"
#include "PX2AILidarData.hpp"

namespace everest
{
	namespace hwdrivers
	{
		class C3iroboticsLidar2;
		class C3iroboticsLidar;
		class CDeviceConnection;
	}
}

#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)

namespace rp{
	namespace standalone
	{
		namespace rplidar
		{
			class RPlidarDriver;
		}
	}

	namespace hal
	{
		class Locker;
	}
}

#endif

class SinglePositionSLAM;

namespace PX2
{

	enum SopasProtocol
	{
		CoLa_A, ///< Command Language ASCI
		CoLa_B,  ///< Command Language binary
		CoLa_Unknown  ///< Unknown Command Language
	};

	class ClientConnector;
	class WRLidar;

	class PX2_ENGINE_ITEM LiDar : public Object
	{
	public:
		LiDar();
		virtual ~LiDar();

		static LiDar *New();

		enum LiDarType
		{
			LT_III,
			LT_III3,
			LT_RP,
			LT_WR,
			LT_SICK,
			LT_MAX_TYPE
		};
		void SetLiDarType(LiDarType type);
		LiDarType GetLiDarType() const;

		bool Open(const std::string &portIP, int baudratePort);

		void SetNeedUpdateTexture(bool up);
		bool IsNeedUpdateTexture() const;

		bool IsOpened() const;
		void Close();

		Mutex &GetMutexOpenClose();

		void SetOffsetDegree(float offsetDegree);

		void SetCurSlam3DTransformLost(bool lost);
		void SetCurTransform(HMatrix &mat);
		const APoint &GetCurPos() const;
		const AVector &GetCurDirection() const;

		void Update(float appSeconds, float elapsedSeconds);

		void SetLiData(const std::vector<RslidarDataComplete> &datas);
		std::vector<RslidarDataComplete> GetLiDarData();
		Timestamp GetLiDarDataTimestamp();

		Texture2D *GetTextureLidar();
		UIFPicBox *GetUIFPicBoxLidar();

		void SetPosColor(int posWidth, int posHeight, int r, int g, int b);

		void GetSlamData();

		virtual void OnEvent(Event *ent);

	protected:
		bool _CheckDeviceHealth(int * errorCode = 0);
		void _InitLiDar(bool sendMsg);
		void _InitLiDar1();

		std::string mPortIP;
		std::string mBaudratePort;

		LiDarType mLiDarType;
		float mOffsetDegree;
		everest::hwdrivers::C3iroboticsLidar2 *mLiDarIII;
		everest::hwdrivers::C3iroboticsLidar *mLiDarIII3;
		everest::hwdrivers::CDeviceConnection *mDeviceConnectin;
		ThreadPtr mThread;
		Mutex mMutex;
		Mutex mMutexOpenClose;
		bool mIsNeedUpdateTexture;

#if defined (_WIN32) || defined(WIN32) || defined(__LINUX__)
		rp::standalone::rplidar::RPlidarDriver * mLidar_drv;
#endif

		WRLidar *mLiDarWR;

		ClientConnector *mClientConnector;

		bool mIsConnected;
		bool mIsSupportControlMoto;

		Texture2DPtr mTextureLiDar;
		UIFPicBoxPtr mPicBoxLiDar;
		std::vector<RslidarDataComplete> mLiDarData;
		std::vector<RslidarDataComplete> mLiDarDataReturn;
		Timestamp mLidarDatTimestamp;
		bool mIsHasDataNew;

		bool mIsCurTransformLost;
		HMatrix mMatrix;
		HMatrix mMatrixRot;
		APoint mCurPos;
		AVector mCurDirection;
		bool mIsTransformUpdate;
	};
	typedef Pointer0<LiDar> LiDarPtr;

}

#endif