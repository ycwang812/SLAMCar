// BBlockSystem.hpp

#ifndef BBLOCKSYSTEM_HPP
#define BBLOCKSYSTEM_HPP

#include "PX2BBlockPre.hpp"
#include "PX2Object.hpp"
#include "PX2Singleton_NeedNew.hpp"
#include "PX2Serial.hpp"
#include "PX2EventHandler.hpp"
#include "PX2BBlockSprite.hpp"
#include "PX2UI.hpp"
#include "PX2SocketAddress.hpp"
#include "PX2UDPServer.hpp"

namespace PX2
{

	class PX2_ENGINE_ITEM BBlockSystem : public EventHandler, public Singleton<BBlockSystem>
	{
	public:
		BBlockSystem();
		virtual ~BBlockSystem();

		void Initlize();
		bool IsInitlized() const;

		virtual void OnEvent(Event *event);

		void Save(const std::string &filename);
		bool Load(const std::string &filename);
		void Clear();

		void Run(bool run);
		void ReStart();
		bool IsRunning() const;
		void Update(float elapsedTime);

		enum SysType
		{
			ST_BLUETOOTH,
			ST_SERIAL,
			ST_WIFI_ROBOT,
			ST_MAX_TYPE
		};
		void SetSysType(SysType st, int targetRobotID = 1,
			const std::string &serialPort = "", int baudrate=9600);
		SysType GetSysType() const;

	private:
		static void _UDPServerRecvCallback(SocketAddress &address,
			const std::string &buf);

		bool mIsInitlized;
		SysType mSetType;
		int mTargetRobotID;
		UDPServerPtr mServerWifi;

		// general operators
	public:
		// base
		void setName(const std::string &name);
		std::string getName() const;

		void setFixUpdateFPS(int fps);
		int getFixUpdateFPS() const;

		// view
		void show(bool show);
		bool isShow() const;

		void setText(const std::string &str);
		std::string getText();

		// transform
		void setScale(float val);
		float getScale() const;
		void setScaleXZ(float scaleX, float scaleZ);
		void setScaleX(float val);
		void setScaleZ(float val);
		float getScaleX() const;
		float getScaleZ() const;
		void addScale(float scale);

		void setRotateXYZ(float x, float y, float z);
		void setRotateX(float degree);
		void setRotateY(float degree);
		void setRotateZ(float degree);
		float getRotateX() const;
		float getRotateY() const;
		float getRotateZ() const;
		void addRotate(float degree);

		void setLayout(float horA, float horB, float horParamA, float horParamB,
			float verA, float verB, float verParamA, float verParamB,
			float pivotX, float pivotY, float width, float height);
		void setPostion(float posX, float posZ);
		void setAnchorHor(float valA, float valB);
		void setAnchorParamHor(float valA, float valB);
		void setAnchorVer(float valA, float valB);
		void setAnchorParamVer(float valA, float valB);
		void move(float moveX, float moveZ);
		void setLayer(int layer);
		int getLayer() const;

		void setPivot(float pivotX, float pivotY);
		void setSize(float size);
		void setSizeWidthHeight(float width, float height);

		// round
		float getDragingDirX() const;
		float getDragingDirY() const;

		enum SimpleDirectionType
		{
			SDT_NONE,
			SDT_FORWARD,
			SDT_BACKWARD,
			SDT_LEFT,
			SDT_RIGHT,
			SDT_MAX_TYPE
		};

		SimpleDirectionType getDragingDirSimple() const;
		float getDragingPercent() const;

		enum DirectionType
		{
			DT_NONE,
			DT_FORWARD,
			DT_BACKWARD,
			DT_MAX_TYPE
		};
		DirectionType getDragingLeftMoveType() const;
		float getDragingLeftMoveSpeed() const;
		DirectionType getDragingRightMoveType() const;
		float getDragingRightMoveSpeed() const;

		// slider
		void setSliderPercent(float perc = 0.5f);
		void setSlider100(int val = 50);
		void setSlider180(int val = 90);
		float getSliderPercent() const;
		int getSlider100() const;
		int getSlider180() const;
		int getSlider360() const;

		// combo
		void addChooseString(const std::string &str);
		void chooseString(const std::string &str);
		void chooseIndex(int index);
		std::string getChooseString();
		int getChooseIndex();

		// web
		void setURL(const std::string &url);

		// arduino -------------------------
		enum Pin
		{
			P_0 = 0,
			P_1,
			P_2,
			P_3,
			P_4,
			P_5,
			P_6,
			P_7,
			P_8,
			P_9,
			P_10,
			P_11,
			P_12,
			P_13,
			P_A0,
			P_A1,
			P_A2,
			P_A3,
			P_A4,
			P_A5,
			P_MAX_TYPE
		};

		enum PMode
		{
			PM_INPUT,
			PM_OUTPUT,
			PM_MAX_TYPE
		};

		void pinMode(Pin pin, PMode mode);
		void digitalWrite(Pin pin, bool isHigh = true);
		void analogWrite(Pin pin, int val);
		int digitalRead(Pin pin);
		int analogRead(Pin pin);

		// server
		void serverInit(int index, Pin pin);
		void serverWrite(int index, int val);

		// dist
		void distInit(Pin pinTrig, Pin pinEcho);
		void distTest();
		float getDist() const;

		// vehicle
		void vehicleInitMotoBoard();
		void vehicleInitMotoBoard4567();
		void vehicleRun(int index, DirectionType type, int speed);
		void vehicleRunSimple(SimpleDirectionType type, int speed);
		void vehicleStop();
		void vehicleSpeedInit(Pin pinLA, Pin pinLB, Pin pinRA, Pin pinRB);
		int vehicleGetSpeed(int motoIndex) const;

		// mp3
		void mp3Init(Pin pinR, Pin pinT);
		void mp3Play();
		void mp3PlayNext();
		void mp3PlayAtIndex(int index);
		void mp3SetVolume(int volume);
		void mp3Stop();

		// ir
		void irSend(int val=100);
		void irRecvInit(Pin pinReceive);
		int getIRReceive() const;

	public:
		// sprite
		BBlockSprite *setCurBBlockSprite(const std::string &name);
		BBlockSprite *getCurBBlockSprite();

		// create
		BBlockSprite *createSpriteFrame();
		BBlockSprite *createSpritePicBox();
		BBlockSprite *createSpriteButton();
		BBlockSprite *createSpriteText();
		BBlockSprite *createSpriteEditBox();
		BBlockSprite *createSpriteComboBox();
		BBlockSprite *createSpriteWebView();
		BBlockSprite *createSpriteRound();

		// set type
		void setBluetooth();
		void setSerial(const std::string &port="COM3");
		void setWifiRobot(int robotID=1);

		void setDeviceID(int id=1);
		int getDeviceID() const;

		// log
		void log(const std::string &str);
		
		// string hep
		int stringToInt(const std::string &str);
		float stringToFloat(const std::string &str);
		std::string intToString(int val);
		std::string floatToString(float val);
		std::string getStringParam(const std::string &paramStr, int index);
		std::string stringAdd(const std::string &str0="a", const std::string &str1="b");

		// time
		int getYear() const;
		int getMonth() const;
		int getWeek() const;
		int getDay() const;
		int getDayOfWeek() const;
		int getTimeHour() const;
		int getTimeMinute() const;
		int getTimeSecond() const;
		std::string getTimeString() const;

		// random
		int rand255();
		int randZeroOne();
		float randZero2One();

		// music
		void playMusic(const std::string &key);
		void stopMusic();
		void playCam();

		// event
		void sendEvent(const std::string &str);

		std::string getEventObjectName() const;
		float getEventDataFloat() const;
		std::string getEventDataString() const;

		// blue tooth
		void bluetoothSendString(const std::string &str);

		// voice
		void voiceStart();
		void voiceEnd();
		void voiceSpeak(const std::string &str = "Hello World");
		void voiceAutoSpeakEnable(bool enable);
		bool voiceIsAutoSpeakEnable() const;

	public_internal:
		void _OnIRReceive(int val);

	public:
		std::string EventObjectName;
		float EventDataFloat;
		std::string EventDataString;
		bool mIsRuning;
		int mDeviceID;
	};

#define PX2BBLOCK_SYS BBlockSystem::GetSingleton()

}

#endif