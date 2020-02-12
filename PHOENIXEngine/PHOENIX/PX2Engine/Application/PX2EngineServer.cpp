// PX2EngineServer.hpp

#include "PX2EngineServer.hpp"
#include "PX2EngineCanvas.hpp"
#include "PX2StringHelp.hpp"
#include "PX2StringTokenizer.hpp"
#include "PX2Application.hpp"
#include "PX2Dir.hpp"
#include "PX2EngineNetCmdProcess.hpp"
#include "PX2EngineNetEvent.hpp"
#include "PX2EngineNetDefine.hpp"
#include "PX2Log.hpp"
#include "PX2Robot.hpp"
#include "PX2RobotDatas.hpp"
#include "PX2OutStream.hpp"
#include "PX2AILiDar.hpp"
#include "PX2RobotDatas.hpp"
using namespace PX2;

//----------------------------------------------------------------------------
_ConnectObj::_ConnectObj()
{
	ClientID = 0;
	HeartTiming = 0.0f;
}
//----------------------------------------------------------------------------
_ConnectObj::~_ConnectObj()
{
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
EngineServer::EngineServer(Server::ServerType serverType, int port) :
	Server(serverType, port, 10, 20),
	mBroadCastMapTiming(0.0f),
	mBroadCastSelfDrawTiming(0.0f),
	mBroadCastLiDarTiming(0.0f),
	mBroadCastRobotStateTiming(0.0f)
{
	RegisterHandler(EngineServerMsgID, (Server::MsgHandleFunc)
		&EngineServer::OnString);
	RegisterHandler(EngineServerArduinoMsgID, (Server::MsgHandleFunc)
		&EngineServer::OnArduinoString);
	RegisterHandler(EngineClientSendRobotToDoID, (Server::MsgHandleFunc)
		&EngineServer::OnRobotToDo);
}
//----------------------------------------------------------------------------
EngineServer::~EngineServer()
{

}
//----------------------------------------------------------------------------
void EngineServer::Run(float elapsedTime)
{
	Server::Run(elapsedTime);

	auto it = mConnections.begin();
	for (; it != mConnections.end();)
	{
		_ConnectObj *obj = it->second;

		obj->HeartTiming += elapsedTime;

		if (obj->HeartTiming > EngineServerHeartTime)
		{
			obj->HeartTiming = 0.0f;

			int clientID = (int)it->first;

			DisconnectClient(it->first);
			it = mConnections.erase(it);

			PX2_LOG_INFO("Heart Disconnect");
		}
		else
		{
			it++;
		}
	}

	Robot *robot = Robot::GetSingletonPtr();
	if (robot)
	{
		mBroadCastMapTiming += elapsedTime;
		if (mBroadCastMapTiming > 1.0f)
		{
			Robot::RoleType rt = robot->GetRoleType();
			if (Robot::RT_MASTER == rt)
			{
				BroadCastRobotMap();
				BroadCastSelfDraw();
			}

			mBroadCastMapTiming = 0;
		}

		//mBroadCastSelfDrawTiming += elapsedTime;
		//if (mBroadCastSelfDrawTiming > 2.0f)
		//{
		//	Robot::RoleType rt = robot->GetRoleType();
		//	if (Robot::RT_MASTER == rt)
		//		BroadCastRobotSelfDrawMap();

		//	mBroadCastSelfDrawTiming = 0.0;
		//}

		mBroadCastLiDarTiming += elapsedTime;
		if (mBroadCastLiDarTiming > 0.5f)
		{
			Robot::RoleType rt = robot->GetRoleType();
			if (Robot::RT_MASTER == rt || Robot::RT_MASTER_ONLY_SENDLIDAR == rt)
			{
				BroadCastLidarData();
			}

			mBroadCastLiDarTiming = 0.0f;
		}

		mBroadCastRobotStateTiming += elapsedTime;
		if (mBroadCastRobotStateTiming > 0.1f)
		{
			Robot::RoleType rt = robot->GetRoleType();
			if (Robot::RT_MASTER == rt)
			{
				BroadCastRobotState();
			}

			mBroadCastRobotStateTiming = 0.0f;
		}
	}
}
//----------------------------------------------------------------------------
void EngineServer::SendString(int clientid, const std::string &str)
{
	if (!str.empty() && clientid > 0)
	{
		SendMsgToClientBuffer(clientid, EngineServerMsgID, str.c_str(),
			str.length());
	}
}
//----------------------------------------------------------------------------
void EngineServer::BroadCastString(const std::string &str)
{
	auto it = mConnections.begin();
	for (; it != mConnections.end(); it++)
	{
		unsigned int clientID = it->first;
		SendString(clientID, str);
	}
}
//----------------------------------------------------------------------------
void EngineServer::SendRobotState(int clientid)
{
	RobotMapData *curMapData = PX2_ROBOT.GetCurMapData();
	RobotMapDataStruct stru = curMapData->MapStruct;
	SendMsgToClientBuffer(clientid, EngineServerSendRobotState,
		(const char *)&stru, sizeof(stru));
}
//----------------------------------------------------------------------------
void EngineServer::BroadCastRobotState()
{
	auto it = mConnections.begin();
	for (; it != mConnections.end(); it++)
	{
		unsigned int clientID = it->first;
		SendRobotState(clientID);
	}
}
//----------------------------------------------------------------------------
void EngineServer::SendRobotMap(int clientid)
{
	RobotMapData *curMapData = PX2_ROBOT.GetCurMapData();
	std::vector<unsigned char> mapDatas = curMapData->Map2DOrigin;
	if (mapDatas.size() == 0)
		return;

	RobotMapDataStruct stru = curMapData->MapStruct;
	SendMsgToClientBuffer(clientid, EngineServerSendMapInfoMsgID,
		(const char *)&stru, sizeof(stru));

	// map
	std::vector<unsigned char> dstDatas;
	dstDatas.resize(mapDatas.size());
	memset(&dstDatas[0], 0, (int)dstDatas.size());

	unsigned long destLength = dstDatas.size();
	PX2_RM.ZipCompress(&(dstDatas[0]), &destLength, &(mapDatas[0]),
		mapDatas.size());

	int sendSize = 1024;
	int sendedSize = 0;
	int toSendSize = 0;
	int leftSize = destLength - sendedSize;
	while (leftSize > 0)
	{
		int thisTimeSendSize =  leftSize > sendSize ? sendSize : leftSize;
		SendMsgToClientBuffer(clientid, EngineServerSendMapMsgID,
			(const char *)&dstDatas[0] + sendedSize, thisTimeSendSize);
		
		sendedSize += thisTimeSendSize;
		leftSize = destLength - sendedSize;
	}

	std::string strEnd = "mapend";
	SendMsgToClientBuffer(clientid, EngineServerSendMapEndMsgID,
		strEnd.c_str(), strEnd.length());
}
//----------------------------------------------------------------------------
void EngineServer::BroadCastRobotMap()
{
	auto it = mConnections.begin();
	for (; it != mConnections.end(); it++)
	{
		unsigned int clientID = it->first;
		SendRobotMap(clientID);
	}
}
//----------------------------------------------------------------------------
void EngineServer::SendRobotSelfDraw(int clientid)
{
	RobotMapData *curMapData = PX2_ROBOT.GetCurMapData();
	std::vector<unsigned char> mapDatas = curMapData->SelfDrawMapData2D;
	if (mapDatas.size() == 0)
		return;

	// map
	std::vector<unsigned char> dstDatas;
	dstDatas.resize(mapDatas.size());
	memset(&dstDatas[0], 0, (int)dstDatas.size());

	unsigned long destLength = dstDatas.size();
	PX2_RM.ZipCompress(&(dstDatas[0]), &destLength, &(mapDatas[0]),
		mapDatas.size());

	int sendSize = 1024;
	int sendedSize = 0;
	int toSendSize = 0;
	int leftSize = destLength - sendedSize;
	while (leftSize > 0)
	{
		int thisTimeSendSize = leftSize > sendSize ? sendSize : leftSize;
		SendMsgToClientBuffer(clientid, EngineServerSendMapSelfDrawMsgID,
			(const char *)&dstDatas[0] + sendedSize, thisTimeSendSize);

		sendedSize += thisTimeSendSize;
		leftSize = destLength - sendedSize;
	}

	std::string strEnd = "mapend";
	SendMsgToClientBuffer(clientid, EngineServerSendMapSelfDrawEndMsgID,
		strEnd.c_str(), strEnd.length());
}
//----------------------------------------------------------------------------
void EngineServer::BroadCastSelfDraw()
{
	auto it = mConnections.begin();
	for (; it != mConnections.end(); it++)
	{
		unsigned int clientID = it->first;
		SendRobotSelfDraw(clientID);
	}
}
//----------------------------------------------------------------------------
void EngineServer::SendLidarData(int clientid)
{
	LiDar *liDar = PX2_ROBOT.GetLidar();
	if (!liDar)
		return;

	std::vector<RslidarDataComplete> lData = liDar->GetLiDarData();
	int size = lData.size();
	if (0 == size)
		return;

	OutStream outStream;

	NetLidarData data;
	data.Datas = lData;

	int bufSize = 0;
	char *buf = 0;

	outStream.Insert(&data);
	outStream.Save(bufSize, buf);

	std::vector<unsigned char> dstDatas;
	dstDatas.resize(bufSize);
	memset(&dstDatas[0], 0, bufSize);

	unsigned long destLength = bufSize;
	PX2_RM.ZipCompress(&(dstDatas[0]), &destLength, (const unsigned char*)buf, bufSize);

	if (destLength > 0)
	{
		//PX2_LOG_INFO("try to send lidardata: %d", destLength);
		SendMsgToClientBuffer(clientid, EngineServerSendLidarMsgID,
			(const char *)&(dstDatas[0]), destLength);
		//PX2_LOG_INFO("end send lidardata");
	}

	delete1(buf);
}
//----------------------------------------------------------------------------
void EngineServer::BroadCastLidarData()
{
	auto it = mConnections.begin();
	for (; it != mConnections.end(); it++)
	{
		unsigned int clientID = it->first;
		SendLidarData(clientID);
	}
}
//----------------------------------------------------------------------------
int EngineServer::OnConnect(unsigned int clientid)
{
	mBroadCastMapTiming = 0.0f;
	mBroadCastLiDarTiming = 0.0f;

	ClientContext *clentContext = GetClientContext(clientid);
	std::string ip = clentContext->TheSocket.GetAddress().GetHost()
		.ToString();

	_ConnectObj *cntObj = new0 _ConnectObj();
	cntObj->ClientID = clientid;
	cntObj->IP = ip;
	mConnections[clientid] = cntObj;

	Event *ent = PX2_CREATEEVENTEX(EngineNetES, OnEngineServerBeConnected);
	ent->SetDataStr0(StringHelp::IntToString((int)clientid));
	ent->SetDataStr1(ip);
	PX2_EW.BroadcastingLocalEvent(ent);

	return 0;
}
//----------------------------------------------------------------------------
int EngineServer::OnDisconnect(unsigned int clientid)
{
	PX2_LOG_INFO("EngineServer Begin Disconnect %d", clientid);

	_ConnectObj *obj = mConnections[clientid];
	std::string ip = obj->IP;
	mConnections.erase(clientid);

	Event *ent = PX2_CREATEEVENTEX(EngineNetES, OnEngineServerBeDisConnected);
	ent->SetDataStr0(StringHelp::IntToString((int)clientid));
	ent->SetDataStr1(ip);
	PX2_EW.BroadcastingLocalEvent(ent);

	PX2_LOG_INFO("EngineServer End Disconnect");

	return 0;
}
//----------------------------------------------------------------------------
int EngineServer::OnString(unsigned int clientid, const void *pbuffer,
	int buflen)
{
	std::string strBuf((const char*)pbuffer, buflen);

	EngineCanvas *engineCanvas = EngineCanvas::GetSingletonPtr();
	if (engineCanvas)
	{
		UIList *list = engineCanvas->GetEngineInfoList();
		list->AddItem("OnString clientid:" + StringHelp::IntToString(clientid) +
			" " + "Str:" + strBuf);
	}

	StringTokenizer stk(strBuf, " ");
	int numTok = stk.Count();
	
	std::string cmdStr;
	std::string paramStr0;
	std::string paramStr1;
	std::string paramStr2;
	if (numTok >= 1)
		cmdStr = stk[0];
	if (numTok >= 2)
		paramStr0 = stk[1];
	if (numTok >= 3)
		paramStr1 = stk[2];
	if (numTok >= 4)
		paramStr2 = stk[3];

	if ("heart" == cmdStr)
	{
		mConnections[clientid]->HeartTiming = 0.0f;
	}

	EngineNetCmdProcess::OnCmd(mConnections[clientid]->IP, 
	cmdStr, paramStr0, paramStr1, paramStr2);

	return 0;
}
//----------------------------------------------------------------------------
int EngineServer::OnArduinoString(unsigned int clientid, 
	const void *pbuffer, int buflen)
{
	std::string strBuf((const char*)pbuffer, buflen);

	Robot *robot = Robot::GetSingletonPtr();
	if (robot)
	{
		Arduino *arduino = robot->GetArduino();
		if (arduino)
		{
			arduino->_Send(strBuf);
		}
	}

	return 0;
}
//----------------------------------------------------------------------------
int EngineServer::OnRobotToDo(unsigned int clientid, const void *pbuffer, 
	int buflen)
{
	RobotToDoStruct robotToDo;
	memcpy(&robotToDo, pbuffer, buflen);

	Robot *robot = Robot::GetSingletonPtr();
	if (robot)
	{
		int tp = robotToDo.Type;
		if (1 == tp)
		{
			robot->SetSpeedLeft(robotToDo.LeftSpeed);
			robot->SetSpeedRight(robotToDo.RightSpeed);
		}
		else if (2 == tp)
		{
			robot->GoTarget(robotToDo.TargetPos);
		}
		else if (3 == tp)
		{
			robot->ClearPathFinder();
		}
		else if (4 == tp)
		{
			robot->SaveMap("map.px2obj");
		}
		else if (5 == tp)
		{
			robot->LoadMap("map.px2obj");
		}
		else if (6 == tp)
		{
			robot->SetObstMapValueAtPos(robotToDo.TargetPos, robotToDo.Range,
				robotToDo.Value);
		}
	}

	return 0;
}
//----------------------------------------------------------------------------