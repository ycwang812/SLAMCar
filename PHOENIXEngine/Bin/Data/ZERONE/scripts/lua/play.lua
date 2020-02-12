-- play.lua

require("Data/ZERONE/scripts/lua/language.lua")
require("Data/ZERONE/scripts/lua/ZERONE.lua")
require("Data/ZERONE/scripts/lua/ZERONECommon.lua")
require("Data/ZERONE/scripts/lua/ZERONEConnectBleSerial.lua")
require("Data/ZERONE/scripts/lua/ZERONEConnectLidar.lua")
require("Data/ZERONE/scripts/lua/ZERONEConnectDevice.lua")
require("Data/ZERONE/scripts/lua/ZERONESetting.lua")
require("Data/ZERONE/scripts/lua/ZERONEScene.lua")
require("Data/ZERONE/scripts/lua/ZERONERobotCtrl.lua")
require("Data/ZERONE/scripts/lua/ZERONEActorAgentCtrl.lua")
require("Data/ZERONE/scripts/lua/ZERONEServer.lua")
require("Data/ZERONE/scripts/lua/ZERONEUIFaceCtrl.lua")
require("Data/ZERONE/scripts/lua/ZERONEUIPadCtrl.lua")

rc_Arduino = nil
rc_Robot = nil

function engine_project_preplay()
	collectgarbage("collect")

	PX2_APP:SetShowInfo(true)
	zo_AddLanguage()	

	PX2_BLUETOOTH:SetDataProtocolHex(false)

	PX2_ENGINECANVAS:SetClearColor(Float4:MakeColor(63, 72, 204, 255))
    PX2_ENGINESCENECANVAS:SetClearColor(Float4:MakeColor(63, 72, 204, 255))
end

connectLidarPort = ""
connectRobotPort = ""
function engine_project_play()
	Robot:InitlizeSlam2D()
	
	zo_ZERONE()

	local serial = Serial()
	serial:UpdatePortList()
	local numPorts = serial:GetNumPorts()
	for i=0, numPorts-1 do
        local portStr = serial:GetPort(i)
        local portDesc = serial:GetPortDesc(i)
		local hardID = serial:GetPortHardID(i)

		local platType = PX2_APP:GetPlatformType()
		if Application.PLT_LINUX==platType then
			if string.find(portDesc, "UART") then
				connectLidarPort = portStr
			elseif string.find(portDesc, "USB2.0") then
				connectRobotPort = portStr
			end
		elseif Application.PLT_WINDOWS==platType then
			if string.find(portDesc, "UART") then
				connectLidarPort = portStr
			elseif string.find(portDesc, "SERIAL") then
				connectRobotPort = portStr
			end
		end
	end  

	PX2_LOGGER:LogInfo("ZERONE", "lidar autoConnect:"..connectLidarPort)
	PX2_LOGGER:LogInfo("ZERONE", "rc_Arduino autoConnect:"..connectRobotPort)

	-- on raspberry we use linux
	local platType = PX2_APP:GetPlatformType()
	if Application.PLT_LINUX==platType and PX2_APP:IsARM() then
		PX2_GR:SendGeneralEvent("autoconnectlidar", "", 7.0)
	end
end

function engine_project_update(appseconds, elapsedseconds)
	zo_AppUpdateCallback(appseconds, elapsedseconds)
end

function startForLidarSender()
	PX2_LOGGER:LogInfo("ZERONE", "startForLidarSender")

	rc_Robot:GetLidar():SetLiDarType(LiDar.LT_III)
	--rc_Robot:SetRoleType(Robot.RT_MASTER_ONLY_SENDLIDAR)
	rc_Robot:SetRoleType(Robot.RT_MASTER)

	rc_Robot:LidarOpen(connectLidarPort, 230400)
	--rc_Robot:LidarOpen(connectLidarPort, 115200)
	
	rc_Arduino:Initlize(Arduino.M_SERIAL, connectRobotPort, 9600)
end

UnRegistAllEventFunctions("GraphicsES::GeneralString")
RegistEventFunction("GraphicsES::GeneralString", function(str)
	if "autoconnectlidar"==str then
		startForLidarSender()
	end
end)

-- cmds default called by PHOENIXEngine
function engine_project_cmd(cmd, param0, param1, param2)
	if "robot" == cmd then
		connectRobotPort = param0
		rc_Arduino:Initlize(Arduino.M_SERIAL, param0)
		PX2_LOGGER:LogInfo("ZERONE", "robot arduino initlized")
	elseif "lidar"==cmd then
		connectLidarPort = param0
		local lidarType = rc_Robot:GetLidar():GetLiDarType()
		if LiDar.LT_III ==lidarType then
			rc_Robot:LidarOpen(param0, 230400)
		else
			rc_Robot:LidarOpen(param0, 115200)
		end
        PX2_LOGGER:LogInfo("ZERONE", "lidar opened")
	elseif "raspberry"==cmd then
		rc_Robot:GetLidar():SetLiDarType(LiDar.LT_III)
		rc_Robot:SetRoleType(Robot.RT_MASTER_ONLY_SENDLIDAR)
		PX2_LOGGER:LogInfo("ZERONE", "raspberry")
	elseif "1"==cmd then
		-- left
		if nil~=ZERONE_UIFaceCtrlScript then
			ZERONE_UIFaceCtrlScript:Move(MoveType.MT_LEFT, 50)
		end
	elseif "2"==cmd then
		-- right
		if nil~=ZERONE_UIFaceCtrlScript then
			ZERONE_UIFaceCtrlScript:Move(MoveType.MT_RIGHT, 50)
		end
	elseif "3"==cmd then
		-- up
		if nil~=ZERONE_UIFaceCtrlScript then
			ZERONE_UIFaceCtrlScript:Move(MoveType.MT_FORWARD, 50)
		end
	elseif "4"==cmd then
		-- down
		if nil~=ZERONE_UIFaceCtrlScript then
			ZERONE_UIFaceCtrlScript:Move(MoveType.MT_BACKWARD, 50)
		end
	elseif "0"==cmd then
		if nil~=ZERONE_UIFaceCtrlScript then
			ZERONE_UIFaceCtrlScript:Move(MoveType.MT_NONE, 0)
		end
	end
end
