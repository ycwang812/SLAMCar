-- stop.lua

function engine_project_prestop()

end

function engine_project_stop()
	PX2_LOGGER:LogInfo("ZERONE", "project_stop 0")

    local udpServer = PX2_PROJ:PoolGet("DUPServer")
    if nil~=udpServer then
        udpServer:Stop()
    end
    ZERONE_UDPServer = nil

    PX2_LOGGER:LogInfo("ZERONE", "project_stop 1")

    PX2_PROJ:PoolSet("Arduino", nil)
    rc_Arduino = nil

    PX2_LOGGER:LogInfo("ZERONE", "project_stop 2")

    if rc_Robot then
        if nil==RobotExt then
            Robot:Delete(rc_Robot)
        else       
            RobotExt:Delete(rc_Robot)
        end
    end
    rc_Robot = nil

    PX2_LOGGER:LogInfo("ZERONE", "project_stop 3")

    PX2_VOICESDK:EnableAutoSpeak(false)
    PX2_VOICESDK:EnableAutoSpeakTTS(false)

    PX2_LOGGER:LogInfo("ZERONE", "project_stop 4")
	
    Robot:TernimateSlam2D()
    
    PX2_LOGGER:LogInfo("ZERONE", "project_stop 5")
end