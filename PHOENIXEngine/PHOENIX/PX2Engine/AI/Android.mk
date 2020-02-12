LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := AI

FILE_LIST := $(wildcard $(LOCAL_PATH)/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/iiiroboticslidar2/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/iiiroboticslidar3/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/detour/src/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/recast/src/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/SkillSystem/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/SlamCommon/*cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/Slam2D/*cpp)
LOCAL_SRC_FILES := $(FILE_LIST:$(LOCAL_PATH)/%=%) 

#FILE_LIST += $(wildcard $(LOCAL_PATH)/Slam2D/*cpp)
#FILE_LIST += $(wildcard $(LOCAL_PATH)/SlamGMapping/*cpp)

LOCAL_C_INCLUDES := $(LOCAL_PATH)/.. \
$(LOCAL_PATH)/../Physics/ \
$(LOCAL_PATH)/../Physics/Bullet/ \
$(LOCAL_PATH)/../Physics/Bullet/LinearMath/ \
$(LOCAL_PATH)/../Physics/Bullet/BulletCollision/ \
$(LOCAL_PATH)/../Physics/Bullet/BulletDynamics/ \
$(LOCAL_PATH)/../Unity/Lua/ \
$(LOCAL_PATH)/../Unity/LuaPlus/ \
$(LOCAL_PATH)/../Unity/ToLua/ \
$(LOCAL_PATH)/iiiroboticslidar2/ \
$(LOCAL_PATH)/iiiroboticslidar3/ \
$(LOCAL_PATH)/recast/include/ \
$(LOCAL_PATH)/detour/include/ \
$(LOCAL_PATH)/SkillSystem/ \
$(LOCAL_PATH)/SlamCommon/ \
$(LOCAL_PATH)/Slam2D/ \
$(PX2_ROOT)/PX2SDK/Include/ \

# $(LOCAL_PATH)/SlamGMapping/
# $(LOCAL_PATH)/Slam2D/

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/..

LOCAL_CFLAGS := -D__ANDROID__ 
LOCAL_CPPFLAGS := -frtti -std=c++11 -fexceptions

include $(BUILD_STATIC_LIBRARY)
