LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := fmod

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libfmodL.so

LOCAL_C_INCLUDES := \
$(LOCAL_PATH)/android/inc

LOCAL_EXPORT_C_INCLUDES := \
$(PX2_ROOT)/ThirdPartiesLibs/FMOD/android/inc

include $(PREBUILT_SHARED_LIBRARY)