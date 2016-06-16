LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := WakeUp
LOCAL_SRC_FILES := wakeup.cpp
LOCAL_LDLIBS += -L$(SYSTEM)/usr/lib -llog

include $(BUILD_SHARED_LIBRARY)
