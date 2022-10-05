
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libtransport-socket
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Transport socket library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DTSKT_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/tskt.c \
	src/tskt_impl.c \
	src/tskt_pomp.c \
	src/tskt_resolv.c \
	src/tskt_resolv_impl.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libtransport-packet \
	libulog
ifeq ("$(TARGET_OS)","windows")
  LOCAL_CFLAGS += -D_WIN32_WINNT=0x0600
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := tskt-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Transport socket library test program
LOCAL_SRC_FILES := \
	tests/tskt_test.c
LOCAL_LIBRARIES := \
	libpomp \
	libtransport-packet \
	libtransport-socket \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := tskt-resolv-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := DNS resolver library test program
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := \
	tests/tskt_resolv_test.c
LOCAL_LIBRARIES := \
	libpomp \
	libtransport-socket \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := tskt-tcp-server
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Transport socket library TCP server test program
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := \
	tests/tskt_tcp_server.c
LOCAL_LIBRARIES := \
	libpomp \
	libtransport-packet \
	libtransport-socket \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := tskt-tcp-client
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Transport socket library TCP client test program
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := \
	tests/tskt_tcp_client.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libtransport-packet \
	libtransport-socket \
	libulog

ifeq ("$(TARGET_OS)","windows")
LOCAL_LDLIBS += -lcrypto
else
LOCAL_LIBRARIES += libcrypto
endif

include $(BUILD_EXECUTABLE)
