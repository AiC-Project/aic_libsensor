# Copyright (C) 2009 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)


include $(CLEAR_VARS)
IGNORED_WARNINGS := -Wno-sign-compare -Wno-unused-parameter -Wno-sign-promo -Wno-error=return-type

LOCAL_PBUF_INTERMEDIATES := $(call generated-sources-dir-for,SHARED_LIBRARIES,libcppsensors_packet,,)/proto/external/aic/libaicd/
LOCAL_CFLAGS		:= -DLOG_TAG=\"protobuf_sensors\" -O2 -DGOOGLE_PROTOBUF_NO_RTTI
LOCAL_SHARED_LIBRARIES := liblog libcutils libstlport libcppsensors_packet
LOCAL_STATIC_LIBRARIES += libprotobuf-cpp-2.3.0-lite libprotobuf-cpp-2.3.0-full

LOCAL_C_INCLUDES := external/stlport/stlport \
					bionic \
					external/aic/libaicd \
					external/protobuf/src \
					$(LOCAL_PBUF_INTERMEDIATES)

LOCAL_SRC_FILES := recv_protobuf.cpp
LOCAL_MODULE := librecv_protobuf
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)

# HAL module implemenation stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.hardware>.so
include $(CLEAR_VARS)

LOCAL_CFLAGS		:= -DLOG_TAG=\"aicsensors\"
LOCAL_MODULE_PATH	:= $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils librecv_protobuf
LOCAL_C_INCLUDES := bionic \
					external/aic/libaicd
LOCAL_SRC_FILES := sensors_aic.c properties.c
LOCAL_MODULE := sensors.goby
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)

