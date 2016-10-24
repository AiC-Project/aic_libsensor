/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* this implements a sensors hardware library for the Android emulator.
 * the following code should be built as a shared library that will be
 * placed into /system/lib/hw/sensors.goldfish.so
 *
 * it will be loaded by the code in hardware/libhardware/hardware.c
 * which is itself called from com_android_server_SensorService.cpp
 */


/* we connect with the emulator through the "sensors" qemud service
 */
#define  SENSORS_SERVICE_NAME "sensors"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cutils/log.h>
#include <cutils/native_handle.h>
#include <cutils/sockets.h>
#include <hardware/sensors.h>

#include <sys/select.h>
#include <sys/types.h>

#define AIC
#if 0
#define  D(...)  ALOGD(__VA_ARGS__)
#else
#define  D(...)  ((void)0)
#endif

#define  E(...)  ALOGE(__VA_ARGS__)

#include <hardware/qemud.h>
#include "properties.h"

/** SENSOR IDS AND NAMES
 **/
#define MAX_RANGE (2*GRAVITY_EARTH)

#define MAX_NUM_SENSORS 14

#define SUPPORTED_SENSORS  ((1<<MAX_NUM_SENSORS)-1)

#define  ID_BASE                SENSORS_HANDLE_BASE
#define  ID_ACCELERATION        (ID_BASE+SENSOR_TYPE_ACCELEROMETER)
#define  ID_MAGNETIC_FIELD      (ID_BASE+SENSOR_TYPE_MAGNETIC_FIELD)
#define  ID_ORIENTATION         (ID_BASE+SENSOR_TYPE_ORIENTATION)
#define  ID_GYROSCOPE           (ID_BASE+SENSOR_TYPE_GYROSCOPE)
#define  ID_LIGHT               (ID_BASE+SENSOR_TYPE_LIGHT)
#define  ID_PRESSURE            (ID_BASE+SENSOR_TYPE_PRESSURE)
#define  ID_TEMPERATURE         (ID_BASE+SENSOR_TYPE_TEMPERATURE)
#define  ID_PROXIMITY           (ID_BASE+SENSOR_TYPE_PROXIMITY)
#define  ID_GRAVITY             (ID_BASE+SENSOR_TYPE_GRAVITY)
#define  ID_LINEAR_ACCELERATION (ID_BASE+SENSOR_TYPE_LINEAR_ACCELERATION)
#define  ID_ROTATION            (ID_BASE+SENSOR_TYPE_ROTATION_VECTOR)
#define  ID_HUMIDITY            (ID_BASE+SENSOR_TYPE_RELATIVE_HUMIDITY)
#define  ID_AMBIENT_TEMPERATURE         (ID_BASE+SENSOR_TYPE_AMBIENT_TEMPERATURE)

#define  SENSORS_ACCELERATION        (1 << ID_ACCELERATION)
#define  SENSORS_MAGNETIC_FIELD      (1 << ID_MAGNETIC_FIELD)
#define  SENSORS_ORIENTATION         (1 << ID_ORIENTATION)
#define  SENSORS_GYROSCOPE           (1 << ID_GYROSCOPE)
#define  SENSORS_LIGHT               (1 << ID_LIGHT)
#define  SENSORS_PRESSURE            (1 << ID_PRESSURE)
#define  SENSORS_TEMPERATURE         (1 << ID_TEMPERATURE)
#define  SENSORS_PROXIMITY           (1 << ID_PROXIMITY)
#define  SENSORS_GRAVITY             (1 << ID_GRAVITY)
#define  SENSORS_LINEAR_ACCELERATION (1 << ID_LINEAR_ACCELERATION)
#define  SENSORS_ROTATION            (1 << ID_ROTATION)
#define  SENSORS_HUMIDITY            (1 << ID_HUMIDITY)
#define  SENSORS_AMBIENT_TEMPERATURE (1 << ID_AMBIENT_TEMPERATURE)

#define  ID_CHECK(x)  ((unsigned)((x)-ID_BASE) < MAX_NUM_SENSORS)

#define  SENSORS_LIST  \
    SENSOR_(ACCELERATION,"acceleration") \
    SENSOR_(MAGNETIC_FIELD,"magnetic-field") \
    SENSOR_(ORIENTATION,"orientation") \
    SENSOR_(GYROSCOPE,"gyroscope") \
    SENSOR_(LIGHT,"light") \
    SENSOR_(PRESSURE,"pressure") \
    SENSOR_(TEMPERATURE,"temperature") \
    SENSOR_(PROXIMITY,"proximity") \
    SENSOR_(GRAVITY,"gravity") \
    SENSOR_(LINEAR_ACCELERATION,"linear_acceleration") \
    SENSOR_(ROTATION, "rotation") \
    SENSOR_(HUMIDITY,"relative_humidity") \
    SENSOR_(AMBIENT_TEMPERATURE,"ambient_temperature") \


static const struct {
    const char*  name;
    int          id; } _sensorIds[MAX_NUM_SENSORS] =
{
#define SENSOR_(x,y)  { y, ID_##x },
    SENSORS_LIST
#undef  SENSOR_
};

static const char*
_sensorIdToName( int  id )
{
    int  nn;
    for (nn = 0; nn < MAX_NUM_SENSORS; nn++)
        if (id == _sensorIds[nn].id)
            return _sensorIds[nn].name;
    return "<UNKNOWN>";
}

static int
_sensorIdFromName( const char*  name )
{
    int  nn;

    if (name == NULL)
        return -1;

    for (nn = 0; nn < MAX_NUM_SENSORS; nn++)
        if (!strcmp(name, _sensorIds[nn].name))
            return _sensorIds[nn].id;

    return -1;
}

static int s_enabled = 0x0;

/** SENSORS POLL DEVICE
 **
 ** This one is used to read sensor data from the hardware.
 ** We implement this by simply reading the data from the
 ** emulator through the QEMUD channel.
 **/

typedef struct SensorPoll {
    struct sensors_poll_device_t  device;
    sensors_event_t               sensors[MAX_NUM_SENSORS];
    int                           events_fd;
    uint32_t                      pendingSensors;
    int64_t                       timeStart;
    int64_t                       timeOffset;
    int                           fd;
    uint32_t                      active_sensors;
} SensorPoll;

/* this must return a file descriptor that will be used to read
 * the sensors data (it is passed to data__data_open() below
 */
static native_handle_t*
control__open_data_source(struct sensors_poll_device_t *dev)
{
    SensorPoll*  ctl = (void*)dev;
    native_handle_t* handle;

    if (ctl->fd < 0) {
        ctl->fd = qemud_channel_open(SENSORS_SERVICE_NAME);
    }
    D("%s: fd=%d", __FUNCTION__, ctl->fd);
    handle = native_handle_create(1, 0);
    handle->data[0] = dup(ctl->fd);
    return handle;
}

static int
control__activate(struct sensors_poll_device_t *dev,
                  int handle,
                  int enabled)
{
    SensorPoll*     ctl = (void*)dev;
    uint32_t        mask, sensors, active, new_sensors, changed;
    char            command[128];
    int             ret;

    D("%s: handle=%s (%d) fd=%d enabled=%d", __FUNCTION__,
        _sensorIdToName(handle), handle, ctl->fd, enabled);

    if (!ID_CHECK(handle)) {
        E("%s: bad handle ID", __FUNCTION__);
        return -1;
    }

    mask    = (1<<handle);
    sensors = enabled ? mask : 0;

    active      = ctl->active_sensors;
    new_sensors = (active & ~mask) | (sensors & mask);
    changed     = active ^ new_sensors;

    if (!changed)
        return 0;

    snprintf(command, sizeof command, "set:%s:%d",
                _sensorIdToName(handle), enabled != 0);

    if (ctl->fd < 0) {
        ctl->fd = qemud_channel_open(SENSORS_SERVICE_NAME);
    }

    ret = qemud_channel_send(ctl->fd, command, -1);
    if (ret < 0) {
        E("%s: when sending command errno=%d: %s", __FUNCTION__, errno, strerror(errno));
        return -1;
    }
    ctl->active_sensors = new_sensors;

    return 0;
}

static int
control__set_delay(struct sensors_poll_device_t *dev, int32_t ms)
{
    SensorPoll*     ctl = (void*)dev;
    char            command[128];

    D("%s: dev=%p delay-ms=%d", __FUNCTION__, dev, ms);

    snprintf(command, sizeof command, "set-delay:%d", ms);

    return qemud_channel_send(ctl->fd, command, -1);
}

static int
control__close(struct hw_device_t *dev)
{
    SensorPoll*  ctl = (void*)dev;
    close(ctl->fd);
    free(ctl);
    return 0;
}

/* return the current time in nanoseconds */
static int64_t
data__now_ns(void)
{
    struct timespec  ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

static int
data__data_open(struct sensors_poll_device_t *dev, native_handle_t* handle)
{
    SensorPoll*  data = (void*)dev;
    int i;
    D("%s: dev=%p fd=%d", __FUNCTION__, dev, handle->data[0]);
    memset(&data->sensors, 0, sizeof(data->sensors));

    data->pendingSensors = 0;
    data->timeStart      = 0;
    data->timeOffset     = 0;

    data->events_fd = dup(handle->data[0]);
    D("%s: dev=%p fd=%d (was %d)", __FUNCTION__, dev, data->events_fd, handle->data[0]);
    native_handle_close(handle);
    native_handle_delete(handle);
    return 0;
}

static int
data__data_close(struct sensors_poll_device_t *dev)
{
    SensorPoll*  data = (void*)dev;
    D("%s: dev=%p", __FUNCTION__, dev);
    if (data->events_fd >= 0) {
        close(data->events_fd);
        data->events_fd = -1;
    }
    return 0;
}

static int
pick_sensor(SensorPoll*       data,
            sensors_event_t*  values)
{
    uint32_t mask = s_enabled;
    while (mask) {
        uint32_t i = 31 - __builtin_clz(mask);
        mask &= ~(1<<i);
        if (data->pendingSensors & (1<<i)) {
            data->pendingSensors &= ~(1<<i);
            *values = data->sensors[i];
            values->timestamp = data__now_ns();
            values->sensor = i;
            values->version = sizeof(*values);

            D("%s: %d [%f, %f, %f]", __FUNCTION__,
                    i,
                    values->data[0],
                    values->data[1],
                    values->data[2]);
            return i;
        }
    }
    ALOGE("No sensor to return!!! pendingSensors=%08x", data->pendingSensors);
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(100000);
    return -EINVAL;
} 

static int parse_sensors_string(char* buff, SensorPoll* data)
{
    int64_t event_time = data__now_ns();
    int new_sensors = 0;
    float params[5];
    ALOGD("Received sensors string: %s", buff);
    /* "acceleration:<x>:<y>:<z>" corresponds to an acceleration event */
    if (sscanf(buff, "acceleration:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_ACCELERATION;
        data->sensors[ID_ACCELERATION].timestamp = event_time;
        data->sensors[ID_ACCELERATION].acceleration.x = params[0];
        data->sensors[ID_ACCELERATION].acceleration.y = params[1];
        data->sensors[ID_ACCELERATION].acceleration.z = params[2];
        set_accelerometer_props(params[0], params[1], params[2]);
    }
    /* "orientation:<azimuth>:<pitch>:<roll>" is sent when orientation changes */
    else if (sscanf(buff, "orientation:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_ORIENTATION;
        data->sensors[ID_ORIENTATION].timestamp = event_time;
        data->sensors[ID_ORIENTATION].orientation.azimuth = params[0];
        data->sensors[ID_ORIENTATION].orientation.pitch   = params[1];
        data->sensors[ID_ORIENTATION].orientation.roll    = params[2];
        data->sensors[ID_ORIENTATION].orientation.status  = SENSOR_STATUS_ACCURACY_HIGH;
        set_orientation_props(params[0], params[1], params[2]);
    }
    /* "magnetic:<x>:<y>:<z>" is sent for the params of the magnetic field */
    else if (sscanf(buff, "magnetic:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_MAGNETIC_FIELD;
        data->sensors[ID_MAGNETIC_FIELD].timestamp = event_time;
        data->sensors[ID_MAGNETIC_FIELD].magnetic.x = params[0];
        data->sensors[ID_MAGNETIC_FIELD].magnetic.y = params[1];
        data->sensors[ID_MAGNETIC_FIELD].magnetic.z = params[2];
        data->sensors[ID_MAGNETIC_FIELD].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
        set_magnetometer_props(params[0], params[1], params[2]);
    }
    /* "temperature:<celsius>" */
    else if (sscanf(buff, "temperature:%g", params+0) == 1) {
        new_sensors = SENSORS_TEMPERATURE;
        data->sensors[ID_TEMPERATURE].timestamp = event_time;
        data->sensors[ID_TEMPERATURE].temperature = params[0];
        set_temperature_props(params[0]);
    }
    /* "proximity:<value>" */
    else if (sscanf(buff, "proximity:%g", params+0) == 1) {
        new_sensors = SENSORS_PROXIMITY;
        data->sensors[ID_PROXIMITY].timestamp = event_time;
        data->sensors[ID_PROXIMITY].distance = params[0];
        set_proximity_props(params[0]);
    }
    else if (sscanf(buff, "rotation:%g:%g:%g:%g:%g", params+0, params+1, params+2, params+3, params+4) == 5) {
        new_sensors = SENSORS_ROTATION;
        data->sensors[ID_ROTATION].timestamp = event_time;
        data->sensors[ID_ROTATION].data[0] = params[0];
        data->sensors[ID_ROTATION].data[1] = params[1];
        data->sensors[ID_ROTATION].data[2] = params[2];
        data->sensors[ID_ROTATION].data[3] = params[3];
        data->sensors[ID_ROTATION].data[4] = params[4];
        set_rotation_props(params);
    }
    else if (sscanf(buff, "pressure:%g", params+0) == 1) {
        new_sensors = SENSORS_PRESSURE;
        data->sensors[ID_PRESSURE].timestamp = event_time;
        data->sensors[ID_PRESSURE].pressure = params[0];
        set_pressure_props(params[0]);
    }
    else if (sscanf(buff, "light:%g", params+0) == 1) {
        new_sensors = SENSORS_LIGHT;
        data->sensors[ID_LIGHT].timestamp = event_time;
        data->sensors[ID_LIGHT].light = params[0];
        set_light_props(params[0]);
    }
    else if (sscanf(buff, "relative_humidity:%g", params+0) == 1) {
        new_sensors = SENSORS_HUMIDITY;
        data->sensors[ID_HUMIDITY].timestamp = event_time;
        data->sensors[ID_HUMIDITY].relative_humidity = params[0];
        set_humidity_props(params[0]);
    }
    else if (sscanf(buff, "ambient_temperature:%g", params+0) == 1) {
        new_sensors = SENSORS_AMBIENT_TEMPERATURE;
        data->sensors[ID_AMBIENT_TEMPERATURE].timestamp = event_time;
        data->sensors[ID_AMBIENT_TEMPERATURE].temperature = params[0];
        set_temperature_props(params[0]);
    }
    else if (sscanf(buff, "gravity:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_GRAVITY;
        data->sensors[ID_GRAVITY].timestamp = event_time;
        data->sensors[ID_GRAVITY].acceleration.x = params[0];
        data->sensors[ID_GRAVITY].acceleration.y = params[1];
        data->sensors[ID_GRAVITY].acceleration.z = params[2];
        set_gravity_props(params[0], params[1], params[2]);
    }
    else if (sscanf(buff, "gyroscope:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_GYROSCOPE;
        data->sensors[ID_GYROSCOPE].timestamp = event_time;
        data->sensors[ID_GYROSCOPE].gyro.x = params[0];
        data->sensors[ID_GYROSCOPE].gyro.y = params[1];
        data->sensors[ID_GYROSCOPE].gyro.z = params[2];
        set_gyro_props(params[0], params[1], params[2]);
    }
    else if (sscanf(buff, "linear_acceleration:%g:%g:%g", params+0, params+1, params+2) == 3) {
        new_sensors = SENSORS_LINEAR_ACCELERATION;
        data->sensors[ID_LINEAR_ACCELERATION].timestamp = event_time;
        data->sensors[ID_LINEAR_ACCELERATION].acceleration.x = params[0];
        data->sensors[ID_LINEAR_ACCELERATION].acceleration.y = params[1];
        data->sensors[ID_LINEAR_ACCELERATION].acceleration.z = params[2];
        set_linear_acceleration_props(params[0], params[1], params[2]);
    }
    else {
        D("No sensors detected");
        return -1;
    }
    return new_sensors;
}
#ifdef AIC
#include "recv_protobuf.h"

static int s_clients[MAX_NUM_SENSORS];
static int64_t s_delay = 100000;

static int start_server(uint16_t port)
{
    int server = -1;
    struct sockaddr_in srv_addr;
    long haddr;
    bzero(&srv_addr, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = INADDR_ANY;
    srv_addr.sin_port = htons(port);
    if ((server = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        E("Unable to create sensors socket");
        exit(EXIT_FAILURE);
        return -1;
    }
    int ok = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &ok, sizeof(int));
    if (bind(server, (struct sockaddr*) &srv_addr, sizeof(srv_addr)) < 0)
    {
        E("Unable to bind sensors socket");
        exit(EXIT_FAILURE);
        return -1;
    }
    return server;
}

static int accept_client(int server)
{
    int j = 0;
    int client = accept(server, NULL, NULL);
    if (client == -1)
    {
        char* err = strerror(errno);
        ALOGD("client connection refused: %s", err);
        return -1;
    }
    for (j = 0; j < MAX_NUM_SENSORS; j++)
    {
        if (s_clients[j] == 0)
        {
            s_clients[j] = client;
            ALOGD("Client connection accepted: %d", client);
            break;
        }
    }
    if (j >= MAX_NUM_SENSORS)
    {
        char buf[256];
        char tmp[16];
        int i;
        for (i = 0; i < MAX_NUM_SENSORS; i++)
        {
            snprintf(tmp, sizeof(tmp), "%d ", s_clients[i]);
            strcat(buf, tmp);
        }
        ALOGE("Too many clients, refused: %s", buf);
        return -1;
    }
    return 0;
}


static int wait_for_client(int server)
{
    int client = -1;
    if (listen(server, 1) < 0)
    {
        ALOGE("Unable to listen");
        return -1;
    }
    client = accept(server, NULL, 0);
    if (client < 0)
    {
        ALOGE("Unable to accept() on the server socket, errno=%d", errno);
        return -1;
    }
    return client;
}

#define SENSORS_PORT 22471
static int read_data(int client, sensors_event_t* values, SensorPoll* data)
{
    uint8_t header_buffer[4];
    int byte_count;
    if ((byte_count = recv(client, header_buffer, sizeof(header_buffer), MSG_PEEK)) == -1)
        ALOGE("Sensors:: Error receiving data");
    else if (byte_count == 0)
        ALOGE("Sensors:: First read byte count is empty");
    else
    {
        uint32_t framing_size = read_header(header_buffer);
        if (framing_size < 4*1024*1024)
        {
            char buff[256];
            read_body(client, framing_size, buff);
            int mask = parse_sensors_string(buff, data);
            if (mask >= 0)
            {
                data->pendingSensors = mask;
                pick_sensor(data, values);
                return 1;
            }
        }
        else
        {
            ALOGE("Sensors:: Framing too big (%d)", framing_size);
            byte_count = 0;
        }
    }
    return 0;
}

static int use_cache(SensorPoll* data, sensors_event_t* values, int count)
{
    data->pendingSensors = s_enabled;
    int events = 0;
    while (events < count && data->pendingSensors)
    {
        pick_sensor(data, values++);
        events++;
    }
    D("Returned %d events", events);
    return events;
}

static int aic__poll(struct sensors_poll_device_t* dev, sensors_event_t* values, int count)
{
    unsigned char header_buffer[4];
    int client;
    int byte_count;
    fd_set readfs;
    SensorPoll* data = (void*)dev;
    int maxfd;
    int events_read = 0;

    while (1)
    {
        struct timeval delay;
        delay.tv_sec = 0;
        delay.tv_usec = s_delay;
        int fd = data->events_fd;
        maxfd = fd;
        FD_ZERO(&readfs);
        FD_SET(fd, &readfs);
        int i = 0;
        for (i = 0; i < MAX_NUM_SENSORS; i++)
        {
            if (s_clients[i] > 0)
            {
                FD_SET(s_clients[i], &readfs);
                if (s_clients[i] > maxfd)
                    maxfd = s_clients[i];
            }
        }
        int ret = select(maxfd + 1, &readfs, NULL, NULL, &delay);

        if (ret < 0)
        {
            if (maxfd == fd)
            {
                close(fd);
                ALOGE("Server connection closed, error: %d", errno);
                exit(EXIT_FAILURE);
            }
            else
            {
                int c;
                for (c = 0; c < MAX_NUM_SENSORS; c++)
                {
                    if (s_clients[c] > 0)
                    {
                        close(s_clients[c]);
                        s_clients[c] = 0;
                    }
                }
                ALOGE("Select failed, disconnecting clients");
                continue;
            }
        }
        if (FD_ISSET(fd, &readfs))
            accept_client(fd);
        int c;
        for (c = 0; c < MAX_NUM_SENSORS; c++)
        {
            if (FD_ISSET(s_clients[c], &readfs))
            {
                int old_events_read = events_read;
                events_read += read_data(s_clients[c], values, data);
                if (old_events_read == events_read)
                {
                    close(s_clients[c]);
                    s_clients[c] = 0;
                }
            }
            if (events_read == count)
                break;
        }
        return use_cache(data, values, count);
    }
    return 0;
}

#endif

static int
data__poll(struct sensors_poll_device_t *dev, sensors_event_t* values)
{
    SensorPoll*  data = (void*)dev;
    int fd = data->events_fd;

    D("%s: data=%p", __FUNCTION__, dev);

    // there are pending sensors, returns them now...
    if (data->pendingSensors) {
        return pick_sensor(data, values);
    }

    // wait until we get a complete event for an enabled sensor
    uint32_t new_sensors = 0;

    while (1) {
        /* read the next event */
        char     buff[256];
        int parsed_sensor;
        int      len = qemud_channel_recv(data->events_fd, buff, sizeof buff-1);
        float    params[3];
        int64_t  event_time;

        if (len < 0) {
            E("%s: len=%d, errno=%d: %s", __FUNCTION__, len, errno, strerror(errno));
            return -errno;
        }

        buff[len] = 0;

        /* "wake" is sent from the emulator to exit this loop. */
        if (!strcmp((const char*)data, "wake")) {
            return 0x7FFFFFFF;
        }

        parsed_sensor = parse_sensors_string(buff, data);
        if (parsed_sensor > 0)
            new_sensors |= parsed_sensor;

        /* "sync:<time>" is sent after a series of sensor events.
         * where 'time' is expressed in micro-seconds and corresponds
         * to the VM time when the real poll occured.
         */
        if (sscanf(buff, "sync:%lld", &event_time) == 1) {
            if (new_sensors) {
                data->pendingSensors = new_sensors;
                int64_t t = event_time * 1000LL;  /* convert to nano-seconds */

                /* use the time at the first sync: as the base for later
                 * time values */
                if (data->timeStart == 0) {
                    data->timeStart  = data__now_ns();
                    data->timeOffset = data->timeStart - t;
                }
                t += data->timeOffset;

                while (new_sensors) {
                    uint32_t i = 31 - __builtin_clz(new_sensors);
                    new_sensors &= ~(1<<i);
                    data->sensors[i].timestamp = t;
                }
                return pick_sensor(data, values);
            } else {
                D("huh ? sync without any sensor data ?");
            }
            continue;
        }
        D("huh ? unsupported command");
    }
    return -1;
}

static int
data__close(struct hw_device_t *dev)
{
    SensorPoll* data = (SensorPoll*)dev;
    if (data) {
        if (data->events_fd >= 0) {
            //ALOGD("(device close) about to close fd=%d", data->events_fd);
            close(data->events_fd);
        }
        free(data);
    }
    return 0;
}

/** SENSORS POLL DEVICE FUNCTIONS **/

static int poll__close(struct hw_device_t* dev)
{
    SensorPoll*  ctl = (void*)dev;
    close(ctl->fd);
    if (ctl->fd >= 0) {
        close(ctl->fd);
    }
    if (ctl->events_fd >= 0) {
        close(ctl->events_fd);
    }
    free(ctl);
    return 0;
}

static int poll__poll(struct sensors_poll_device_t *dev,
            sensors_event_t* data, int count)
{
    SensorPoll*  datadev = (void*)dev;
    int ret;
    int i;
    D("%s: dev=%p data=%p count=%d ", __FUNCTION__, dev, data, count);

    for (i = 0; i < count; i++)  {
        ret = data__poll(dev, data);
        data++;
        if (ret > MAX_NUM_SENSORS || ret < 0) {
           return i;
        }
        if (!datadev->pendingSensors) {
           return i + 1;
        }
    }
    return count;
}

static int poll__activate(struct sensors_poll_device_t *dev,
            int handle, int enabled)
{
    int ret;
    native_handle_t* hdl;
    SensorPoll*  ctl = (void*)dev;
    D("%s: dev=%p handle=%x enable=%d ", __FUNCTION__, dev, handle, enabled);
#ifndef AIC
    if (ctl->fd < 0) {
        D("%s: OPEN CTRL and DATA ", __FUNCTION__);
        hdl = control__open_data_source(dev);
        ret = data__data_open(dev,hdl);
    }
    ret = control__activate(dev, handle, enabled);
#else
    int value = (1 << handle);
    if (enabled == 0)
    {
        if (s_enabled & value)
            s_enabled -= (1 << handle);
    }
    else
    {
        if (!(s_enabled & value))
            s_enabled |= (1 << handle);
    }
    ret = 0;
#endif
    return ret;
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
            int handle, int64_t ns)
{
    // TODO
#ifdef AIC
    s_delay = ns / 1000ULL;
#endif
    return 0;
}

/** MODULE REGISTRATION SUPPORT
 **
 ** This is required so that hardware/libhardware/hardware.c
 ** will dlopen() this library appropriately.
 **/

/*
 * the following is the list of all supported sensors.
 * this table is used to build sSensorList declared below
 * according to which hardware sensors are reported as
 * available from the emulator (see get_sensors_list below)
 *
 * note: numerical values for maxRange/resolution/power were
 *       taken from the reference AK8976A implementation
 */
static const struct sensor_t sSensorListInit[] = {
        { .name       = "Goldfish 3-axis Accelerometer",
          .vendor     = "The Android Open Source Project",
          .version    = 1,
          .handle     = ID_ACCELERATION,
          .type       = SENSOR_TYPE_ACCELEROMETER,
          .maxRange   = MAX_RANGE,
          .resolution = 0.1f,
          .power      = 3.0f,
          .minDelay   = 5000,
          .reserved   = {}
        },

        { .name       = "Goldfish 3-axis Magnetic field sensor",
          .vendor     = "The Android Open Source Project",
          .version    = 1,
          .handle     = ID_MAGNETIC_FIELD,
          .type       = SENSOR_TYPE_MAGNETIC_FIELD,
          .maxRange   = 2000.0f,
          .resolution = 1.0f,
          .power      = 6.7f,
          .reserved   = {}
        },

        { .name       = "Goldfish Orientation sensor",
          .vendor     = "The Android Open Source Project",
          .version    = 1,
          .handle     = ID_ORIENTATION,
          .type       = SENSOR_TYPE_ORIENTATION,
          .maxRange   = 360.0f,
          .resolution = 1.0f,
          .power      = 9.7f,
          .reserved   = {}
        },

        { .name       = "AiC Gyroscope sensor",
          .vendor     = "AiC",
          .handle     = ID_GYROSCOPE,
          .type       = SENSOR_TYPE_GYROSCOPE,
          .maxRange   = MAX_RANGE,
          .resolution = 0.5f,
          .power      = 0.57f,
          .reserved   = {}
        },

        { .name       = "AiC Light sensor",
          .vendor     = "AiC",
          .version    = 1,
          .handle     = ID_LIGHT,
          .type       = SENSOR_TYPE_LIGHT,
          .maxRange   = 10000.0f,
          .resolution = 1.0f,
          .power      = 0.57f,
          .reserved   = {}
        },

        { .name       = "AiC Pressure sensor",
          .vendor     = "AiC",
          .version    = 1,
          .handle     = ID_PRESSURE,
          .type       = SENSOR_TYPE_PRESSURE,
          .maxRange   = 5000.0f,
          .resolution = 1.0f,
          .power      = 0.57f,
          .reserved   = {}
        },
        { .name       = "Goldfish Temperature sensor",
          .vendor     = "The Android Open Source Project",
          .version    = 1,
          .handle     = ID_TEMPERATURE,
          .type       = SENSOR_TYPE_TEMPERATURE,
          .maxRange   = 100.0f,
          .resolution = 1.0f,
          .power      = 0.0f,
          .reserved   = {}
        },

        { .name       = "Goldfish Proximity sensor",
          .vendor     = "The Android Open Source Project",
          .version    = 1,
          .handle     = ID_PROXIMITY,
          .type       = SENSOR_TYPE_PROXIMITY,
          .maxRange   = 100.0f,
          .resolution = 1.0f,
          .power      = 20.0f,
          .reserved   = {}
        },

        { .name       = "AiC Gravity sensor",
          .vendor     = "AiC",
          .version    = 1,
          .handle     = ID_GRAVITY,
          .type       = SENSOR_TYPE_GRAVITY,
          .maxRange   = MAX_RANGE,
          .resolution = 0.5f,
          .power      = 0.57f,
          .reserved   = {}
        },
        { .name       = "AiC Linear Acceleration sensor",
          .vendor     = "AiC",
          .handle     = ID_LINEAR_ACCELERATION,
          .type       = SENSOR_TYPE_LINEAR_ACCELERATION,
          .maxRange   = MAX_RANGE,
          .resolution = 0.2f,
          .power      = 0.57f,
          .reserved   = {}
        },
        { .name       = "AiC Rotation Vector sensor",
          .vendor     = "AiC",
          .handle     = ID_ROTATION,
          .type       = SENSOR_TYPE_ROTATION_VECTOR,
          .maxRange   = MAX_RANGE,
          .resolution = 0.1f,
          .power      = 0.57f,
          .reserved   = {}
        },

        { .name       = "AiC Relative Humidity sensor",
          .vendor     = "AiC",
          .handle     = ID_HUMIDITY,
          .type       = SENSOR_TYPE_RELATIVE_HUMIDITY,
          .maxRange   = 100.0f,
          .resolution = 1.0f,
          .power      = 0.57f,
          .reserved   = {}
        },
        { .name       = "AiC Ambient Temperature sensor",
          .vendor     = "AiC",
          .version    = 1,
          .handle     = ID_AMBIENT_TEMPERATURE,
          .type       = SENSOR_TYPE_AMBIENT_TEMPERATURE,
          .maxRange   = 100.0f,
          .resolution = 0.5f,
          .power      = 0.57f,
          .reserved   = {}
        }
};

static struct sensor_t  sSensorList[MAX_NUM_SENSORS];

static int sensors__get_sensors_list(struct sensors_module_t* module,
        struct sensor_t const** list)
{
    char buffer[12];
    int  mask, nn, count;

    int  ret;
#ifndef AIC
    int  fd = qemud_channel_open(SENSORS_SERVICE_NAME);
    if (fd < 0) {
        E("%s: no qemud connection", __FUNCTION__);
        return 0;
    }
    ret = qemud_channel_send(fd, "list-sensors", -1);
    if (ret < 0) {
        E("%s: could not query sensor list: %s", __FUNCTION__,
          strerror(errno));
        close(fd);
        return 0;
    }
    ret = qemud_channel_recv(fd, buffer, sizeof buffer-1);
    if (ret < 0) {
        E("%s: could not receive sensor list: %s", __FUNCTION__,
          strerror(errno));
        close(fd);
        return 0;
    }
    buffer[ret] = 0;
    close(fd);

    /* the result is a integer used as a mask for available sensors */
    mask  = atoi(buffer);
#else
    mask = 0x1FFF;
#endif
    count = 0;
    for (nn = 0; nn < MAX_NUM_SENSORS; nn++) {
        if (((1 << nn) & mask) == 0)
            continue;

        sSensorList[count++] = sSensorListInit[nn];
    }
    D("%s: returned %d sensors (mask=%d)", __FUNCTION__, count, mask);
    *list = sSensorList;

    return count;
}


static int
open_sensors(const struct hw_module_t* module,
             const char*               name,
             struct hw_device_t*      *device)
{
    int  status = -EINVAL;

    D("%s: name=%s", __FUNCTION__, name);

    if (!strcmp(name, SENSORS_HARDWARE_POLL)) {
        SensorPoll *dev = malloc(sizeof(*dev));

        memset(dev, 0, sizeof(*dev));

        dev->device.common.tag     = HARDWARE_DEVICE_TAG;
        dev->device.common.version = 0;
        dev->device.common.module  = (struct hw_module_t*) module;
        dev->device.common.close   = poll__close;
        dev->device.poll           = aic__poll;
        dev->device.activate       = poll__activate;
        dev->device.setDelay       = poll__setDelay;
        dev->events_fd             = -1;
        dev->fd                    = -1;

        *device = &dev->device.common;
        status  = 0;

        dev->events_fd = socket_inaddr_any_server(SENSORS_PORT, SOCK_STREAM);
        memset(s_clients, 0, sizeof(s_clients));

        // Initialize default sensors values
        int64_t timestamp = data__now_ns();

        dev->sensors[ID_ACCELERATION].type = SENSOR_TYPE_ACCELEROMETER;
        dev->sensors[ID_ACCELERATION].timestamp = timestamp;
        dev->sensors[ID_ACCELERATION].sensor = ID_ACCELERATION;
        dev->sensors[ID_ACCELERATION].version = sizeof(sensors_event_t);
        dev->sensors[ID_ACCELERATION].acceleration.x = 0;
        dev->sensors[ID_ACCELERATION].acceleration.y = 9.776219;
        dev->sensors[ID_ACCELERATION].acceleration.x = 0.813417;
        set_accelerometer_props(0., 9.776219, 0.813417);

        dev->sensors[ID_ORIENTATION].type = SENSOR_TYPE_ORIENTATION;
        dev->sensors[ID_ORIENTATION].timestamp = timestamp;
        dev->sensors[ID_ORIENTATION].sensor = ID_ORIENTATION;
        dev->sensors[ID_ORIENTATION].version = sizeof(sensors_event_t);
        dev->sensors[ID_ORIENTATION].orientation.azimuth = 0.;
        dev->sensors[ID_ORIENTATION].orientation.pitch = 0.;
        dev->sensors[ID_ORIENTATION].orientation.roll = 0.;
        set_orientation_props(0., 0., 0.);

        dev->sensors[ID_PROXIMITY].type = SENSOR_TYPE_PROXIMITY;
        dev->sensors[ID_PROXIMITY].timestamp = timestamp;
        dev->sensors[ID_PROXIMITY].sensor = ID_PROXIMITY;
        dev->sensors[ID_PROXIMITY].version = sizeof(sensors_event_t);
        dev->sensors[ID_PROXIMITY].distance = 8.;
        set_proximity_props(8.);

        dev->sensors[ID_PRESSURE].type = SENSOR_TYPE_PRESSURE;
        dev->sensors[ID_PRESSURE].timestamp = timestamp;
        dev->sensors[ID_PRESSURE].sensor = ID_PRESSURE;
        dev->sensors[ID_PRESSURE].version = sizeof(sensors_event_t);
        dev->sensors[ID_PRESSURE].pressure = 999.;
        set_pressure_props(999.);

        dev->sensors[ID_LIGHT].type = SENSOR_TYPE_LIGHT;
        dev->sensors[ID_LIGHT].timestamp = timestamp;
        dev->sensors[ID_LIGHT].sensor = ID_LIGHT;
        dev->sensors[ID_LIGHT].version = sizeof(sensors_event_t);
        dev->sensors[ID_LIGHT].light = 1000.;
        set_light_props(1000.);

        dev->sensors[ID_AMBIENT_TEMPERATURE].type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
        dev->sensors[ID_AMBIENT_TEMPERATURE].timestamp = timestamp;
        dev->sensors[ID_AMBIENT_TEMPERATURE].sensor = ID_AMBIENT_TEMPERATURE;
        dev->sensors[ID_AMBIENT_TEMPERATURE].version = sizeof(sensors_event_t);
        dev->sensors[ID_AMBIENT_TEMPERATURE].temperature = 20.;
        set_temperature_props(20.);

        dev->sensors[ID_HUMIDITY].type = SENSOR_TYPE_RELATIVE_HUMIDITY;
        dev->sensors[ID_HUMIDITY].timestamp = timestamp;
        dev->sensors[ID_HUMIDITY].sensor = ID_HUMIDITY;
        dev->sensors[ID_HUMIDITY].version = sizeof(sensors_event_t);
        dev->sensors[ID_HUMIDITY].relative_humidity = 50.;
        set_humidity_props(50.);

        dev->sensors[ID_GRAVITY].type = SENSOR_TYPE_GRAVITY;
        dev->sensors[ID_GRAVITY].timestamp = timestamp;
        dev->sensors[ID_GRAVITY].sensor = ID_GRAVITY;
        dev->sensors[ID_GRAVITY].version = sizeof(sensors_event_t);
        dev->sensors[ID_GRAVITY].acceleration.x = 0;
        dev->sensors[ID_GRAVITY].acceleration.y = 9.776219;
        dev->sensors[ID_GRAVITY].acceleration.z = 0.813417;
        set_gravity_props(0., 9.776219, 0.813417);

        dev->sensors[ID_GYROSCOPE].type = SENSOR_TYPE_GYROSCOPE;
        dev->sensors[ID_GYROSCOPE].timestamp = timestamp;
        dev->sensors[ID_GYROSCOPE].sensor = ID_GYROSCOPE;
        dev->sensors[ID_GYROSCOPE].version = sizeof(sensors_event_t);
        dev->sensors[ID_GYROSCOPE].data[0] = 0.;
        dev->sensors[ID_GYROSCOPE].data[1] = 0.;
        dev->sensors[ID_GYROSCOPE].data[2] = 0.;
        set_gyro_props(0., 0., 0.);

        dev->sensors[ID_LINEAR_ACCELERATION].type = SENSOR_TYPE_LINEAR_ACCELERATION;
        dev->sensors[ID_LINEAR_ACCELERATION].timestamp = timestamp;
        dev->sensors[ID_LINEAR_ACCELERATION].sensor = ID_LINEAR_ACCELERATION;
        dev->sensors[ID_LINEAR_ACCELERATION].version = sizeof(sensors_event_t);
        dev->sensors[ID_LINEAR_ACCELERATION].acceleration.x = 0.;
        dev->sensors[ID_LINEAR_ACCELERATION].acceleration.y = 0.;
        dev->sensors[ID_LINEAR_ACCELERATION].acceleration.z = 0.;
        set_linear_acceleration_props(0., 0., 0.);

        dev->sensors[ID_ROTATION].type = SENSOR_TYPE_ROTATION_VECTOR;
        dev->sensors[ID_ROTATION].timestamp = timestamp;
        dev->sensors[ID_ROTATION].sensor = ID_ROTATION;
        dev->sensors[ID_ROTATION].version = sizeof(sensors_event_t);
        dev->sensors[ID_ROTATION].data[0] = 0.;
        dev->sensors[ID_ROTATION].data[1] = 0.;
        dev->sensors[ID_ROTATION].data[2] = 1.;
        dev->sensors[ID_ROTATION].data[3] = 0.;
        dev->sensors[ID_ROTATION].data[4] = 0.;
        set_rotation_props(dev->sensors[ID_ROTATION].data);

        dev->sensors[ID_MAGNETIC_FIELD].type = SENSOR_TYPE_MAGNETIC_FIELD;
        dev->sensors[ID_MAGNETIC_FIELD].timestamp = timestamp;
        dev->sensors[ID_MAGNETIC_FIELD].sensor = ID_MAGNETIC_FIELD;
        dev->sensors[ID_MAGNETIC_FIELD].version = sizeof(sensors_event_t);
        dev->sensors[ID_MAGNETIC_FIELD].magnetic.x = 7.;
        dev->sensors[ID_MAGNETIC_FIELD].magnetic.y = 8.;
        dev->sensors[ID_MAGNETIC_FIELD].magnetic.z = 9.;
        set_linear_acceleration_props(7., 8., 9.);

        dev->sensors[ID_TEMPERATURE].type = SENSOR_TYPE_TEMPERATURE;
        dev->sensors[ID_TEMPERATURE].timestamp = timestamp;
        dev->sensors[ID_TEMPERATURE].sensor = ID_TEMPERATURE;
        dev->sensors[ID_TEMPERATURE].version = sizeof(sensors_event_t);
    }
    return status;
}


static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "Goldfish SENSORS Module",
        .author = "The Android Open Source Project",
        .methods = &sensors_module_methods,
    },
    .get_sensors_list = sensors__get_sensors_list
};
