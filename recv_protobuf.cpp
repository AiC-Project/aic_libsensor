#include <sys/socket.h>
#include <cutils/log.h>
#include <cutils/properties.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <sensors_packet.pb.h>

#include "recv_protobuf.h"
#include "aic.h"

uint32_t read_header(unsigned char *buf)
{
  google::protobuf::uint32 size;
  google::protobuf::io::ArrayInputStream ais(buf,4);
  google::protobuf::io::CodedInputStream coded_input(&ais);
  coded_input.ReadVarint32(&size);
  ALOGD("read_header: read byte count is %d", size);
  return size;
}

uint32_t read_body(int csock, uint32_t size, char* fmt_buffer)
{
    int bytecount;
    int written = 0;
    const uint32_t full_size = size + 4;
    sensors_packet payload;
    char* buffer = new char[full_size];
    if((bytecount = recv(csock, (void *)buffer, full_size, MSG_WAITALL))== -1)
    {
        ALOGE("read_body: Error receiving data (%d)", errno);
        delete[] buffer;
        return written;
    }
    else if (bytecount != full_size)
    {
        ALOGE("read_body: Received the wrong payload size (expected %d, received %d)", full_size, bytecount);
        delete[] buffer;
        return written;
    }
    ALOGD("read_body: Second read byte count is %d", bytecount);
    google::protobuf::io::ArrayInputStream ais(buffer, full_size);
    google::protobuf::io::CodedInputStream coded_input(&ais);
    coded_input.ReadVarint32(&size);
    google::protobuf::io::CodedInputStream::Limit msgLimit = coded_input.PushLimit(size);
    payload.ParseFromCodedStream(&coded_input);
    coded_input.PopLimit(msgLimit);

    if(payload.has_sensor_accelerometer()){
        if (payload.sensor_accelerometer().has_x() &&
                payload.sensor_accelerometer().has_y() &&
                payload.sensor_accelerometer().has_z() ){
            written = snprintf(fmt_buffer, 128, "acceleration:%f:%f:%f",
                    payload.sensor_accelerometer().x(),
                    payload.sensor_accelerometer().y(),
                    payload.sensor_accelerometer().z());
        }
    }

    if(payload.has_sensor_magnetometer()){
        if (payload.sensor_magnetometer().has_x() &&
                payload.sensor_magnetometer().has_y() &&
                payload.sensor_magnetometer().has_z() ){
            written = snprintf(fmt_buffer, 128, "magnetic:%f:%f:%f",
                    payload.sensor_magnetometer().x(),
                    payload.sensor_magnetometer().y(),
                    payload.sensor_magnetometer().z());
        }
    }

    if(payload.has_sensor_orientation()){
        if (payload.sensor_orientation().has_azimuth() &&
                payload.sensor_orientation().has_pitch() &&
                payload.sensor_orientation().has_roll() ){
            written = snprintf(fmt_buffer, 128, "orientation:%f:%f:%f", 
                    payload.sensor_orientation().azimuth(),
                    payload.sensor_orientation().pitch(),
                    payload.sensor_orientation().roll());
        }
    }

    if(payload.has_sensor_gyroscope()){
        if (payload.sensor_gyroscope().has_azimuth() &&
                payload.sensor_gyroscope().has_pitch() &&
                payload.sensor_gyroscope().has_roll() ){
            written = snprintf(fmt_buffer, 128, "gyroscope:%f:%f:%f",
                    payload.sensor_gyroscope().azimuth(),
                    payload.sensor_gyroscope().pitch(),
                    payload.sensor_gyroscope().roll());
        }
    }

    if(payload.has_sensor_gravity()){
        if (payload.sensor_gravity().has_x() &&
                payload.sensor_gravity().has_y() &&
                payload.sensor_gravity().has_z() ){
            written = snprintf(fmt_buffer, 128, "gravity:%f:%f:%f",
                    payload.sensor_gravity().x(),
                    payload.sensor_gravity().y(),
                    payload.sensor_gravity().z());
        }
    }

    if(payload.has_sensor_rot_vector()){
        if (payload.sensor_rot_vector().size() > 4 ){
            written = snprintf(fmt_buffer, 128, "rotation:%f:%f:%f:%f:%f",
                payload.sensor_rot_vector().data().Get(0),
                payload.sensor_rot_vector().data().Get(1),
                payload.sensor_rot_vector().data().Get(2),
                payload.sensor_rot_vector().data().Get(3),
                payload.sensor_rot_vector().data().Get(4));
        }
    }

    if(payload.has_sensor_linear_acc()){
        if (payload.sensor_linear_acc().has_x() &&
                payload.sensor_linear_acc().has_y() &&
                payload.sensor_linear_acc().has_z() ){
            written = snprintf(fmt_buffer, 128, "linear_acceleration:%f:%f:%f",
                    payload.sensor_linear_acc().x(),
                    payload.sensor_linear_acc().y(),
                    payload.sensor_linear_acc().z());
        }
    }

    if(payload.has_sensor_temperature()){
        if (payload.sensor_temperature().has_temperature() )
            written = snprintf(fmt_buffer, 128, "ambient_temperature:%f",
                    payload.sensor_temperature().temperature());
    }

    if(payload.has_sensor_proximity()){
        if (payload.sensor_proximity().has_distance() )
            written = snprintf(fmt_buffer, 128, "proximity:%f",
                    payload.sensor_proximity().distance());
    }

    if(payload.has_sensor_light()){
        if (payload.sensor_light().has_light() )
            written = snprintf(fmt_buffer, 128, "light:%f",
                    payload.sensor_light().light());
    }

    if(payload.has_sensor_pressure()){
        if (payload.sensor_pressure().has_pressure() )
            written = snprintf(fmt_buffer, 128, "pressure:%f",
                    payload.sensor_pressure().pressure());
    }

    if(payload.has_sensor_relative_humidity()){
        if (payload.sensor_relative_humidity().has_relative_humidity() )
            written = snprintf(fmt_buffer, 128, "relative_humidity:%f",
                    payload.sensor_relative_humidity().relative_humidity());
    }
    delete[] buffer;
    return written;
}
