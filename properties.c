#include <cutils/properties.h>
#include "aic.h"

#define BUFFER_MAX 128

static void prop_float(char* property, float value)
{
    char buffer[BUFFER_MAX];
    snprintf(buffer, BUFFER_MAX, "%lf", value);
    property_set(property, buffer);
}

void set_accelerometer_props(float x, float y, float z)
{
    prop_float(ACCELEROMETER_X, x);
    prop_float(ACCELEROMETER_Y, y);
    prop_float(ACCELEROMETER_Z, z);
}

void set_pressure_props(float pressure)
{
    prop_float(PRESSURE, pressure);
}

void set_gravity_props(float x, float y, float z)
{
    prop_float(GRAVITY_X, x);
    prop_float(GRAVITY_Y, y);
    prop_float(GRAVITY_Z, z);
}

void set_gyro_props(float azimuth, float pitch, float roll)
{
    prop_float(GYROSCOPE_AZIMUTH, azimuth);
    prop_float(GYROSCOPE_PITCH, pitch);
    prop_float(GYROSCOPE_ROLL, roll);
}

void set_humidity_props(float humidity)
{
    prop_float(HUMIDITY, humidity);
}

void set_linear_acceleration_props(float x, float y, float z)
{
    prop_float(LINEAR_ACC_X, x);
    prop_float(LINEAR_ACC_Y, y);
    prop_float(LINEAR_ACC_Z, z);
}

void set_light_props(float light)
{
    prop_float(LIGHT, light);
}

void set_magnetometer_props(float x, float y, float z)
{
    prop_float(MAGNETOMETER_X, x);
    prop_float(MAGNETOMETER_Y, y);
    prop_float(MAGNETOMETER_Z, z);
}

void set_orientation_props(float azimuth, float pitch, float roll)
{
    prop_float(ORIENTATION_AZIMUTH, azimuth);
    prop_float(ORIENTATION_PITCH, pitch);
    prop_float(ORIENTATION_ROLL, roll);
}

void set_rotation_props(float* data)
{
}

void set_proximity_props(float distance)
{
    prop_float(DISTANCE, distance);
}

void set_temperature_props(float temperature)
{
    prop_float(TEMPERATURE, temperature);
}
