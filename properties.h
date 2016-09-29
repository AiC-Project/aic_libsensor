#ifndef __AIC_SESNORS_PROPS__
#define __AIC_SENSORS_PROPS__
void set_accelerometer_props(float x, float y, float z);
void set_pressure_props(float pressure);
void set_gravity_props(float x, float y, float z);
void set_gyro_props(float azimuth, float pitch, float roll);
void set_humidity_props(float humidity);
void set_linear_acceleration_props(float x, float y, float z);
void set_light_props(float light);
void set_magnetometer_props(float x, float y, float z);
void set_orientation_props(float azimuth, float pitch, float roll);
void set_rotation_props(float* data);
void set_proximity_props(float distance);
void set_temperature_props(float temperature);
#endif
