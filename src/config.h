#ifndef __CONFIG_H
#define __CONFIG_H

Sensors config();

//Nathan
//Struct with the id of each sensors
typedef struct _sensors_t{
  uint8_t color_sensor;
  uint8_t gyro_sensor;
  uint8_t ultrasonic_sensor;
  uint8_t compass_sensor;
  uint8_t touch_sensor;
} sensors_t;

//Array with the type of each sensor
const uint8_t sensors_type[] = { LEGO_EV3_COLOR, LEGO_EV3_GYRO, LEGO_EV3_US, HT_NXT_COMPASS, LEGO_EV3_TOUCH};
const int number_of_sensors = sizeof(sensors_type);

#endif
