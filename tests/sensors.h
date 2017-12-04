/*Written by Nathan Biette for the OS project at Eurecom 2017 - 2018*/

#ifndef SENSORS_H
#define SENSORS_H

#include "ev3_sensor.h"

typedef struct _DoubleValue{
  int value0;
  int value1;
} DoubleValue;

typedef struct _TripleValue{
  int value0;
  int value1;
  int value2;
} TripleValue;

//Color sensor access functions
int get_color(uint8_t sensor_id);
int get_reflection(uint8_t sensor_id);
int get_ambient(uint8_t sensor_id);
DoubleValue get_raw_reflected(uint8_t sensor_id);
TripleValue get_raw_rgb(uint8_t sensor_id);
//Gyro sensor access functions
int get_angle(uint8_t sensor_id);
int get_rot_speed(uint8_t sensor_id);
int get_raw_gyro(uint8_t sensor_id);
DoubleValue get_angle_and_rot_speed(uint8_t sensor_id);
//Ultra sonic sensor access functions
int get_distance(uint8_t sensor_id);
int get_single_dist(uint8_t sensor_id);
int get_continuous_distance(uint8_t sensor_id);
//Compass sensor access functions
int get_compass_direction(uint8_t sensor_id);
#endif
