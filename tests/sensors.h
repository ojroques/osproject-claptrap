#ifndef SENSORS_H
#define SENSORS_H

#include "ev3_sensor.h"

typedef struct _RawReflectedLight{
  int value0;
  int value1;
} RawReflectedLight;

typedef struct _RawRGB{
  int value0;
  int value1;
  int value3;
} RawRGB;

int get_color(uint8_t sensor_id);
int get_ambient(uint8_t sensor_id);
int get_reflection(uint8_t sensor_id);
RawReflectedLight get_raw_reflected(uint8_t sensor_id);
RawRGB get_raw_rgb(uint8_t sensor_id);

#endif
