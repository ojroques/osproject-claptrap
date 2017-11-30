#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "sensors.h"

/**
Set of function which control the sensors.
**/

/**
 *Function which gives the color detected by the sensor.
 *Returns an int between 0 and ?
 **/
int get_color(uint8_t sensor_id){
  int val;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != LEGO_EV3_COLOR_COL_COLOR) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_COL_COLOR);
  }
  if ( !get_sensor_value( 0, sensor_id, &val ) || ( val < 0 ) ) {
    val = 0;
  }
  return val;
}

/**
 *Function which gives the percentage of reflected ligth.
 *Returns an int between 0 and 100.
 **/
int get_reflection(uint8_t sensor_id){
  int val;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != LEGO_EV3_COLOR_COL_REFLECT) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_COL_REFLECT);
  }
  if ( !get_sensor_value( 0, sensor_id, &val ) || ( val < 0 ) ) {
    val = 0;
  }
  return val;
}

/**
 *Function which gives the percentage of ambient ligth.
 *Returns an int between 0 and 100.
 **/
int get_ambient(uint8_t sensor_id){
  int val;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != LEGO_EV3_COLOR_COL_AMBIENT) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_COL_AMBIENT);
  }
  if ( !get_sensor_value( 0, sensor_id, &val ) || ( val < 0 ) ) {
    val = 0;
  }
  return val;
}

/**
 *Function which gives the raw reflected light.
 *Returns an two int between 0 and 1020 in a struct ReflectedLight.
 **/
RawReflectedLight get_raw_reflected(uint8_t sensor_id){
  RawReflectedLight ref;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != LEGO_EV3_COLOR_REF_RAW) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_REF_RAW);
  }
  if ( !get_sensor_value( 0, sensor_id, &(ref.value0) ) || ( ref.value0 < 0 ) ) {
    ref.value0 = 0;
  }
  if ( !get_sensor_value( 1, sensor_id, &(ref.value1) ) || ( ref.value1 < 0 ) ) {
    ref.value1 = 0;
  }
  return ref;
}


/**
 *Function which gives the raw RGB values.
 *Returns an three int between 0 and 1020 in a struct RawRGB.
 **/
RawRGB get_raw_rgb(uint8_t sensor_id){
  RawRGB rgb;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != LEGO_EV3_COLOR_RGB_RAW) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_RGB_RAW);
  }
  if ( !get_sensor_value( 0, sensor_id, &(rgb.value0) ) || ( rgb.value0 < 0 ) ) {
    rgb.value0 = 0;
  }
  if ( !get_sensor_value( 1, sensor_id, &(rgb.value1) ) || ( rgb.value1 < 0 ) ) {
    rgb.value1 = 0;
  }
  if ( !get_sensor_value( 2, sensor_id, &(rgb.value2) ) || ( rgb.value2 < 0 ) ) {
    rgb.value2 = 0;
  }
  return rgb;

}
