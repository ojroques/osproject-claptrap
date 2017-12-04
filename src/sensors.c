/*Written by Nathan Biette for the OS project at Eurecom 2017 - 2018*/

#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "sensors.h"

const int DEBUG = 0;

/**
Set of function which control the sensors.
**/

//purely technical function to factorize code of sensor reading functions

int retrieve_single_value(uint8_t sensor_id, uint8_t sensor_mode){
  int val;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != sensor_mode) {
    set_sensor_mode_inx(sensor_id, sensor_mode);
  }
  //test if the we managed to get a value, if not return default value 0
  if ( !get_sensor_value( 0, sensor_id, &val ) ){
    val = 0;
  }
  return val;
}

//Same for double value

DoubleValue retrieve_double_value(uint8_t sensor_id, uint8_t sensor_mode){
  DoubleValue ref;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != sensor_mode) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_REF_RAW);
  }
  if ( !get_sensor_value( 0, sensor_id, &(ref.value0) ) ) {
    ref.value0 = 0;
  }
  if ( !get_sensor_value( 1, sensor_id, &(ref.value1) ) ) {
    ref.value1 = 0;
  }
  return ref;
}
//Same for triple value

TripleValue retrieve_triple_value(uint8_t sensor_id, uint8_t sensor_mode){
  TripleValue ref;
  char s[ 256 ];
  if (get_sensor_mode(sensor_id, s, sizeof(s)) != sensor_mode) {
    set_sensor_mode_inx(sensor_id, LEGO_EV3_COLOR_REF_RAW);
  }
  if ( !get_sensor_value( 0, sensor_id, &(ref.value0) ) ) {
    ref.value0 = 0;
  }
  if ( !get_sensor_value( 1, sensor_id, &(ref.value1) ) ) {
    ref.value1 = 0;
  }
  if ( !get_sensor_value( 2, sensor_id, &(ref.value2) ) ) {
    ref.value2 = 0;
  }
  return ref;
}


//########################LIGHT_SENSOR##############################

/**
 *Function which gives the color detected by the sensor.
 *Returns an int between 0 and ?
 **/
int get_color(uint8_t sensor_id){
  int color = retrieve_single_value(sensor_id, LEGO_EV3_COLOR_COL_COLOR);
  if (color < 0 || color > 7){
    if(DEBUG){
      printf("color out of range : %d \n", color);
    }
    color = 0;
  }
  return color;
}

/**
 *Function which gives the percentage of reflected ligth.
 *Returns an int between 0 and 100.
 **/
int get_reflection(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_COLOR_COL_REFLECT);
}

/**
 *Function which gives the percentage of ambient ligth.
 *Returns an int between 0 and 100.
 **/
int get_ambient(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_COLOR_COL_AMBIENT);
}

/**
 *Function which gives the raw reflected light.
 *Returns an two int between 0 and 1020 in a struct ReflectedLight.
 **/
DoubleValue get_raw_reflected(uint8_t sensor_id){
  return retrieve_double_value(sensor_id, LEGO_EV3_COLOR_REF_RAW);
}

/**
 *Function which gives the raw RGB values.
 *Returns an three int between 0 and 1020 in a struct RawRGB.
 **/
TripleValue get_raw_rgb(uint8_t sensor_id){
  return retrieve_triple_value(sensor_id,LEGO_EV3_COLOR_RGB_RAW);
}

//##################GYRO_SENSOR################################

/**
 *Function which gives the angle in degrees detected by the gyro sensor.
 *Returns an int between -32768 and 32767
 **/
int get_angle(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_GYRO_GYRO_ANG);
}

/**
 *Function which gives the rotation speed in degree per seconds detected by the gyro sensor.
 *Returns an int between -440 and 440
 **/
int get_rot_speed(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_GYRO_GYRO_RATE);
}

/**
 *Function which gives the raw sensor value detected by the gyro sensor.
 *Returns an int between -1464 and 1535
 **/
int get_raw_gyro(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_GYRO_GYRO_FAS);
}

/**
 *Function which gives the angle and the rotational speed value detected by the gyro sensor.
 *Returns an angle (int) between -32768 and 32767
 *Returns a speed (int) between -440 and 440
 **/
DoubleValue get_angle_and_rot_speed(uint8_t sensor_id){
  return retrieve_double_value(sensor_id, LEGO_EV3_GYRO_GYRO_G_AND_A);
}

//####################ULTRA_SONIC_ENSOR################################
//NOTE : the sensor might lock if the mode is written too often
//--> maybe we should be aware of reading too many times the mode get_color

/**
 *Function which gives the distance in cm detected by the gyro sensor.
 *Returns an int between 0 and 2550
 *Doesn't poweroff the ultrasonic sensor
 **/
int get_distance(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_US_US_DIST_CM);
}

/**
 *Function which gives a single distance measurement in cm
 *detected by the gyro sensor and then power off the sensor
 *Returns an int between 0 and 2550
 **/
int get_single_dist(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_US_US_SI_CM);
}

/**
 *Function which gives continuously the distance in cm detected by the gyro sensor.
 *Returns an int between 0 and 2550
 **/
int get_continuous_distance(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, LEGO_EV3_US_US_DC_CM);
}

//###############################COMPASS##############################

/**
 *Function which gives the angle in degrees detected by the compass sensor.
 *Returns an int between 0 and 359
 **/
int get_compass_direction(uint8_t sensor_id){
  return retrieve_single_value(sensor_id, HT_NXT_COMPASS_COMPASS);
}
