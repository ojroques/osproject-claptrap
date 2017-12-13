#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "const.h"
#include "config.h"
#include "image.h"
#include "ev3.h"
#include "ev3_tacho.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

//Nathan
void search_sensor( uint8_t sensor_type, uint8_t *sensor_id ){
  if ( ev3_search_sensor (sensor_type, sensor_id, 0 )) {
      printf("Sensor found!\n");
  } else {
      printf("no sensor found\n");
  }
}

//Erwan
void config_tacho(){
  uint8_t lsn;
  uint8_t rsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
    }
  }
}

sensors_t config(){
  config_tacho();
  init_image();
  //Nathan
  sensors_t sensors = {0, 0, 0, 0, 0};
  ev3_sensor_init();
  search_sensor(LEGO_EV3_COLOR, &(sensors.color_sensor));
  search_sensor(LEGO_EV3_GYRO, &(sensors.gyro_sensor));
  search_sensor(LEGO_EV3_US, &(sensors.ultrasonic_sensor));
  search_sensor(HT_NXT_COMPASS, &(sensors.compass_sensor));
  search_sensor(LEGO_EV3_TOUCH, &(sensors.touch_sensor));
  return sensors;
}
