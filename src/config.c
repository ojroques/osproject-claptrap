#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "../robot_movement/configuration.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

//Nathan
void search_sensor( uint8_t sensor_type, uint8_t sensor_id ){
  if ( ev3_search_sensor( sensor_type, &sensor_id, 0 )) {
      printf("sensor of type %d found \n", sensor_type);
      printf("sensor number for sensor is %d \n", sensor_id);
      printf("    Port = %s\n", ev3_sensor_port_name(sensor_id, s ));
  } else {
      printf("no sensor of type %d found\n", sensor_type);
  }
}

//Erwan
sensors_t config(){

  uint8_t lsn;
  uint8_t rsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
    }
  }

  //Nathan
  sensors_t sensors;
  ev3_sensor_init();
  search_sensor(LEGO_EV3_COLOR, Sensors.color_sensor);
  search_sensor(LEGO_EV3_GYRO, Sensors.gyro_sensor);
  search_sensor(LEGO_EV3_US, Sensors.ultrasonic_sensor);
  search_sensor(HT_NXT_COMPASS, Sensors.compass_sensor);
  search_sensor(LEGO_EV3_TOUCH, Sensors.touch_sensor);
  return sensors;
}
