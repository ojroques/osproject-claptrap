#include "ev3_sensor.h"

/**
Set of function which control the sensors.
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
