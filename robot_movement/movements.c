#include <stdlib.h>
#include <math.h>
#include "movements.h"
#include "configuration.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"



void turn_left(float angle){
  uint8_t lsn;
  uint8_t rsn;
  float rad = angle/360 * 2*M_PI;
  while (ev3_tacho_init() < 1) Sleep(1);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      int max_speed;
      int count_per_rot;
      int rel_pos;
      get_tacho_max_speed(lsn, &max_speed);
      get_tacho_count_per_rot(lsn, &count_per_rot);
      rel_pos = floor(ROBOT_RADIUS*rad/WHEEL_PERIMETER * count_per_rot);
      set_tacho_speed_sp( lsn, max_speed / 2 );
      set_tacho_speed_sp( rsn, max_speed / 2 );
      set_tacho_ramp_up_sp( lsn, 0 );
      set_tacho_ramp_up_sp( rsn, 0 );
  		set_tacho_ramp_down_sp( lsn, 0 );
      set_tacho_ramp_down_sp( rsn, 0 );
  		set_tacho_position_sp( lsn, -rel_pos );
      set_tacho_position_sp( rsn, rel_pos );
      set_tacho_command_inx( lsn, TACHO_RUN_TO_REL_POS );
      set_tacho_command_inx( rsn, TACHO_RUN_TO_REL_POS );
    }
  }

}
