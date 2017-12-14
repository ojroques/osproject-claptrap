#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "position.h"
#include "tacho.h"
#include "const.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

#define TACHO_BUFFER_SIZE 256

/* By Olivier.
Wait for the tachos to stop. */
void wait_tachos() {
    char lsn_state[TACHO_BUFFER_SIZE];
    char rsn_state[TACHO_BUFFER_SIZE];
    uint8_t lsn, rsn;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            do {
                get_tacho_state(lsn, lsn_state, TACHO_BUFFER_SIZE);
                get_tacho_state(rsn, rsn_state, TACHO_BUFFER_SIZE);
                Sleep(200);
            } while (1);
        }
    }
}

//Erwan
void turn_left(float angle){
  uint8_t lsn;
  uint8_t rsn;
  float rad = angle/360 * 2*M_PI;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      int max_speed;
      int count_per_rot;
      int rel_pos;
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
      get_tacho_max_speed(lsn, &max_speed);
      get_tacho_count_per_rot(lsn, &count_per_rot);
      rel_pos = floor(ROBOT_RADIUS*rad/WHEEL_PERIMETER * count_per_rot);
      set_tacho_speed_sp( lsn, max_speed / 2 );
      set_tacho_speed_sp( rsn, max_speed / 2 );
      set_tacho_ramp_up_sp( lsn, 50 );
      set_tacho_ramp_up_sp( rsn, 50 );
  		set_tacho_ramp_down_sp( lsn, 50 );
      set_tacho_ramp_down_sp( rsn, 50 );
  		set_tacho_position_sp( lsn, -rel_pos );
      set_tacho_position_sp( rsn, rel_pos );
      set_tacho_command_inx( lsn, TACHO_RUN_TO_REL_POS );
      set_tacho_command_inx( rsn, TACHO_RUN_TO_REL_POS );
      update_theta(angle);
    }
  }

}

//Erwan
void turn_rigth(float angle){
  uint8_t lsn;
  uint8_t rsn;
  float rad = angle/360 * 2*M_PI;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      int max_speed;
      int count_per_rot;
      int rel_pos;
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
      get_tacho_max_speed(lsn, &max_speed);
      get_tacho_count_per_rot(lsn, &count_per_rot);
      rel_pos = floor(ROBOT_RADIUS*rad/WHEEL_PERIMETER * count_per_rot);
      set_tacho_speed_sp( lsn, max_speed / 2 );
      set_tacho_speed_sp( rsn, max_speed / 2 );
      set_tacho_ramp_up_sp( lsn, 50 );
      set_tacho_ramp_up_sp( rsn, 50 );
  		set_tacho_ramp_down_sp( lsn, 50 );
      set_tacho_ramp_down_sp( rsn, 50 );
  		set_tacho_position_sp( lsn, rel_pos );
      set_tacho_position_sp( rsn, -rel_pos );
      set_tacho_command_inx( lsn, TACHO_RUN_TO_REL_POS );
      set_tacho_command_inx( rsn, TACHO_RUN_TO_REL_POS );
      update_theta(-angle);
    }
  }

}

//Erwan
void forward(float distance){
  uint8_t lsn;
  uint8_t rsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      int max_speed;
      int count_per_rot;
      int rel_pos;
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
      get_tacho_max_speed(lsn, &max_speed);
      get_tacho_count_per_rot(lsn, &count_per_rot);
      rel_pos = floor(distance/WHEEL_PERIMETER * count_per_rot);
      set_tacho_speed_sp( lsn, max_speed / 2 );
      set_tacho_speed_sp( rsn, max_speed / 2 );
      set_tacho_ramp_up_sp( lsn, 50 );
      set_tacho_ramp_up_sp( rsn, 50 );
  		set_tacho_ramp_down_sp( lsn, 50 );
      set_tacho_ramp_down_sp( rsn, 50 );
  		set_tacho_position_sp( lsn, rel_pos );
      set_tacho_position_sp( rsn, rel_pos );
      set_tacho_command_inx( lsn, TACHO_RUN_TO_REL_POS );
      set_tacho_command_inx( rsn, TACHO_RUN_TO_REL_POS );
      update_coordinate(distance);
    }
  }

}

//Erwan
void backward(float distance){
  uint8_t lsn;
  uint8_t rsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      int max_speed;
      int count_per_rot;
      int rel_pos;
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
      get_tacho_max_speed(lsn, &max_speed);
      get_tacho_count_per_rot(lsn, &count_per_rot);
      rel_pos = floor(distance/WHEEL_PERIMETER * count_per_rot);
      set_tacho_speed_sp( lsn, max_speed / 2 );
      set_tacho_speed_sp( rsn, max_speed / 2 );
      set_tacho_ramp_up_sp( lsn, 50 );
      set_tacho_ramp_up_sp( rsn, 50 );
  		set_tacho_ramp_down_sp( lsn, 50 );
      set_tacho_ramp_down_sp( rsn, 50 );
  		set_tacho_position_sp( lsn, -rel_pos );
      set_tacho_position_sp( rsn, -rel_pos );
      set_tacho_command_inx( lsn, TACHO_RUN_TO_REL_POS );
      set_tacho_command_inx( rsn, TACHO_RUN_TO_REL_POS );
      update_coordinate(-distance);
    }
  }

}
