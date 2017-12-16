#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "const.h"
#include "position.h"
#include "tacho.h"
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
            } while (strcmp("holding", lsn_state) && strcmp("holding", rsn_state));
        }
    }
}

/* By Olivier.
   Wait for the tongs to stop. */
void wait_tongs() {
    char tsn_state[TACHO_BUFFER_SIZE];
    uint8_t tsn;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT, 0, &tsn, 0)) {
        do {
            get_tacho_state(tsn, tsn_state, TACHO_BUFFER_SIZE);
            Sleep(200);
        } while (strcmp("holding", tsn_state));
    }
}

//Erwan
void turn_left(float angle) {
    if (angle == 0) {
        return;
    }
    uint8_t lsn;
    uint8_t rsn;
    float rad = angle/360 * 2*M_PI;
    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
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
void turn_right(float angle){
    if (angle == 0) {
        return;
    }
    uint8_t lsn;
    uint8_t rsn;
    float rad = angle/360 * 2*M_PI;
    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
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
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
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
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
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

//Erwan
void down_tongs(){
  uint8_t dsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT,0, &dsn, 0 )){
      int max_speed;
      int rel_pos = TONGS_UP_DOWN_DISTANCE;
      set_tacho_stop_action_inx(dsn,TACHO_HOLD);
      get_tacho_max_speed(dsn, &max_speed);
      set_tacho_speed_sp( dsn, max_speed / 2 );
      set_tacho_ramp_up_sp( dsn, 25 );
      set_tacho_ramp_down_sp( dsn, 100 );
      set_tacho_position_sp( dsn, rel_pos );
      set_tacho_command_inx( dsn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
void up_tongs(){
  uint8_t usn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT,0, &usn, 0 )){
      int max_speed;
      int rel_pos = TONGS_UP_DOWN_DISTANCE;
      set_tacho_stop_action_inx(usn,TACHO_HOLD);
      get_tacho_max_speed(usn, &max_speed);
      set_tacho_speed_sp( usn, max_speed / 2 );
      set_tacho_ramp_up_sp( usn, 25 );
      set_tacho_ramp_down_sp( usn, 100 );
      set_tacho_position_sp( usn, rel_pos );
      set_tacho_command_inx( usn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
void close_tongs(){
  uint8_t csn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT,0, &csn, 0 )){
      int max_speed;
      int rel_pos = TONGS_OPEN_CLOSE_DISTANCE;
      set_tacho_stop_action_inx(csn,TACHO_HOLD);
      get_tacho_max_speed(csn, &max_speed);
      set_tacho_speed_sp( csn, max_speed / 2 );
      set_tacho_ramp_up_sp( csn, 25 );
      set_tacho_ramp_down_sp( csn, 100 );
      set_tacho_position_sp( csn, rel_pos );
      set_tacho_command_inx( csn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
void open_tongs(){
  uint8_t osn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT,0, &osn, 0 )){
      int max_speed;
      int rel_pos = TONGS_OPEN_CLOSE_DISTANCE;
      set_tacho_stop_action_inx(osn,TACHO_HOLD);
      get_tacho_max_speed(osn, &max_speed);
      set_tacho_speed_sp( osn, max_speed / 2 );
      set_tacho_ramp_up_sp( osn, 25 );
      set_tacho_ramp_down_sp( osn, 100 );
      set_tacho_position_sp( osn, rel_pos );
      set_tacho_command_inx( osn, TACHO_RUN_TO_REL_POS );
    }
}
