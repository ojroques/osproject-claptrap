#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>

#include "const.h"
#include "position.h"
#include "sensors.h"
#include "tacho.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

#ifdef TACHO_DEBUG
coordinate_t coordinate = {60, 30, 90, PTHREAD_MUTEX_INITIALIZER};
volatile int quit_request = 0;   // To stop the position thread
#endif

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
void wait_tongs(int id) {
    char tsn_state[TACHO_BUFFER_SIZE];
    char udsn_state[TACHO_BUFFER_SIZE];
    uint8_t tsn, udsn;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (id == UP_DOWN_ID && ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT, 0, &udsn, 0)) {
        do {
            get_tacho_state(udsn, udsn_state, TACHO_BUFFER_SIZE);
            Sleep(200);
        } while (strcmp("holding", udsn_state));
    }
    else if (id == OPEN_CLOSE_ID && ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT, 0, &tsn, 0)) {
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
            int max_speed, speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
            speed = (int)((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);
            set_tacho_speed_sp( lsn, speed );
            set_tacho_speed_sp( rsn, speed );
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
            int max_speed, speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
            speed = (int)((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);
            set_tacho_speed_sp( lsn, speed );
            set_tacho_speed_sp( rsn, speed );
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
            int max_speed, speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
            speed = (int)((float)max_speed * TRANSLATION_SPEED / 100.0 + 0.5);
            set_tacho_speed_sp( lsn, speed );
            set_tacho_speed_sp( rsn, speed );
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
            int max_speed, speed;
            int count_per_rot;
            int rel_pos;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
            speed = (int)((float)max_speed * TRANSLATION_SPEED / 100.0 + 0.5);
            set_tacho_speed_sp( lsn, speed );
            set_tacho_speed_sp( rsn, speed );
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
/* By Olivier
  Stop both tachos. */
void stop_moving() {
    uint8_t lsn;
    uint8_t rsn;
    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            set_tacho_command_inx( lsn, TACHO_STOP );
            set_tacho_command_inx( rsn, TACHO_STOP );
        }
    }
}

//Erwan
// Down: negative value
void down_tongs(uint8_t sonar_id){
    int max_speed, speed;
    uint8_t dsn;
    // Check that the tongs can indeed move down
    if (get_avg_distance(sonar_id, 5) < 40) return;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT,0, &dsn, 0 )){
        set_tacho_stop_action_inx(dsn,TACHO_HOLD);
        get_tacho_max_speed(dsn, &max_speed);
        speed = (int)((float)max_speed * UP_DOWN_SPEED / 100.0 + 0.5);
        set_tacho_speed_sp( dsn, speed );
        set_tacho_ramp_up_sp( dsn, 25 );
        set_tacho_ramp_down_sp( dsn, 100 );
        set_tacho_position_sp( dsn, -TONGS_UP_DOWN_DISTANCE );
        set_tacho_command_inx( dsn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
// Up: positive value
void up_tongs(uint8_t sonar_id){
    int max_speed, speed;
    uint8_t usn;
    // Check that the tongs can indeed move up
    if (get_avg_distance(sonar_id, 5) > 40) return;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT,0, &usn, 0 )){
        set_tacho_stop_action_inx(usn,TACHO_HOLD);
        get_tacho_max_speed(usn, &max_speed);
        speed = (int)((float)max_speed * UP_DOWN_SPEED / 100.0 + 0.5);
        set_tacho_speed_sp( usn, speed );
        set_tacho_ramp_up_sp( usn, 25 );
        set_tacho_ramp_down_sp( usn, 100 );
        set_tacho_position_sp( usn, TONGS_UP_DOWN_DISTANCE );
        set_tacho_command_inx( usn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
// Close: positive value
void close_tongs(){
  uint8_t csn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT,0, &csn, 0 )){
      int max_speed, speed;
      int rel_pos = TONGS_OPEN_CLOSE_DISTANCE;
      set_tacho_stop_action_inx(csn,TACHO_HOLD);
      get_tacho_max_speed(csn, &max_speed);
      speed = (int)((float)max_speed * OPEN_CLOSE_SPEED / 100.0 + 0.5);
      set_tacho_speed_sp( csn, speed );
      set_tacho_ramp_up_sp( csn, 25 );
      set_tacho_ramp_down_sp( csn, 100 );
      set_tacho_position_sp( csn, rel_pos );
      set_tacho_command_inx( csn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
// Open: negative value
void open_tongs(){
  uint8_t osn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT,0, &osn, 0 )){
      int max_speed, speed;
      int rel_pos = -TONGS_OPEN_CLOSE_DISTANCE;
      set_tacho_stop_action_inx(osn,TACHO_HOLD);
      get_tacho_max_speed(osn, &max_speed);
      speed = (int)((float)max_speed * OPEN_CLOSE_SPEED / 100.0 + 0.5);
      set_tacho_speed_sp( osn, speed );
      set_tacho_ramp_up_sp( osn, 25 );
      set_tacho_ramp_down_sp( osn, 100 );
      set_tacho_position_sp( osn, rel_pos );
      set_tacho_command_inx( osn, TACHO_RUN_TO_REL_POS );
    }
}

//Erwan
void turn_left_gyro(float angle, uint8_t gyro_id) {
    if (angle == 0) {
        return;
    }
    int angle_start, current_angle;
    uint8_t lsn;
    uint8_t rsn;
    angle_start = get_angle(gyro_id);
    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed, speed;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            speed = (int)((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);
            set_tacho_speed_sp( lsn, speed );
            set_tacho_speed_sp( rsn, speed );
            set_tacho_ramp_up_sp( lsn, 50 );
            set_tacho_ramp_up_sp( rsn, 50 );
            set_tacho_ramp_down_sp( lsn, 50 );
            set_tacho_ramp_down_sp( rsn, 50 );
            set_tacho_polarity_inx( lsn, TACHO_INVERSED );
            set_tacho_polarity_inx( rsn, TACHO_NORMAL );
            current_angle = get_angle(gyro_id);
            set_tacho_command_inx( lsn, TACHO_RUN_FOREVER );
            set_tacho_command_inx( rsn, TACHO_RUN_FOREVER );
            while ((abs(abs(angle_start - current_angle) - angle)) < 2){
              if (abs(angle_start - current_angle) - angle < 0){
                set_tacho_polarity_inx( lsn, TACHO_INVERSED );
                set_tacho_polarity_inx( rsn, TACHO_NORMAL );
              }
              else{
                set_tacho_polarity_inx( lsn, TACHO_NORMAL );
                set_tacho_polarity_inx( rsn, TACHO_INVERSED );
              }
            }
            set_tacho_command_inx( lsn, TACHO_STOP );
            set_tacho_command_inx( rsn, TACHO_STOP );
            set_tacho_polarity_inx( lsn, TACHO_NORMAL );
            set_tacho_polarity_inx( rsn, TACHO_NORMAL );
        }
    }
}


//nathan
//Make the robot turn based on angle from gyro sensor
void turn_gyro_left(float angle, uint8_t gyro_id) {
    if (angle == 0) {
        return;
    }
    int angle_start, current_angle;
    uint8_t lsn;
    uint8_t rsn;
    while (ev3_tacho_init() < 1) Sleep(1000);

    //NOTE : don't use initialisation each time you want to use tacho !!!!
    //config it once and for all !!!
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            int max_speed, rspeed, lspeed;
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);

            //init tacho's speed
            get_tacho_max_speed(lsn, &max_speed);
            if (angle > 0) {
                rspeed = (int)((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);
            }
            else {
                rspeed = (int)((float)(-1)*max_speed * ROTATION_SPEED / 100.0 + 0.5);
            }
            lspeed = -rspeed;
            set_tacho_speed_sp( lsn, lspeed );
            set_tacho_speed_sp( rsn, rspeed );
            //init tacho's speed curve shape
            set_tacho_ramp_up_sp( lsn, 50 );
            set_tacho_ramp_up_sp( rsn, 50 );
            set_tacho_ramp_down_sp( lsn, 500 );
            set_tacho_ramp_down_sp( rsn, 500 );

            //init angle start angle
            angle_start = get_angle(gyro_id);
            current_angle = get_angle(gyro_id);
            //launch tacho
            set_tacho_command_inx( lsn, TACHO_RUN_FOREVER );
            set_tacho_command_inx( rsn, TACHO_RUN_FOREVER );
            int inverse = 0;
            while ((abs(abs(angle_start - current_angle) - abs(angle))) > 2){
              //if the robot goes beyond the the asked angle value go back
              if (abs(angle_start - current_angle) - abs(angle) > 0 && inverse == 0){
                rspeed = -rspeed/2;
                lspeed = -lspeed/2;
                inverse = 1;
                set_tacho_speed_sp( lsn, lspeed );
                set_tacho_speed_sp( rsn, rspeed );
                set_tacho_command_inx( lsn, TACHO_RUN_FOREVER );
                set_tacho_command_inx( rsn, TACHO_RUN_FOREVER );
              }
              else if (abs(angle_start - current_angle) - abs(angle) < 0 && inverse == 1){
                rspeed = -rspeed/2;
                lspeed = -lspeed/2;
                inverse = 0;
                set_tacho_speed_sp( lsn, lspeed );
                set_tacho_speed_sp( rsn, rspeed );
                set_tacho_command_inx( lsn, TACHO_RUN_FOREVER );
                set_tacho_command_inx( rsn, TACHO_RUN_FOREVER );
              }
              //update current angle
              Sleep(100);
              current_angle = get_angle(gyro_id);
            }
            set_tacho_command_inx( lsn, TACHO_STOP );
            set_tacho_command_inx( rsn, TACHO_STOP );
        }
    }
}


#ifdef TACHO_DEBUG

/* ********************** MAIN USED FOR TESTS ********************** */
int main(int argc, char *argv[]) {
    uint8_t udsn;
    uint8_t ocsn;
    uint8_t sonar_id, gyro_id;
    int max_speed, speed, rel_pos, distance;

    if (argc != 3) {
        printf("Usage: ./tacho ud_distance oc_distance\n");
        exit(-1);
    }

    int ud_distance = atoi(argv[1]);
    int oc_distance = atoi(argv[2]);
    ev3_sensor_init();
    ev3_search_sensor(LEGO_EV3_GYRO, &gyro_id, 0);
    ev3_search_sensor(LEGO_EV3_US, &sonar_id, 0);

    printf("Up / Down distance: %d\n", ud_distance);
    printf("Open / Close distance: %d\n", oc_distance);
    if (ud_distance > 200 || oc_distance > 200) {
        printf("One of these values seems a little high, continue anyway ? (CTRL + C to quit) ");
        getchar();
    }

    printf("Initializing tachos...\n");
    for (int i = 0; i < 5 && ev3_tacho_init() < 1; i++) Sleep(1000);
    if (ev3_search_tacho_plugged_in(UP_DOWN_TONG_PORT, 0, &udsn, 0)) {
        printf("    Up / Down tacho OK\n");
        if (ev3_search_tacho_plugged_in(OPEN_CLOSE_TONG_PORT, 0, &ocsn, 0)) {
            printf("    Open / Close tacho OK\n");
            set_tacho_stop_action_inx(udsn, TACHO_HOLD);
            set_tacho_stop_action_inx(ocsn, TACHO_HOLD);
            printf("Done.\n");
        } else {
            printf("Error.\n");
            exit(-1);
        }
    } else {
        printf("Error.\n");
        exit(-1);
    }

    //UP TONGS
    printf("Up / Down tongs... ");
    rel_pos = ud_distance;
    distance = get_avg_distance(sonar_id, 5);
    if (distance < 40 && ud_distance < 0) {
        printf("Error!\n");
        exit(EXIT_FAILURE);
    }

    if (distance > 40 && ud_distance > 0) {
        printf("Error!\n");
        exit(EXIT_FAILURE);
    }
    get_tacho_max_speed(udsn, &max_speed);
    speed = (int)((float)max_speed * UP_DOWN_SPEED / 100.0 + 0.5);
    set_tacho_speed_sp( udsn, speed );
    set_tacho_ramp_up_sp( udsn, 25 );
    set_tacho_ramp_down_sp( udsn, 100 );
    set_tacho_position_sp( udsn, rel_pos );
    set_tacho_command_inx( udsn, TACHO_RUN_TO_REL_POS );
    printf("Done.\n");

    // OPEN TONGS
    printf("Opening / Closing tongs... ");
    rel_pos = oc_distance;
    get_tacho_max_speed(ocsn, &max_speed);
    speed = (int)((float)max_speed * OPEN_CLOSE_SPEED / 100.0 + 0.5);
    set_tacho_speed_sp( ocsn, speed );
    set_tacho_ramp_up_sp( ocsn, 25 );
    set_tacho_ramp_down_sp( ocsn, 100 );
    set_tacho_position_sp( ocsn, rel_pos );
    set_tacho_command_inx( ocsn, TACHO_RUN_TO_REL_POS );
    printf("Done.\n");

    Sleep(500);
    printf("Angle before: %d\n", get_angle(gyro_id));
    turn_gyro_left(90.0, gyro_id);
    wait_tachos();
    Sleep(500);
    printf("Angle after: %d\n", get_angle(gyro_id));

    ev3_uninit();
}

#endif
