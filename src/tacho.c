#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "const.h"
#include "position.h"
#include "sensors.h"
#include "tacho.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

//#define TACHO_DEBUG

#ifdef TACHO_DEBUG
#include <pthread.h>
coordinate_t coordinate = {60, 30, 90, PTHREAD_MUTEX_INITIALIZER};
volatile int quit_request = 0;   // To stop the position thread
#endif

/* By Olivier
  Stop given tacho. */
void stop_tacho(uint8_t tacho_id) {
    set_tacho_command_inx(tacho_id, TACHO_STOP);
}

/* By Olivier.
   Wait for the given tachos to stop. */
void wait_wheels(uint8_t right_wheel, uint8_t left_wheel) {
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    do {
        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));
}

/* By Nathan.
   Wait for the ultrasonic sensor tacho to stop. */
void wait_head(uint8_t ultrasonic_tacho) {
    char head_state[TACHO_BUFFER_SIZE];
    do {
        get_tacho_state(ultrasonic_tacho, head_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", head_state));
}


/* By Olivier.
   While moving, this function checks if there is an obstacle and stop tachos
   if indeed there is one. */
void waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id) {
    int current_distance;
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    do {
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
        if (current_distance < TRESHOLD_MANEUVER) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            break;
        }
        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));
}

/* By Erwan
   Tranlate by X millimeters. */
void translation(uint8_t right_wheel, uint8_t left_wheel, int distance) {
    if (!distance) return;

    int max_speed, speed;
    int count_per_rot, rel_pos;
    /* int position_start, current_position, temp; */

    // Set behavior when tachos will stop
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    // Get the tachos current settings
    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);
    /* get_tacho_position(left_wheel, &position_start); */

    // Calculate the speed percentage and the number of rotation for the wheel
    rel_pos = round(((float)distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
    speed = round((float)max_speed * TRANSLATION_SPEED / 100.0 + 0.5);

    // Set the tachos speed to the one calculated
    set_tacho_speed_sp(left_wheel, speed);
    set_tacho_speed_sp(right_wheel, speed);

    // Set the acceleration
    set_tacho_ramp_up_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_up_sp(right_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(right_wheel, RAMP_DURATION);

    // Set the number of wheel rotation
    set_tacho_position_sp(left_wheel, rel_pos);
    set_tacho_position_sp(right_wheel, rel_pos);

    // Run the specified command
    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);

    // Update the position
    /* get_tacho_position(left_wheel, &current_position);
    while((current_position - position_start) != rel_pos) {
        temp = current_position;
        get_tacho_position(left_wheel, &current_position);
        update_coordinate(WHEEL_PERIMETER * abs(current_position - temp) / count_per_rot);
        Sleep(100);
    } */
}

/* By Erwan
   Rotate by X degres */
void rotation(uint8_t right_wheel, uint8_t left_wheel, int angle) {
    if (!angle) return;

    float rad = ((float)angle / 360.0) * 2 * M_PI;
    int max_speed, speed;
    int count_per_rot, rel_pos;

    // Set behavior when tachos will stop
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    // Get the tachos current settings
    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);

    // Calculate the speed percentage and the number of rotation for the wheel
    rel_pos = round((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
    speed = round((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);

    // Set the tachos speed to the one calculated
    set_tacho_speed_sp(left_wheel, speed);
    set_tacho_speed_sp(right_wheel, speed);

    // Set the acceleration
    set_tacho_ramp_up_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_up_sp(right_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(right_wheel, RAMP_DURATION);

    // Set the number of wheel rotation
    set_tacho_position_sp(left_wheel, -rel_pos);
    set_tacho_position_sp(right_wheel, rel_pos);

    // Run the specified command
    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);
}

// Nathan
// Make the robot turn based on angle from gyro sensor
void rotation_gyro(uint8_t right_wheel, uint8_t left_wheel, uint8_t gyro_id, int angle) {
    if (!angle) return;

    const int RANGE_ANGLE = 2;
    const int SPEED_MAX   = 40;
    const int SPEED_MIN   = 18;

    int angle_start, current_angle;

    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    //init angle start angle
    angle_start = get_angle(gyro_id);
    current_angle = angle_start;

    //duty_cycle is the roughly the percentage of power given to the tacho
    int duty_cycle = angle - (current_angle - angle_start);

    if (duty_cycle > SPEED_MAX || duty_cycle < ((-1) * SPEED_MAX)){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MAX;
    }
    if (duty_cycle > ((-1) * SPEED_MIN) && duty_cycle < SPEED_MIN){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MIN;
    }

    //set the tacho's rotation
    set_tacho_duty_cycle_sp(left_wheel, duty_cycle);
    set_tacho_duty_cycle_sp(right_wheel, (-1) * duty_cycle);

    //launch tachos
    set_tacho_command_inx(left_wheel, TACHO_RUN_DIRECT );
    set_tacho_command_inx(right_wheel, TACHO_RUN_DIRECT );

    while ((abs(abs(angle_start - current_angle) - angle)) > RANGE_ANGLE){

      //recompute duty cycle value
      duty_cycle = angle - (current_angle - angle_start);
      if (duty_cycle > SPEED_MAX || duty_cycle < ((-1) * SPEED_MAX)){
        duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MAX;
      }
      if (duty_cycle > ((-1) * SPEED_MIN) && duty_cycle < SPEED_MIN){
        duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MIN;
      }

      //update duty cycle value
      set_tacho_duty_cycle_sp(left_wheel, duty_cycle);
      set_tacho_duty_cycle_sp(right_wheel, (-1) * duty_cycle);
      Sleep(50);
      //update current angle
      current_angle = get_angle(gyro_id);
    }
    set_tacho_command_inx(left_wheel, TACHO_STOP);
    set_tacho_command_inx(right_wheel, TACHO_STOP);
}


//Nathan
//function to operate tacho

void operate_tacho(uint8_t tacho, int angle){
  if (!angle) return;

  int max_speed, speed;

  // Set behavior when tachos will stop
  set_tacho_stop_action_inx(tacho, TACHO_HOLD);

  // Get the tachos current settings
  get_tacho_max_speed(tacho, &max_speed);

  //Compute the speed of rotation
  speed = round((float)max_speed / 8 );

  // Set the tachos speed to the one calculated
  set_tacho_speed_sp(tacho, speed);

  // Set the acceleration
  set_tacho_ramp_up_sp(tacho, RAMP_DURATION);
  set_tacho_ramp_down_sp(tacho, RAMP_DURATION);

  // Set the number of wheel rotation
  set_tacho_position_sp(tacho, angle);

  // Run the specified command
  set_tacho_command_inx(tacho, TACHO_RUN_TO_REL_POS);
}

//Nathan
//function to turn the head of the robot
//NOTE : the max rotation angle that can be given to the tacho to turn the head
//is 135 or -135 when the head is at the center position

void turn_ultrasonic_tacho(uint8_t ultrasonic_tacho, int angle){

  int rel_pos;
  rel_pos = round(angle);
  if (rel_pos > THRESHOLD_ULTRASONIC_TACHO){
    rel_pos = THRESHOLD_ULTRASONIC_TACHO;
  }
  if (rel_pos < -1 * THRESHOLD_ULTRASONIC_TACHO){
    rel_pos = -1 * THRESHOLD_ULTRASONIC_TACHO;
  }

  //operate tacho with the right angle
  operate_tacho(ultrasonic_tacho, rel_pos);
}


//Nathan
//function to turn the tacho which operate the carrier

void turn_carrier_tacho(uint8_t obstacle_carrier, int angle){
  operate_tacho(obstacle_carrier, angle);
}

//Nathan
//Function to perform a single scan

int single_scan(uint8_t ultrasonic_tacho, uint8_t sonar_id, int angle){
  turn_ultrasonic_tacho(ultrasonic_tacho, angle);
  return get_avg_distance(sonar_id, NB_SENSOR_MESURE);
}

//Nathan
//Function to perform a scan of the area from the min angle to the max angle

void scan_distance(uint8_t ultrasonic_tacho, uint8_t sonar_id, int number_of_scan, int min_angle, int max_angle, int * array_of_scan_values){
  int angle_delta = round((float)(max_angle - min_angle) / (float)number_of_scan);
  turn_ultrasonic_tacho(ultrasonic_tacho, min_angle);
  // get first value of scan
  array_of_scan_values[0] = get_avg_distance(sonar_id, NB_SENSOR_MESURE);
  //for remaining scan, turn head and scan
  for(int i = 1; i < number_of_scan; i++){
    array_of_scan_values[i] = single_scan(ultrasonic_tacho, sonar_id, angle_delta);
  }
  //turn head back to center position
  turn_ultrasonic_tacho(ultrasonic_tacho, -1 * max_angle);
}


#ifdef TACHO_DEBUG

#define LEFT_WHEEL_PORT        66
#define RIGHT_WHEEL_PORT       67
#define ULTRASONIC_TACHO_PORT  68
#define CARRIER_PORT           65

/* ********************** MAIN USED FOR TESTS ********************** */
int main(int argc, char *argv[]) {
    if (argc != 6) {
        printf("Usage: ./tacho translation_distance rotation_angle ultrasonic_tacho_rotation obstacle_carrier_rotation number_of_scan\n");
        exit(-1);
    }

    uint8_t right_wheel, left_wheel, ultrasonic_tacho, obstacle_carrier;
    uint8_t sonar_id, color_id;

    int translation_dist = atoi(argv[1]);
    int rotation_angle   = atoi(argv[2]);
    int ultrasonic_tacho_rotation = atoi(argv[3]);
    int obstacle_carrier_rotation   = atoi(argv[4]);
    int number_of_scan   = atoi(argv[5]);

    ev3_sensor_init();
    ev3_tacho_init();
    ev3_search_sensor(LEGO_EV3_US, &sonar_id, 0);
    ev3_search_sensor(LEGO_EV3_COLOR, &color_id, 0);

    printf("Initializing tachos...\n");
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &right_wheel, 0)) {
        printf("    [OK] Right wheel\n");
        if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &left_wheel, 0)) {
            printf("    [OK] Left wheel\n");
            if (ev3_search_tacho_plugged_in(ULTRASONIC_TACHO_PORT, 0, &ultrasonic_tacho, 0)) {
                printf("    [OK] Ultrasonic tacho\n");
                if (ev3_search_tacho_plugged_in(CARRIER_PORT, 0, &obstacle_carrier, 0)) {
                    printf("    [OK] Carrier tacho\n");
                } else {
                  printf("    [ERR] Carrier tacho\n");
                  exit(-1);
                }
            } else {
              printf("    [ERR] Ultrasonic tacho\n");
              exit(-1);
            }
        } else {
            printf("    [ERR] Left wheel\n");
            exit(-1);
        }
    } else {
        printf("    [ERR] Right wheel\n");
        exit(-1);
    }
    printf("Done.\n");

    printf("Rotating by %d degres... ", rotation_angle);
    //rotation(right_wheel, left_wheel, rotation_angle);
    //wait_wheels(right_wheel, left_wheel);
    printf("Done.\n");

    printf("Moving forward by %d mm and detecting obstacles... ", translation_dist);
    //translation(right_wheel, left_wheel, translation_dist);
    //waitncheck_wheels(right_wheel, left_wheel, sonar_id);
    printf("Done.\n");

    printf("Moving backward by %d mm... ", translation_dist);
    //translation(right_wheel, left_wheel, -translation_dist);
    //wait_wheels(right_wheel, left_wheel);
    printf("Done.\n");

    //printf("Color detected: %d\n", get_avg_color(color_id, NB_SENSOR_MESURE));

    printf("Turning ultrasonic sensor of %d degree... ", ultrasonic_tacho_rotation);
    turn_ultrasonic_tacho(ultrasonic_tacho, ultrasonic_tacho_rotation);
    wait_head(ultrasonic_tacho);
    printf("Done.\n");

    printf("Performing %d scans... ", number_of_scan);
    int scanned_values[number_of_scan];
    scan_distance(ultrasonic_tacho, sonar_id, number_of_scan, -135, 135, scanned_values);
    for (int i = 0; i<number_of_scan; i++){
      printf("value %d scanned = %d \n", i, scanned_values[i]);
    }
    wait_head(ultrasonic_tacho);
    printf("Done.\n");

    set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
    set_tacho_stop_action_inx(left_wheel, TACHO_COAST);
    set_tacho_stop_action_inx(ultrasonic_tacho, TACHO_COAST);
    set_tacho_stop_action_inx(obstacle_carrier, TACHO_COAST);
    stop_tacho(right_wheel);
    stop_tacho(left_wheel);
    stop_tacho(ultrasonic_tacho);
    stop_tacho(obstacle_carrier);
    ev3_uninit();
}

#endif
