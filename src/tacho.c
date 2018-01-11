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

#ifdef TACHO_DEBUG
#include <pthread.h>
coordinate_t coordinate = {600, 300, 90, PTHREAD_MUTEX_INITIALIZER};
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
   Wait for the tacho to stop. */
void wait_tacho(uint8_t tacho) {
    char tacho_state[TACHO_BUFFER_SIZE];
    do {
        get_tacho_state(tacho, tacho_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", tacho_state));
}

/* By Olivier.
   While moving, this function checks if there is an obstacle and stop tachos
   if indeed there is one.
   Return 0 if tachos stopped properly
          1 if an obstacle has been detected. */
int waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id) {
    int current_distance;
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    do {
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
        if (current_distance < TRESHOLD_MANEUVER) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            return 1;
        }
        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));
    return 0;
}
















/* By Nathan
   Tranlate by X millimeters.
   And do the checking for obstacle and update coordinates
    */
int translation_light(uint8_t right_wheel, uint8_t left_wheel, int distance, uint8_t ultrasonic_tacho, uint8_t ultrasonic_id) {
    if (!distance) return 0;

    int max_speed, speed;
    int count_per_rot, rel_pos, position_start_left, position_start_right, position_start;

    // Set behavior when tachos will stop
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    // Get the tachos current settings
    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);
    get_tacho_position(left_wheel, &position_start_left);
    get_tacho_position(right_wheel, &position_start_right);
    position_start = round((position_start_left + position_start_right)/2);

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

    //DURING NAVIGATION
    //previously in waitncheck_wheels

    int current_distance, previous_distance, previous_traveled_distance;
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    previous_traveled_distance = 0;

    current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);


    // Run the specified command
    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);
    int current_position_right;
    int current_position_left;
    int current_position, delta_position, traveled_distance, delta_traveled_distance, delta_distance;

    do {

        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);
        
        previous_distance = current_distance;
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);


        get_tacho_position(right_wheel, &current_position_right);
        get_tacho_position(left_wheel, &current_position_left);

        //get the average postion of the two tachos
        current_position = round((current_position_left + current_position_right) / 2) ;

        //compare it to the position rel_pos
        delta_position = current_position - position_start;

        //compute traveled distance by doing the inverse computation from rel_pos
        traveled_distance = round((((float)delta_position - 0.5) / count_per_rot ) * WHEEL_PERIMETER);
        delta_traveled_distance = traveled_distance - previous_traveled_distance;

        //Compute the traveled distance from the distance sensor
        delta_distance = (previous_distance - current_distance);

        //compare it to the delta of distance from the sensor (fisrt value against current value)
        //chose the best of the two
        if (delta_distance < delta_traveled_distance + NAV_DIST_RANGE && delta_distance > delta_traveled_distance - NAV_DIST_RANGE){
          delta_traveled_distance = delta_distance;
          }


        if (current_distance < TRESHOLD_MANEUVER && distance > 0) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            //update the distance with it
            update_coordinate(delta_traveled_distance);
            return 1;
        }
        //update the distance with it
        update_coordinate(delta_traveled_distance);
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));
    return 0;
}

















/* By Nathan
   Tranlate by X millimeters.
   And do the checking for obstacle and update coordinatesprevious_distance = current_distance;
    */

/*
int translation(uint8_t right_wheel, uint8_t left_wheel, int distance, uint8_t ultrasonic_tacho, uint8_t ultrasonic_id) {
    if (!distance) return 0;

    int max_speed, speed;
    int count_per_rot, rel_pos, position_start_left, position_start_right, position_start;

    // Set behavior when tachos will stop
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    // Get the tachos current settings
    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);
    get_tacho_position(left_wheel, &position_start_left);
    get_tacho_position(right_wheel, &position_start_right);
    position_start = round((position_start_left + position_start_right)/2);

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

    //DURING NAVIGATION
    //previously in waitncheck_wheels

    int current_distance, previous_distance, previous_traveled_distance;
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    int step = 0;
    previous_traveled_distance = 0;

    current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
    previous_distance = current_distance;

    // Run the specified command
    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);

    do {
        if (step == 1 || step == 3){
          previous_distance = current_distance;
        }
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
        if ((step == 0 || step == 2 ) && current_distance < TRESHOLD_MANEUVER && distance > 0) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            //update position TODO
            //update the coordinates values
            update_distance(right_wheel, left_wheel, ultrasonic_id, position_start, count_per_rot, previous_traveled_distance, previous_distance);
            return 1;
        }
        if((step == 1 || step == 3 ) && current_distance < TRESHOLD_MANEUVER_SIDE && distance > 0){
          stop_tacho(right_wheel);
          stop_tacho(left_wheel);
          //Put back the head at the center position
          if (step == 1){
            turn_ultrasonic_tacho(ultrasonic_tacho, -NAV_ANGLE_RANGE);
            wait_tacho(ultrasonic_tacho);
          }else{
            turn_ultrasonic_tacho(ultrasonic_tacho, NAV_ANGLE_RANGE);
            wait_tacho(ultrasonic_tacho);
          }
          current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
          //update position TODO
          //update the coordinates values
          update_distance(right_wheel, left_wheel, ultrasonic_id, position_start, count_per_rot, previous_traveled_distance, previous_distance);
          return 1;
        }


        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);

        //INSERT the code for robust position
        //NOTE mais need to update at reduce frequency ?
        //step 0 = start center
        //step 1 = left
        //step 2 = center
        //step 3 = right
        //loop

        if (step == 0 || step == 2){
          //update the coordinates values
          previous_traveled_distance = update_distance(right_wheel, left_wheel, ultrasonic_id, position_start, count_per_rot, previous_traveled_distance, previous_distance);

          if (step == 0){
            turn_ultrasonic_tacho(ultrasonic_tacho, NAV_ANGLE_RANGE);
          }
          if(step == 2){
            turn_ultrasonic_tacho(ultrasonic_tacho, -NAV_ANGLE_RANGE);
          }
          step += 1;
        }else{
          if (step == 1){
            step += 1;
            turn_ultrasonic_tacho(ultrasonic_tacho, -NAV_ANGLE_RANGE);
          }
          if (step == 3){
            step = 0;
            turn_ultrasonic_tacho(ultrasonic_tacho, NAV_ANGLE_RANGE);
          }
        }
        Sleep(400);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));

    //put back the head in place
    if (step == 1){
      turn_ultrasonic_tacho(ultrasonic_tacho, -NAV_ANGLE_RANGE);
      wait_tacho(ultrasonic_tacho);
    }
    if (step == 3 ){
      turn_ultrasonic_tacho(ultrasonic_tacho, NAV_ANGLE_RANGE);
      wait_tacho(ultrasonic_tacho);
    }

    //update the coordinates values
    update_distance(right_wheel, left_wheel, ultrasonic_id, position_start, count_per_rot, previous_traveled_distance, previous_distance);
    return 0;
}
*/



//Nathan
//Function called to update the coordinates

/*
int update_distance(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id, int position_start, int count_per_rot, int previous_traveled_distance, int previous_distance){
  //get the position of the tachos
  int current_position_right;
  int current_position_left;
  get_tacho_position(right_wheel, &current_position_right);
  get_tacho_position(left_wheel, &current_position_left);

  //get the average postion of the two tachos
  int current_position = round((current_position_left + current_position_right) / 2) ;
  int current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);

  //compare it to the position rel_pos
  int delta_position = current_position - position_start;

  //compute traveled distance by doing the inverse computation from rel_pos
  int traveled_distance = round((((float)delta_position - 0.5) / count_per_rot ) * WHEEL_PERIMETER);
  int delta_traveled_distance = traveled_distance - previous_traveled_distance;

  //Compute the traveled distance from the distance sensor
  int delta_distance = (previous_distance - current_distance);

  //compare it to the delta of distance from the sensor (fisrt value against current value)
  //chose the best of the two
  if (delta_distance < delta_traveled_distance + NAV_DIST_RANGE && delta_distance > delta_traveled_distance - NAV_DIST_RANGE){
    delta_traveled_distance = delta_distance;
  }

  //update the distance with it
  update_coordinate(delta_traveled_distance);

  //return the new value of previous_traveled_distance
  return current_distance;
}
*/


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
    int duty_cycle;
    if(angle > 0 ){
      //for positive value of angle
      duty_cycle = angle - abs(current_angle - angle_start);
    }else{
      //for negative value of angle
      duty_cycle = angle + abs(current_angle - angle_start);
    }

    if (duty_cycle > SPEED_MAX || duty_cycle < ((-1) * SPEED_MAX)){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MAX;
    }
    if (duty_cycle > ((-1) * SPEED_MIN) && duty_cycle < SPEED_MIN){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MIN;
    }

    //set the tacho's rotation1
    set_tacho_duty_cycle_sp(left_wheel, duty_cycle);
    set_tacho_duty_cycle_sp(right_wheel, (-1) * duty_cycle);

    //launch tachos
    set_tacho_command_inx(left_wheel, TACHO_RUN_DIRECT );
    set_tacho_command_inx(right_wheel, TACHO_RUN_DIRECT );

    while ((abs(abs(angle_start - current_angle) - angle)) > RANGE_ANGLE){

      //recompute duty cycle value
      if(angle > 0 ){
        //for positive value of angle
        duty_cycle = angle - abs(current_angle - angle_start);
      }else{
        //for negative value of angle
        duty_cycle = angle + abs(current_angle - angle_start);
      }

      if (duty_cycle == 0){
        break;
      }

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
    if (angle < 0){
      update_theta(-abs(angle_start - current_angle));
    }
    else{
      update_theta(abs(angle_start - current_angle));
    }
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
  speed = round((float)max_speed / 6 );

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
//The value may be a bit different due to non symetrical behavior of tacho

void turn_ultrasonic_tacho(uint8_t ultrasonic_tacho, int angle){

  int rel_pos;
  rel_pos = round(angle);
  if (rel_pos > THRESHOLD_ULTRASONIC_TACHO_SUP){
    rel_pos = THRESHOLD_ULTRASONIC_TACHO_SUP;
  }
  if (rel_pos < THRESHOLD_ULTRASONIC_TACHO_INF){
    rel_pos = THRESHOLD_ULTRASONIC_TACHO_INF;
  }

  //operate tacho with the right angle
  operate_tacho(ultrasonic_tacho, rel_pos);
}


//Nathan
//NOTE : the carrier tacho when is up needs -60 degree to get in position
//in order to carry object
//the carrier needs -75 more degrees to be in down position
//the absolute position is set at the tacho initialisation
//(can be reset by unplug replug it)
//moving the tacho by hand doesn't change the absolute position stored in memory
//it is safer to operate the tacho with relative position value

//The following behavior order must be respected:
//up (starting position) -> middle -> down -> up

//Nathan
//Function to get the carrier in middle position
//NOTE : the carrier should ALWAYS start from up position
void carrier_middle_position(uint8_t obstacle_carrier){
  operate_tacho(obstacle_carrier, -60);
}

//Nathan
//function to release object
void carrier_down_position(uint8_t obstacle_carrier){
  operate_tacho(obstacle_carrier, -75);
}

//Nathan
//function to get the carrier back up
void carrier_up_position(uint8_t obstacle_carrier){
  operate_tacho(obstacle_carrier, 135);
}

//Nathan
//Function to perform a single scan

int single_scan(uint8_t ultrasonic_tacho, uint8_t sonar_id, int angle){
  turn_ultrasonic_tacho(ultrasonic_tacho, angle);
  return get_avg_distance(sonar_id, NB_SENSOR_MESURE);
}

//Nathan
//Function to perform a scan of the area from the min angle to the max angle
//need the min max angle to define range of scan and number of scan that is non zero
//and the array with the right length

void scan_distance(uint8_t ultrasonic_tacho, uint8_t sonar_id, int number_of_scan, int min_angle, int max_angle, int * array_of_scan_values){
  if (!number_of_scan) return;
  int angle_delta = round( (max_angle - min_angle) / (number_of_scan - 1) );
  turn_ultrasonic_tacho(ultrasonic_tacho, min_angle);
  wait_tacho(ultrasonic_tacho);
  // get first value of scan
  array_of_scan_values[0] = get_avg_distance(sonar_id, NB_SENSOR_MESURE);
  //for remaining scan, turn head and scan
  for(int i = 1; i < number_of_scan; i++){
    array_of_scan_values[i] = single_scan(ultrasonic_tacho, sonar_id, angle_delta);
    wait_tacho(ultrasonic_tacho);
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
        printf("Usage: ./tacho <translation_distance> <rotation_angle> <ultrasonic_tacho_rotation> <obstacle_carrier_rotation> <number_of_scan>\n");
        exit(EXIT_SUCCESS);
    }

    uint8_t right_wheel, left_wheel, ultrasonic_tacho, obstacle_carrier;
    uint8_t sonar_id, color_id, gyro_id, compass_id;

    int translation_dist = atoi(argv[1]);
    int rotation_angle = atoi(argv[2]);
    int ultrasonic_tacho_rotation = atoi(argv[3]);
    int obstacle_carrier_rotation = atoi(argv[4]);
    int number_of_scan = atoi(argv[5]);

    ev3_sensor_init();
    ev3_tacho_init();
    ev3_search_sensor(LEGO_EV3_US, &sonar_id, 0);
    ev3_search_sensor(LEGO_EV3_COLOR, &color_id, 0);
    ev3_search_sensor(LEGO_EV3_GYRO, &gyro_id, 0);
    ev3_search_sensor(HT_NXT_COMPASS, &compass_id, 0);
    Sleep(4000);
    int compass_starting_angle = get_compass_direction(compass_id);
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
    //rotation_gyro(right_wheel, left_wheel, gyro_id, rotation_angle);
    //wait_wheels(right_wheel, left_wheel);
    printf("Done.\n");

    printf("Moving forward by %d mm and detecting obstacles... \n", translation_dist);
    int return_value = translation_light(right_wheel, left_wheel, translation_dist, ultrasonic_tacho, sonar_id);
    //waitncheck_wheels(right_wheel, left_wheel, sonar_id);
    printf("Done.\n");

    printf("Moving backward by %d mm... \n", translation_dist);
    //translation(right_wheel, left_wheel, -translation_dist, sonar_id);
    //waitncheck_wheels(right_wheel, left_wheel, sonar_id);
    printf("Done.\n");

    printf("Rotating then using compass to recalibrate\n");
    //for(int i = 0; i < 1; i++){
    //  rotation (right_wheel, left_wheel, 90);
    //  wait_wheels(right_wheel, left_wheel);
    //}
    //recalibrate_theta(compass_id, compass_starting_angle);

    printf("Color detected: %d\n", get_avg_color(color_id, NB_SENSOR_MESURE));
    printf("Turning ultrasonic sensor of %d degree... ", ultrasonic_tacho_rotation);
    //turn_ultrasonic_tacho(ultrasonic_tacho, ultrasonic_tacho_rotation);
    //wait_tacho(ultrasonic_tacho);
    printf("Done.\n");

    printf("Turning carrier tacho of %d degree... ", obstacle_carrier_rotation);
    //carrier_middle_position(obstacle_carrier);
    //wait_tacho(obstacle_carrier);
    //carrier_down_position(obstacle_carrier);
    //wait_tacho(obstacle_carrier);
    //carrier_up_position(obstacle_carrier);
    //wait_tacho(obstacle_carrier);
    printf("Done.\n");

    printf("Performing %d scans... ", number_of_scan);
    //int scanned_values[number_of_scan];
    //scan_distance(ultrasonic_tacho, sonar_id, number_of_scan, -135, 135, scanned_values);
    //for (int i = 0; i < number_of_scan; i++){
    //    printf("value %d scanned = %d \n", i, scanned_values[i]);
    //}
    //wait_tacho(ultrasonic_tacho);
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
