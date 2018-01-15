#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>


#include "image.h"
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
coordinate_t coordinate = {600, 300, 0, PTHREAD_MUTEX_INITIALIZER};
volatile int quit_request = 0;   // To stop the position thread
#endif

/* By Olivier
  Stop given tacho. */
void stop_tacho(uint8_t tacho_id) {
    set_tacho_command_inx(tacho_id, TACHO_STOP);
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

//#########################################SECTION OF TACHO OPERATORS TO TURN AND TRANSLATE#########################################

/* By Nathan and Erwan
   Tranlate by X millimeters.
   And do the checking for obstacle and update coordinates and angle while moving
    */
int translation_light(uint8_t right_wheel, uint8_t left_wheel, int distance, uint8_t ultrasonic_id, uint8_t gyro_id, int threshold){
    if (!distance) return 0;

    int max_speed, speed;
    int count_per_rot, rel_pos;
    float old_x, old_y, current_x, current_y;
    old_x = get_coordinate_x();
    old_y = get_coordinate_y();

    // Set behavior when tachos will stop
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    // Get the tachos current settings
    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);

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

    //#####first Initialize navigation variables

    //the variables to store the tacho state
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];


    //################distance (values of the ultrasonic sensor) variables##########################
    //current distance store distance to the obstacle returned by ultrasonic sensor
    //previous distance stores the previous value;
    //delta_distance stores the distanced traveled by the robot (diff of previous and current dist)
    int current_distance, previous_distance, delta_distance;
    current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);

    //#################position of the tachos variables############################################
    //position of the right tacho and left tacho
    //current position of the mean of the positions of the tachos,
    //previous idem position and delta between the two
    int current_position_right, current_position_left;
    float current_position, previous_position;
    get_tacho_position(right_wheel, &current_position_right);
    get_tacho_position(left_wheel, &current_position_left);
    current_position = (float)(current_position_left + current_position_right) / 2;
    //The variable storing the distance traveled since the previous loop
    //is computed from the most precise value between the tacho position_start
    //and the distance sensor
    int traveled_distance;

    //#################angle of the robot#########################################################
    int current_angle, previous_angle, delta_angle;
    //init angle start angle
    current_angle = get_angle(gyro_id);

    // Run the specified command
    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);

    do {
        //######Compute distances with the ultrasonic sensor#################
        //save previous value of the ultrasonic sensor
        previous_distance = current_distance;
        //get a new value of the ultrasonic sensor
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
        //Compute the traveled distance from the last last loop up until now
        //using the distance sensor
        delta_distance = (previous_distance - current_distance);

        //######Compute positions with the tachos values#################
        //store the previous position
        previous_position = current_position;
        //update position value of the tachos
        get_tacho_position(right_wheel, &current_position_right);
        get_tacho_position(left_wheel, &current_position_left);
        //update the current position = the average postion of the two tachos
        current_position = (float)(current_position_left + current_position_right) / 2;
        //compute traveled distance by doing the inverse computation with
        //the diff of the two positions.
        traveled_distance = round((((current_position - previous_position) - 0.5) / count_per_rot ) * WHEEL_PERIMETER);

        //compare it to the delta of distance from the sensor (fisrt value against current value)
        //chose the best of the two
        if (delta_distance < traveled_distance + NAV_DIST_RANGE && delta_distance > traveled_distance - NAV_DIST_RANGE){
          traveled_distance = delta_distance;
          }

        //####################Get the angle and update it###############################
        //Get the current angle
        previous_angle = current_angle;
        current_angle = get_angle(gyro_id);
        delta_angle = current_angle - previous_angle;


        //Checking for obstacle in front of the robot.
        if (current_distance < threshold && distance > 0) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            //update the distance with it
            update_coordinate(traveled_distance);
            update_theta(delta_angle);
            float new_x = get_coordinate_x();
            float new_y = get_coordinate_y();
            explored_line(old_x, new_x, old_y, new_y);
            return 1;
        }

        //update the distance with it
        update_coordinate(traveled_distance);
        update_theta(delta_angle);

        current_x = get_coordinate_x();
        current_y = get_coordinate_y();

        //check if any of them is negative
        if(current_x < 0 || current_y < 0){
          stop_tacho(right_wheel);
          stop_tacho(left_wheel);
          float new_x = get_coordinate_x();
          float new_y = get_coordinate_y();
          explored_line(old_x, new_x, old_y, new_y);
          printf("stop because of negative x or y : x = %f y = %f", current_x, current_y);
          return 2;
        }

        //update the tacho state values
        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);

        //sleep
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));

    //update one last time position of the robot and angle one last time
    previous_distance = current_distance;
    current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
    delta_distance = (previous_distance - current_distance);
    previous_position = current_position;
    get_tacho_position(right_wheel, &current_position_right);
    get_tacho_position(left_wheel, &current_position_left);
    current_position = (float)(current_position_left + current_position_right) / 2;
    traveled_distance = round((((current_position - previous_position) - 0.5) / count_per_rot ) * WHEEL_PERIMETER);
    if (delta_distance < traveled_distance + NAV_DIST_RANGE && delta_distance > traveled_distance - NAV_DIST_RANGE){
      traveled_distance = delta_distance;
      }
    update_coordinate(traveled_distance);
    previous_angle = current_angle;
    current_angle = get_angle(gyro_id);
    delta_angle = current_angle - previous_angle;
    update_theta(delta_angle);

    //add explored path to map matrix
    float new_x = get_coordinate_x();
    float new_y = get_coordinate_y();
    explored_line(old_x, new_x, old_y, new_y);
    return 0;
}


// Nathan
// Make the robot turn based on angle from gyro sensor
//and update the abgle at the end of rotation
void rotation_gyro(uint8_t right_wheel, uint8_t left_wheel, uint8_t gyro_id, int angle) {
    if (!angle) return;

    //Constant for the rotation
    const int RANGE_ANGLE = 2;
    const int SPEED_MAX   = 40;
    const int SPEED_MIN   = 18;

    //angle_start stores the angle at the beginning
    //the current angle is stored in current_angle
    //for when the robot is stuck
    int angle_start, current_angle;
    angle_start = get_angle(gyro_id);
    current_angle = angle_start;

    //set the tachos to hold when stopped
    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    //duty_cycle is the roughly the percentage of power given to the tacho
    int duty_cycle;

    //set duty cycle according to the the turning direction of the robot
    if(angle > 0 ){
      //for positive value of angle
      duty_cycle = angle - abs(current_angle - angle_start);
    }else{
      //for negative value of angle
      duty_cycle = angle + abs(current_angle - angle_start);
    }

    //Cut values of duty cycle too big or too low
    if (duty_cycle > SPEED_MAX || duty_cycle < ((-1) * SPEED_MAX)){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MAX;
    }
    if (duty_cycle > ((-1) * SPEED_MIN) && duty_cycle < SPEED_MIN){
      duty_cycle = duty_cycle / abs(duty_cycle) * SPEED_MIN;
    }

    //set the tacho's rotation
    set_tacho_duty_cycle_sp(left_wheel, duty_cycle);
    set_tacho_duty_cycle_sp(right_wheel, (-1) * duty_cycle);

    //to prevent being stuck : watch time spent truning
    time_t start_time = time(NULL);

    //launch tachos
    set_tacho_command_inx(left_wheel, TACHO_RUN_DIRECT );
    set_tacho_command_inx(right_wheel, TACHO_RUN_DIRECT );

    //###############LOOP FOR TURNING####################################

    while ((abs(abs(angle_start - current_angle) - angle)) > RANGE_ANGLE){

      //recompute duty cycle value
      if(angle > 0 ){
        //for positive value of angle
        duty_cycle = angle - abs(current_angle - angle_start);
      }else{
        //for negative value of angle
        duty_cycle = angle + abs(current_angle - angle_start);
      }

      //if no more power were to be sent to the tachos
      if (duty_cycle == 0){
        break;
      }

      //cut the too big or too low values of duty cycle.
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

      //try to prevent the robot being stuck while turning
      if (difftime(time(NULL), start_time) > TURNING_TIME) {
          set_tacho_command_inx(left_wheel, TACHO_STOP);
          set_tacho_command_inx(right_wheel, TACHO_STOP);
          recalibrate_gyro(gyro_id);
          return;
      }

      //if the gyro gets crazy, or an angle bigger than 360 was sent
      //the turning maneuver is stopped
      current_angle = get_angle(gyro_id);
      if (abs(current_angle - angle_start) > 360){
        set_tacho_command_inx(left_wheel, TACHO_STOP);
        set_tacho_command_inx(right_wheel, TACHO_STOP);
        recalibrate_gyro(gyro_id);
        return;
      }
    //################## END OF LOOP ####################################
  }

  set_tacho_command_inx(left_wheel, TACHO_STOP);
  set_tacho_command_inx(right_wheel, TACHO_STOP);

  //update the value of the angle at the end of the turn
  if (angle < 0){
    update_theta(-abs(angle_start - current_angle));
  }else{
    update_theta(abs(angle_start - current_angle));
  }
}

//#######################END OF SECTION FOR TRANSLATION AND ROTATION############################################

//##################TACHO OPERATING US SENSOR AND CARRIER SECTION ###############################################


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
  speed = round((float)max_speed / 4 );

  // Set the tachos speed to the one calculated
  set_tacho_speed_sp(tacho, speed);

  // Set the acceleration
  set_tacho_ramp_up_sp(tacho, RAMP_DURATION_TACHO);
  set_tacho_ramp_down_sp(tacho, RAMP_DURATION_TACHO);

  // Set the number of wheel rotation
  set_tacho_position_sp(tacho, angle);

  // Run the specified command
  set_tacho_command_inx(tacho, TACHO_RUN_TO_REL_POS);
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
//NOTE : the max rotation angle that can be given to the tacho to turn the head
//is 100 or -100 when the head is at the center position
//The value may be a bit different due to non symetrical behavior of tacho
//NOTE : the real value of the head angle is the angle tacho value / 2


//Nathan
//Function to perform a single scan
int single_scan(uint8_t ultrasonic_tacho, uint8_t sonar_id, int angle){
  operate_tacho(ultrasonic_tacho, angle);
  wait_tacho(ultrasonic_tacho);
  return get_avg_distance(sonar_id, NB_SENSOR_MESURE);
}

//Nathan
//Function to perform a scan of the area from the min angle to the max angle
//need the min max angle to define range of scan and number of scan that is non zero
//and the array with the right length
void scan_distance(uint8_t ultrasonic_tacho, uint8_t sonar_id, int number_of_scan, int min_angle, int max_angle, int * array_of_scan_values){
  if (!number_of_scan) return;
  int angle_delta = round( (max_angle - min_angle) / (number_of_scan - 1) );
  operate_tacho(ultrasonic_tacho, min_angle);
  wait_tacho(ultrasonic_tacho);
  // get first value of scan
  array_of_scan_values[0] = get_avg_distance(sonar_id, NB_SENSOR_MESURE);
  //for remaining scan, turn head and scan
  for(int i = 1; i < number_of_scan; i++){
    array_of_scan_values[i] = single_scan(ultrasonic_tacho, sonar_id, angle_delta);
  }
  //turn head back to center position
  operate_tacho(ultrasonic_tacho, -1 * max_angle);
  wait_tacho(ultrasonic_tacho);
}

//################################END OF TACHO FOR US SENSOR AND CARRIER SECTION######################################

#ifdef TACHO_DEBUG

#define LEFT_WHEEL_PORT        66
#define RIGHT_WHEEL_PORT       67
#define ULTRASONIC_TACHO_PORT  68
#define CARRIER_PORT           65

/* ********************** MAIN USED FOR TESTS ********************** */
int main(int argc, char *argv[]) {
    if (argc != 7) {
        printf("Usage: ./tacho <translation_distance> <rotation_angle> <ultrasonic_tacho_rotation> <obstacle_carrier_rotation> <number_of_scan> <angle_scan>\n");
        exit(EXIT_SUCCESS);
    }

    uint8_t right_wheel, left_wheel, ultrasonic_tacho, obstacle_carrier;
    uint8_t sonar_id, color_id, gyro_id, compass_id;

    int translation_dist = atoi(argv[1]);
    int rotation_angle = atoi(argv[2]);
    int ultrasonic_tacho_rotation = atoi(argv[3]);
    int obstacle_carrier_rotation = atoi(argv[4]);
    int number_of_scan = atoi(argv[5]);
    int angle_scan = atoi(argv[6]);

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
    //int return_value = translation_light(right_wheel, left_wheel, translation_dist, ultrasonic_tacho, sonar_id);
    //waitncheck_wheels(right_wheel, left_wheel, sonar_id);
    printf("Done.\n");

    printf("Moving backward by %d mm... \n", translation_dist);
    //translation_light(right_wheel, left_wheel, translation_dist, sonar_id);
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
    //operate_tacho(ultrasonic_tacho, ultrasonic_tacho_rotation);
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
    //scan_distance(ultrasonic_tacho, sonar_id, number_of_scan, (-1)*angle_scan, angle_scan, scanned_values);
    //for (int i = 0; i < number_of_scan; i++){
    //    printf("value %d scanned = %d \n", i, scanned_values[i]);
    //}
    printf("Done.\n");

    printf("Recalibration of gyro ...\n");
    //recalibrate_gyro(gyro_id);
    printf("Done\n");

    printf("Test axe ...\n");
    init_image(24,40);
    float current_x = get_coordinate_x();
    float current_y = get_coordinate_y();
    const int DIR_NB_SCAN = 5;
    const int DIR_ANG_MIN = -60;
    const int DIR_ANG_MAX = 60;
    int scans[DIR_NB_SCAN];    // Hold the mesures from the scan
    int value, i, angle_i, pas;
    int16_t x_dest, y_dest;
    pas = (DIR_ANG_MAX - DIR_ANG_MIN) / (DIR_NB_SCAN - 1);

    scan_distance(ultrasonic_tacho, sonar_id, DIR_NB_SCAN, DIR_ANG_MIN, DIR_ANG_MAX, scans);

    for(i = 0; i < DIR_NB_SCAN; i++) {
        angle_i = (DIR_ANG_MIN + i * pas) / 2;
        get_obst_position(scans[i], angle_i, &x_dest, &y_dest);
        explored_line((int16_t)current_x, x_dest, (int16_t)current_y, y_dest);
        place_obstacle(x_dest, y_dest);
      }
    print_image();
    rotation_gyro(right_wheel, left_wheel, gyro_id, 90);
    scan_distance(ultrasonic_tacho, sonar_id, DIR_NB_SCAN, DIR_ANG_MIN, DIR_ANG_MAX, scans);

    for(i = 0; i < DIR_NB_SCAN; i++) {
        angle_i = (DIR_ANG_MIN + i * pas) / 2;
        get_obst_position(scans[i], angle_i, &x_dest, &y_dest);
        explored_line((int16_t)current_x, x_dest, (int16_t)current_y, y_dest);
        place_obstacle(x_dest, y_dest);
      }
    print_image();
    printf("Done\n");

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
