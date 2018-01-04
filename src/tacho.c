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

/* By Olivier.
   While moving, this function checks if there is an obstacle and stop tachos
   if indeed there is one. */
void waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id, position_start) {
    int current_distance, count_per_rot, current_position, temp;
    char right_state[TACHO_BUFFER_SIZE];
    char left_state[TACHO_BUFFER_SIZE];
    get_tacho_count_per_rot(left_wheel, &count_per_rot);
    current_position = position_start;
    do {
        current_distance = get_avg_distance(ultrasonic_id, NB_SENSOR_MESURE);
        if (current_distance < TRESHOLD_MANEUVER) {
            stop_tacho(right_wheel);
            stop_tacho(left_wheel);
            temp = current_position;
            get_tacho_position(left_wheel, &current_position);
            update_coordinate(WHEEL_PERIMETER * abs(current_position - temp) / count_per_rot);
            break;
        }
        get_tacho_state(right_wheel, right_state, TACHO_BUFFER_SIZE);
        get_tacho_state(left_wheel, left_state, TACHO_BUFFER_SIZE);
        temp = current_position;
        get_tacho_position(left_wheel, &current_position);
        update_coordinate(WHEEL_PERIMETER * abs(current_position - temp) / count_per_rot);
        Sleep(200);
    } while (strcmp("holding", right_state) && strcmp("holding", left_state));
}

/* By Erwan
   Tranlate by X millimeters. */
void translation(uint8_t right_wheel, uint8_t left_wheel,uint8_t ultrasonic_id, int distance) {
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
    get_tacho_position(left_wheel, &position_start);

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

    if (distance > 0){
      waitncheck_wheels(right_wheel, left_wheel, ultrasonic_id, position_start);
    }
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


#ifdef TACHO_DEBUG

#define LEFT_WHEEL_PORT        66
#define RIGHT_WHEEL_PORT       67

/* ********************** MAIN USED FOR TESTS ********************** */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: ./tacho translation_distance rotation_angle\n");
        exit(-1);
    }

    uint8_t right_wheel, left_wheel;
    uint8_t sonar_id, color_id;

    int translation_dist = atoi(argv[1]);
    int rotation_angle   = atoi(argv[2]);

    ev3_sensor_init();
    ev3_tacho_init();
    ev3_search_sensor(LEGO_EV3_US, &sonar_id, 0);
    ev3_search_sensor(LEGO_EV3_COLOR, &color_id, 0);

    printf("Initializing tachos...\n");
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &right_wheel, 0)) {
        printf("    [OK] Right wheel\n");
        if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &left_wheel, 0)) {
            printf("    [OK] Left wheel\n");
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
    rotation(right_wheel, left_wheel, rotation_angle);
    wait_wheels(right_wheel, left_wheel);
    printf("Done.\n");

    printf("Moving forward by %d mm and detecting obstacles... ", translation_dist);
    translation(right_wheel, left_wheel, translation_dist, sonar_id);
    printf("Done.\n");

    printf("Moving backward by %d mm... ", translation_dist);
    translation(right_wheel, left_wheel, -translation_dist);
    wait_wheels(right_wheel, left_wheel);
    printf("Done.\n");

    printf("Color detected: %d\n", get_avg_color(color_id, NB_SENSOR_MESURE));

    set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
    set_tacho_stop_action_inx(left_wheel, TACHO_COAST);
    stop_tacho(right_wheel);
    stop_tacho(left_wheel);
    ev3_uninit();
}

#endif
