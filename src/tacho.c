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

/* By Olivier
  Stop given tachos. */
void stop_tachos(uint8_t *tachos_id, int size) {
    int i;
    for (i = 0; i < size; i++) {
        set_tacho_command_inx(tachos_id[i], TACHO_STOP );
    }
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
void waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id) {
    // TODO: Write this function
}

//Erwan
void translation(uint8_t right_wheel, uint8_t left_wheel, int distance) {
    if (!distance) return;

    int max_speed, speed;
    int count_per_rot, rel_pos;

    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);

    rel_pos = (int)(((float)distance / WHEEL_PERIMETER) * count_per_rot + 0.5);
    speed = (int)((float)max_speed * TRANSLATION_SPEED / 100.0 + 0.5);

    set_tacho_speed_sp(left_wheel, speed);
    set_tacho_speed_sp(right_wheel, speed);

    set_tacho_ramp_up_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_up_sp(right_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(right_wheel, RAMP_DURATION);

    set_tacho_position_sp(left_wheel, rel_pos);
    set_tacho_position_sp(right_wheel, rel_pos);

    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);
}

//Erwan
void rotation(uint8_t right_wheel, uint8_t left_wheel, int angle) {
    if (!angle) return;

    float rad = (float)angle / 360 * 2*M_PI;
    int max_speed, speed;
    int count_per_rot, rel_pos;

    set_tacho_stop_action_inx(left_wheel, TACHO_HOLD);
    set_tacho_stop_action_inx(right_wheel, TACHO_HOLD);

    get_tacho_max_speed(left_wheel, &max_speed);
    get_tacho_count_per_rot(left_wheel, &count_per_rot);

    rel_pos = (int)((ROBOT_RADIUS * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
    speed = (int)((float)max_speed * ROTATION_SPEED / 100.0 + 0.5);

    set_tacho_speed_sp(left_wheel, speed);
    set_tacho_speed_sp(right_wheel, speed);

    set_tacho_ramp_up_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_up_sp(right_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(left_wheel, RAMP_DURATION);
    set_tacho_ramp_down_sp(right_wheel, RAMP_DURATION);

    set_tacho_position_sp(left_wheel, -rel_pos);
    set_tacho_position_sp(right_wheel, rel_pos);

    set_tacho_command_inx(left_wheel, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_wheel, TACHO_RUN_TO_REL_POS);
}

// Nathan
// Make the robot turn based on angle from gyro sensor
void rotation_gyro(uint8_t right_wheel, uint8_t left_wheel, uint8_t gyro_id, int angle) {
    if (angle == 0) {
        return;
    }

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

/* ********************** MAIN USED FOR TESTS ********************** */
int main(int argc, char *argv[]) {
    uint8_t udsn;
    uint8_t ocsn;
    uint8_t sonar_id, gyro_id;
    int max_speed, speed, rel_pos, distance;

    if (argc != 5) {
        printf("Usage: ./tacho ud_distance oc_distance angle radius\n");
        exit(-1);
    }

    int ud_distance = atoi(argv[1]);
    int oc_distance = atoi(argv[2]);
    float angle     = atof(argv[3]);
    float radius    = atof(argv[4]);
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
            printf("    Open / Close tacho ERR\n");
            exit(-1);
        }
    } else {
        printf("    Up/ Down tacho ERR\n");
        exit(-1);
    }

    //UP TONGS
    printf("Up / Down tongs... ");
    rel_pos = ud_distance;
    distance = get_avg_distance(sonar_id, NB_SENSOR_MESURE);
    printf("Distance: %d\n", distance);
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
    wait_tongs(UP_DOWN_ID);
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
    wait_tongs(OPEN_CLOSE_ID);
    printf("Done.\n");

    printf("Turning left... ");
    Sleep(500);
    if (angle == 0) {
        return 0;
    }
    uint8_t lsn;
    uint8_t rsn;
    int max_speed, speed;
    int count_per_rot;
    float rad = angle/360 * 2*M_PI;
    int rel_pos;

    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            get_tacho_max_speed(lsn, &max_speed);
            get_tacho_count_per_rot(lsn, &count_per_rot);
            rel_pos = (int)((radius * rad / WHEEL_PERIMETER) * count_per_rot + 0.5);
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
        }
    }
    wait_tachos();
    printf("Done.");
    ev3_uninit();
}

#endif
