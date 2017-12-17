/* Written by Olivier Roques for the OS project.
Eurecom, 2017 - 2018. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>

#include "main.h"
#include "const.h"
#include "config.h"
#include "tacho.h"
#include "sensors.h"
#include "position.h"
#include "image.h"
#include "client.h"

#define MAIN_DEBUG 0

coordinate_t coordinate = {60, 30, 90, PTHREAD_MUTEX_INITIALIZER};
volatile int quit_request = 0;   // To stop the position thread
sensors_t sensors_id;
// Angles of {EAST, NORTH, WEST, SOUTH}
const int ANGLES[NB_DIRECTION] = {0, 90, 180, -90};
const char *DIRECTIONS_NAME[NB_DIRECTION] = {"E", "N", "W", "S"};
int current_direction = NORTH;
int mv_history[2] = {-1, -2};

/* Drop non-movable obstacle. */
void drop_obstacle() {
    printf("    Dropping non-movable obstacle... ");
    down_tongs(sensors_id.ultrasonic_sensor);
    wait_tongs();
    open_tongs();
    wait_tongs();
    Sleep(1000);    // Wait for the ball to stop moving
    up_tongs(sensors_id.ultrasonic_sensor);
    wait_tongs();
    close_tongs();
    wait_tongs();
    printf("Done.\n");
}

/* Grab non-movable obstacle. */
void grab_obstacle() {
    printf("    Grabbing non-movable obstacle... ");
    open_tongs();
    wait_tongs();
    down_tongs(sensors_id.ultrasonic_sensor);
    wait_tongs();
    close_tongs();
    wait_tongs();
    up_tongs(sensors_id.ultrasonic_sensor);
    wait_tongs();
    printf("Done.\n");
}

/* Return the type of obstacle detectd:
* NO_OBST: No obstacle, sonar_value is updated accordingly
* MV_OBST: Movable obstacle
* NONMV_OBST: Non-movable obstacle */
int obstacle_type(int *sonar_value) {
    int distance, new_distance, color;
    distance = *sonar_value;

    // Move forward and stop at about 4cm of the obstacle
    if (distance > DIST_COLOR) {
        distance = distance - DIST_COLOR;
    }

    forward(((float)distance) / 10.0);
    wait_tachos();
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    // Check if there is really an obstacle
    new_distance = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
    *sonar_value = distance + new_distance; // Update sonar_value with a more reliable value
    if (new_distance > DIST_COLOR) {
        backward(((float)distance) / 10.0);
        wait_tachos();
        printf("No obstacle (dist: %d)\n", new_distance);
        return NO_OBST;
    }

    // Get the obstacle color
    color = get_avg_color(sensors_id.color_sensor, NB_SENSOR_MESURE);
    backward(((float)distance) / 10.0);
    wait_tachos();
    if (color == RED_ID) {
        printf("Movable obstacle (color: %d)\n", color);
        // grab_obstacle();
        return MV_OBST;
    }
    printf("Non-movable obstacle (color: %d)\n", color);
    return NONMV_OBST;
}

/* Analyse all four directions and write the corresponding sonar value into
the given array */
void analyse_env(int mesures[NB_DIRECTION]) {
    int sonar_value, initial_direction, i;
    int16_t x_obstacle, y_obstacle;

    printf("    Mesures:\n");

    initial_direction = current_direction;
    for (i = 0; i < NB_DIRECTION; i++) {
        current_direction = (initial_direction + i) % NB_DIRECTION;
        sonar_value = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
        printf("    - %s: %dmm, OBST: ", DIRECTIONS_NAME[current_direction], sonar_value);
        // If non-movable obstacle detected, place obstacle
        if (sonar_value < DIST_TRESHOLD && obstacle_type(&sonar_value) == NONMV_OBST) {
            get_obst_position((float)sonar_value / 10., (float)ANGLES[current_direction], &x_obstacle, &y_obstacle);
            place_obstacle(x_obstacle, y_obstacle);
        } else {
            printf("None\n");
        }
        mesures[current_direction] = sonar_value;
        if (MAIN_DEBUG) getchar();    // PAUSE PROGRAM
        if (i < NB_DIRECTION - 1) {   // To avoid returning to the initial direction
            turn_left(90.0);
            wait_tachos();
        }
    }
}

/* Return the direction with the largest free space (> 20cm)
or -1 if there are none */
int choose_direction(int mesures[NB_DIRECTION]) {
    int i, direction, is_looping;
    direction = -1;
    for (i = 0; i < NB_DIRECTION; i++) {
        // is_looping indicates if direction i would result in a looping route
        is_looping = (mv_history[0] == mv_history[1] && i == mv_history[1]);
        if (mesures[i] >= DIST_TRESHOLD && !is_looping) {
            if (direction == -1 || mesures[i] > mesures[direction]) {
                direction = i;
            }
        }
    }
    printf("    - CURRENT DIRECTION: %s\n", DIRECTIONS_NAME[current_direction]);
    if (direction != -1) {
        printf("    - CHOSEN DIRECTION: %s\n", DIRECTIONS_NAME[direction]);
    }
    else {
        printf("    - Claptrap is stuck!\n");
    }
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM
    return direction;
}

/* Update the last two movements */
void update_history(int new_direction) {
    current_direction = new_direction;
    mv_history[1] = mv_history[0];
    mv_history[0] = new_direction;
}

/* While moving, this function checks if there is an obstacle and stop tachos
   if indeed there is one. */
void check_obstacle() {
    int dist_available;
    char lsn_state[TACHO_BUFFER_SIZE];
    char rsn_state[TACHO_BUFFER_SIZE];
    uint8_t lsn, rsn;

    ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &lsn, 0);
    ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0);

    do {
        dist_available = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
        if (dist_available < DIST_TRESHOLD) {
            stop_moving();
            break;
        }
        get_tacho_state(lsn, lsn_state, TACHO_BUFFER_SIZE);
        get_tacho_state(rsn, rsn_state, TACHO_BUFFER_SIZE);
        Sleep(200);
    } while (strcmp("holding", lsn_state) && strcmp("holding", rsn_state));
}

/* Rotate and move 20cm forward in the given direction */
void move(int direction, int mesures[NB_DIRECTION]) {
    int travel_distance, rotation;

    rotation = direction - current_direction;
    if (rotation < 0) {
        rotation = rotation + NB_DIRECTION;
    }
    printf("    - Rotating by %d deg... ", ANGLES[rotation]);
    turn_left((float)ANGLES[rotation]);
    wait_tachos();
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    travel_distance = mesures[direction] - DIST_TRESHOLD;
    printf("    - Moving by %dmm and updating history... ", travel_distance);
    forward((float)travel_distance / 10.0);
    check_obstacle();
    update_history(direction);
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM
    // TODO: Update image accordingly
}

int main() {
    int chosen_direction;
    int mesures[NB_DIRECTION] = {0};
    time_t start_time;
    pthread_t pos_thread;
    sensors_id = config();

    if (sensors_id.is_null) {
        printf("ERROR: Initialization has failed\n");
        return EXIT_FAILURE;
    }

    if(pthread_create(&pos_thread, NULL, position_thread, NULL) == -1) {
        printf("ERROR: Could not start the position thread\n");
        return EXIT_FAILURE;
    }

    printf("Press any key to begin exploration\n");
    getchar();
    drop_obstacle();
    start_time = time(NULL);
    printf("********** START OF EXPLORATION  **********\n\n");
    while (difftime(time(NULL), start_time) < EXPLORATION_TIME) {
        printf("[1] ENVIRONMENT ANALYSIS\n");
        analyse_env(mesures);
        printf("[2] DECISION\n");
        chosen_direction = choose_direction(mesures);
        if (chosen_direction == -1) {
            break;
        }
        printf("[3] MOVEMENT\n");
        move(chosen_direction, mesures);
        printf("\n");
    }

    printf("********** END OF EXPLORATION **********\n\n");
    printf("Killing position thread...");
    quit_request = 1;
    if (!pthread_join(pos_thread, NULL)) {
        printf("Done.\n");
    } else {
        printf("Error.\n");
    }
    printf("Sending image to the server...\n");
    send_image();
    printf("Done.\n");
    clean_exit();
    printf("See you later!\n");
    return EXIT_SUCCESS;
}
