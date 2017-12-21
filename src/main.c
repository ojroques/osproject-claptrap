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

volatile int quit_request                 = 0;    // To stop the position thread
sensors_t sensors_id                      = {0, 0, 0, 0, 0};    // Contains the sensors' identifiant
tachos_t tachos_id                        = {0, 0, 0, 0};       // Contains the tachos' identifiant
int current_direction                     = NORTH;              // Direction faced by the robot at the beginning
const int ANGLES[NB_DIRECTION]            = {0, 90, 180, -90};  // Angles for each direction from east
const char *DIRECTIONS_NAME[NB_DIRECTION] = {"E", "N", "W", "S"};    // Name of all 4 directions
coordinate_t coordinate                   = {600, 300, 90, PTHREAD_MUTEX_INITIALIZER};    // The current position and angle
int mv_history[2]                         = {-1, -2};          // Holds the last two moves

/* Drop non-movable obstacle. */
void drop_obstacle() {
    printf("    Dropping non-movable obstacle... ");
    // TODO: Rewrite function
    Sleep(1000);
    printf("Done.\n");
}

/* Return the type of obstacle detectd:
* NO_OBST: No obstacle, sonar_value is updated accordingly
* MV_OBST: Movable obstacle
* NONMV_OBST: Non-movable obstacle */
int obstacle_type(int *sonar_value) {
    int distance, new_distance, color;
    distance = *sonar_value;

    // Move forward and stop at about 3cm of the obstacle
    if (distance > TRESHOLD_COLOR) {
        distance = distance - TRESHOLD_COLOR;
    }
    translation(tachos_id.right_wheel, tachos_id.left_wheel, distance);
    waitncheck_wheels(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.ultrasonic_sensor);
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    // Check if there is really an obstacle
    new_distance = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
    *sonar_value = distance + new_distance; // Update sonar_value with a more reliable value

    // Check color, after multiplying TRESHOLD_COLOR by 2 as an error margin
    if (new_distance > 2*TRESHOLD_COLOR) {
        translation(tachos_id.right_wheel, tachos_id.left_wheel, -distance);
        waitncheck_wheels(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.ultrasonic_sensor);
        printf("No obstacle (dist: %d)\n", new_distance);
        return NO_OBST;
    }

    // Get the obstacle color
    color = get_avg_color(sensors_id.color_sensor, NB_SENSOR_MESURE);
    translation(tachos_id.right_wheel, tachos_id.left_wheel, -distance);
    waitncheck_wheels(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.ultrasonic_sensor);
    if (color == RED_ID) {
        printf("Movable obstacle (color: %d)\n", color);
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
        // Mesure distance of the current direction
        current_direction = (initial_direction + i) % NB_DIRECTION;
        sonar_value = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
        printf("    - %s: %dmm, OBST: ", DIRECTIONS_NAME[current_direction], sonar_value);

        // If non-movable obstacle detected, place obstacle
        if (sonar_value < TRESHOLD_CHECK_OBST && obstacle_type(&sonar_value) == NONMV_OBST) {
            get_obst_position(sonar_value, ANGLES[current_direction], &x_obstacle, &y_obstacle);
            place_obstacle(x_obstacle, y_obstacle);
        } else {
            printf("None\n");
        }

        // Rotate to the next direction
        mesures[current_direction] = sonar_value;
        if (MAIN_DEBUG) getchar();    // PAUSE PROGRAM
        if (i < NB_DIRECTION - 1) {   // To avoid returning to the initial direction
            rotation(tachos_id.right_wheel, tachos_id.left_wheel, 90);
            wait_wheels(tachos_id.right_wheel, tachos_id.left_wheel);
        }
    }
}

/* Return the direction with the largest free space (> 10cm)
or -1 if there are none */
int choose_direction(int mesures[NB_DIRECTION]) {
    int i, direction, is_looping;
    direction = -1;

    for (i = 0; i < NB_DIRECTION; i++) {
        // is_looping indicates if direction i would result in a looping route
        is_looping = (mv_history[1] == (mv_history[0] + 2) % NB_DIRECTION && i == mv_history[0]);
        if (mesures[i] >= TRESHOLD_MANEUVER && !is_looping) {
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

/* Rotate and move 20cm forward in the given direction */
void move(int direction, int mesures[NB_DIRECTION]) {
    int travel_distance, ang;

    // Calculate the angle from current direction to chosen direction
    ang = direction - current_direction;
    if (ang < 0) {
        ang = ang + NB_DIRECTION;
    }

    // Rotate accordingly
    printf("    - Rotating by %d deg... ", ANGLES[ang]);
    rotation(tachos_id.right_wheel, tachos_id.left_wheel, ANGLES[ang]);
    wait_wheels(tachos_id.right_wheel, tachos_id.left_wheel);
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    // Go forward until an obstacle is detected
    travel_distance = mesures[direction] - TRESHOLD_MANEUVER;
    printf("    - Moving by %dmm and updating history... ", travel_distance);
    translation(tachos_id.right_wheel, tachos_id.left_wheel, travel_distance);
    waitncheck_wheels(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.ultrasonic_sensor);
    update_history(direction);
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM
    // TODO: Update image accordingly
}

int main() {
    signal(SIGINT, clean_exit);
    pthread_t pos_thread;

    // General configuration
    if (!config_all(&sensors_id, &tachos_id)) {
        printf("ERROR: Initialization has failed\n");
        return EXIT_FAILURE;
    }

    // Run the position thread, sending the current position every 1.5s to the server
    if(pthread_create(&pos_thread, NULL, position_thread, NULL) == -1) {
        printf("ERROR: Could not start the position thread\n");
        return EXIT_FAILURE;
    }

    time_t start_time;                  // The robot stops after 3mn50
    int mesures[NB_DIRECTION] = {0};    // Contains the mesured distance of all 4 directions
    int chosen_direction;               // The direction the robot will move to

    printf("Press any key to begin exploration\n");
    getchar();

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

    // Stop sending the current position
    printf("Killing position thread...");
    quit_request = 1;
    if (!pthread_join(pos_thread, NULL)) {
        printf("Done.\n");
    } else {
        printf("Error.\n");
    }

    // Send the map to the server
    printf("Sending map to the server...\n");
    send_image();
    printf("Done.\n");

    clean_exit(0);
    return EXIT_SUCCESS;
}
