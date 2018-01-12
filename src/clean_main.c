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
#include <math.h>

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
coordinate_t coordinate                   = {600., 300., 90, PTHREAD_MUTEX_INITIALIZER};    // The current position and angle
int mv_history[2]                         = {-1, -2};          // Holds the last two moves

/* Drop non-movable obstacle. */
void drop_obstacle() {
    printf("    Dropping non-movable obstacle... ");
    carrier_down_position(tachos_id.obstacle_carrier);
    wait_tacho(tachos_id.obstacle_carrier);
    Sleep(1000);
    carrier_up_position(tachos_id.obstacle_carrier);
    wait_tacho(tachos_id.obstacle_carrier);
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
    translation(tachos_id.right_wheel, tachos_id.left_wheel, distance, tachos_id.ultrasonic_tacho, sensors_id.ultrasonic_sensor);
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    // Check if there is really an obstacle
    new_distance = get_avg_distance(sensors_id.ultrasonic_sensor, NB_SENSOR_MESURE);
    *sonar_value = distance + new_distance; // Update sonar_value with a more reliable value

    // Check color, after multiplying TRESHOLD_COLOR by 2 as an error margin
    if (new_distance > 2*TRESHOLD_COLOR) {
        translation(tachos_id.right_wheel, tachos_id.left_wheel, -distance, tachos_id.ultrasonic_tacho, sensors_id.ultrasonic_sensor);
        printf("No obstacle (dist: %d)\n", new_distance);
        return NO_OBST;
    }

    // Get the obstacle color
    color = get_avg_color(sensors_id.color_sensor, NB_SENSOR_MESURE);
    translation(tachos_id.right_wheel, tachos_id.left_wheel, -distance, tachos_id.ultrasonic_tacho, sensors_id.ultrasonic_sensor);
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
            rotation_gyro(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.gyro_sensor, 90);
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
        is_looping = (mv_history[1] == ((mv_history[0] + 2) % NB_DIRECTION) && i == mv_history[0]);
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
    rotation_gyro(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.gyro_sensor, ANGLES[ang]);
    wait_wheels(tachos_id.right_wheel, tachos_id.left_wheel);
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM

    // Go forward until an obstacle is detected
    travel_distance = mesures[direction] - TRESHOLD_MANEUVER;
    printf("    - Moving by %dmm and updating history... ", travel_distance);
    translation(tachos_id.right_wheel, tachos_id.left_wheel, travel_distance, tachos_id.ultrasonic_tacho, sensors_id.ultrasonic_sensor);
    update_history(direction);
    printf("Done.\n");
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM
    // TODO: Update image accordingly
}

/* Go to position (x, y)
 * Return 1 if stopped by an obstacle, else 0.*/
void goto_area(int16_t x_unexp, int16_t y_unexp) {
    int r, theta;
    int16_t delta_x, delta_y;

    delta_x = x_unexp - round(coordinate.x);
    delta_y = y_unexp - round(coordinate.y);
    r = round(sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
    theta = round(180 * 2 * atan((double)delta_y / (double)(delta_x + r)) / M_PI) - coordinate.theta;

    rotation_gyro(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.gyro_sensor, theta);
    wait_wheels(tachos_id.right_wheel, tachos_id.left_wheel);
    translation(tachos_id.right_wheel, tachos_id.left_wheel, r, tachos_id.ultrasonic_tacho, sensors_id.ultrasonic_sensor);
    rotation_gyro(tachos_id.right_wheel, tachos_id.left_wheel, sensors_id.gyro_sensor, -theta);
    wait_wheels(tachos_id.right_wheel, tachos_id.left_wheel);

    if (MAIN_DEBUG) {
        printf("r: %d, theta: %d", r, theta);
        getchar();  // PAUSE PROGRAM
    }
}

void algorithm() {
    time_t start_time;                  // The robot stops after 3mn50
    int mesures[NB_DIRECTION] = {0};    // Contains the mesured distance of all 4 directions
    int chosen_direction;               // The direction the robot will move to
    int16_t x_unexp, y_unexp;           // Position of an unexplored area
    int i, running;

    running = 1;                        // Exploration running or not
    start_time = time(NULL);
    printf("********** START OF EXPLORATION  **********\n\n");

    drop_obstacle();
    if (MAIN_DEBUG) getchar();  // PAUSE PROGRAM
    while (running) {
         unexplored_area(&x_unexp, &y_unexp);
         printf("UNEXPLORED AREA: (%d, %d)\n\n", x_unexp, y_unexp);
         goto_area(x_unexp, y_unexp);

        for (i = 0; i < NB_ANALYSIS; i++) {
            printf("[1] ENVIRONMENT ANALYSIS\n");
            analyse_env(mesures);
            printf("[2] DECISION\n");
            chosen_direction = choose_direction(mesures);
            if (chosen_direction == -1) {
                running = 0;
                break;
            }
            printf("[3] MOVEMENT\n");
            move(chosen_direction, mesures);
            printf("\n");
            if (difftime(time(NULL), start_time) < EXPLORATION_TIME) {
                running = 0;
                break;
            }
        }
    }

    printf("********** END OF EXPLORATION **********\n\n");
}

int main(int argc, char *argv[]) {

    // Retrieve the map dimensions
    if (argc != 1 && argc != 5) {
        printf("Usage 1: %s <map_width> <map_height> <x_init> <y_init>\n", argv[0]);
        printf("Usage 2: %s - Default values: (24, 40)\n", argv[0]);
        return EXIT_FAILURE;
    }

    // Variables initialization
    int map_width, map_height;          // The map dimensions
    pthread_t pos_thread;               // The position thread

    // General configuration
    signal(SIGINT, clean_exit);         // Redirect CTRL + C to clean_exit in config.c
    if (argc == 1) {
        map_width  = 24;
        map_height = 40;
    } else {
        map_width    = atoi(argv[1]);
        map_height   = atoi(argv[2]);
        coordinate.x = atof(argv[3]);
        coordinate.y = atof(argv[4]);
    }

    if (!config_all(&sensors_id, &tachos_id, map_width, map_height)) {
        printf("ERROR: Initialization has failed\n");
        clean_exit(0);
    }

    // Run the position thread, sending the current position every 1.5s to the server
    int wheels_id[2] = {tachos_id.right_wheel, tachos_id.left_wheel};
    if(pthread_create(&pos_thread, NULL, position_thread, wheels_id) == -1) {
        printf("ERROR: Could not start the position thread\n");
        clean_exit(0);
    }

    // The main algorithm
    printf("Press any key to begin exploration\n");
    getchar();

    if (MAIN_DEBUG) {
        while(1) {
            printf("Rotation impossible ? %d\n", is_rotation_impossible());
            getchar();
            printf("Sonar value: %d\n", get_dir_distance());
        }
    } else {
        algorithm();
    }

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