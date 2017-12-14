#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "main.h"
#include "const.h"
#include "config.h"
#include "tacho.h"
#include "sensors.h"
#include "position.h"
#include "image.h"
#include "client.h"

coordinate_t coordinate = {60, 30, 90, PTHREAD_MUTEX_INITIALIZER};
// Angles of {EAST, NORTH, WEST, SOUTH}
const int ANGLES[NB_DIRECTION] = {0, 90, 180, -90};
const char *DIRECTIONS_NAME[NB_DIRECTION] = {"E", "N", "W", "S"};
int current_direction = NORTH;
int mv_history[2] = {-1, -2};

/* Return the type of obstacle detectd:
* NO_OBST: No obstacle, sonar_value is updated accordingly
* MV_OBST: Movable obstacle
* NONMV_OBST: Non-movable obstacle */
int obstacle_type(int *sonar_value, uint8_t sonar_id, uint8_t color_id) {
    int distance, new_distance, color;
    distance = *sonar_value;

    // Move forward and stop at about 4cm of the obstacle
    if (distance > 40) {
        distance = distance - 40;
    }

    forward(((float)distance) / 10.0);
    Sleep(DELAY_TACHO);

    // Check if there is really an obstacle
    new_distance = get_distance(sonar_id);
    Sleep(DELAY_SENSOR);
    *sonar_value = distance + new_distance; // Update sonar_value with a more reliable value
    if (new_distance > 40) {
        backward(((float)distance) / 10.0);
        Sleep(DELAY_TACHO);
        printf("No obstacle\n");
        return NO_OBST;
    }

    color = get_color(color_id);
    Sleep(DELAY_SENSOR);
    backward(((float)distance) / 10.0);
    Sleep(DELAY_TACHO);
    if (color == RED_ID) {
        printf("Movable obstacle\n");
        return MV_OBST;
    }
    printf("Non-movable obstacle\n");
    return NONMV_OBST;
}

/* Analyse all four directions and write the corresponding sonar value into
the given array */
void analyse_env(int mesures[NB_DIRECTION], uint8_t sonar_id, uint8_t color_id) {
    int sonar_value, i;
    int16_t x_obstacle, y_obstacle;

    printf("    Mesures:\n");
    for (i = 0; i < NB_DIRECTION; i++) {
        current_direction = (current_direction + i) % NB_DIRECTION;
        sonar_value = get_distance(sonar_id);
        printf("    - %s: %dmm, OBST: ", DIRECTIONS_NAME[current_direction], sonar_value);
        Sleep(DELAY_SENSOR);
        // If non-movable obstacle detected, place obstacle
        if (sonar_value < DIST_TRESHOLD && obstacle_type(&sonar_value, sonar_id, color_id) == 1) {
            get_obst_position((float)sonar_value / 10., (float)ANGLES[current_direction], &x_obstacle, &y_obstacle);
            place_obstacle(x_obstacle, y_obstacle);
        } else {
            printf("None\n");
        }
        mesures[current_direction] = sonar_value;
        if (i < NB_DIRECTION - 1) {   // To avoid returning to the initial direction
            turn_rigth(90.0);
            Sleep(DELAY_TACHO);
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
    return direction;
}

/* Update the last two movements */
void update_history(int new_direction) {
    mv_history[1] = mv_history[0];
    mv_history[0] = new_direction;
}

/* Rotate and move 20cm forward in the given direction */
void move(int direction) {
    printf("    - Rotating by %d deg... ", ANGLES[(current_direction + direction) % NB_DIRECTION]);
    turn_rigth((float)ANGLES[(current_direction + direction) % NB_DIRECTION]);
    Sleep(DELAY_TACHO);
    printf("Done.\n");
    current_direction = direction;
    printf("    - Moving by 20cm and updating history... ");
    forward(DIST_TRESHOLD / 10);
    Sleep(DELAY_TACHO);
    update_history(direction);
    printf("Done.\n");
    // TODO: Update image
}

int main() {
    int chosen_direction;
    int mesures[NB_DIRECTION];
    time_t start_time;
    pthread_t pos_thread;
    sensors_t sensors_id = config();

    if (sensors_id.is_null) {
        printf("ERROR: Initialization has failed\n");
        return EXIT_FAILURE;
    }

    if(pthread_create(&pos_thread, NULL, position_thread, NULL) == -1) {
        printf("ERROR: Could not start the position thread\n");
        return EXIT_FAILURE;
    }

    start_time = time(NULL);
    printf("********** START OF EXPLORATION  **********\n\n");
    while (difftime(time(NULL), start_time) < EXPLORATION_TIME) {
        printf("[1] ENVIRONMENT ANALYSIS\n");
        analyse_env(mesures, sensors_id.ultrasonic_sensor, sensors_id.color_sensor);
        printf("[2] DECISION\n");
        chosen_direction = choose_direction(mesures);
        if (chosen_direction == -1) {
            break;
        }
        printf("[3] MOVEMENT\n");
        move(chosen_direction);
        printf("\n");
    }

    printf("********** END OF EXPLORATION **********\n\n");
    printf("Sending image to the server...\n");
    send_image();
    printf("Done.\n");
    clean_exit();
    printf("See you later!\n");
    return EXIT_SUCCESS;
}
