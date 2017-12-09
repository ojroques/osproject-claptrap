#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "main.h"
#include "const.h"
#include "config.g"
#include "tacho.h"
#include "sensors.h"
#include "position.h"
#include "image.h"
#include "client.h"

// Angles for {EAST, NORTH, WEST, SOUTH}
const int ANGLES[4] = {0, 90, 180, -90};
int current_direction = NORTH;
int mv_history[2] = {-1, -2};

/* Return the type of obstacle detectd:
* NO_OBST: No obstacle, sonar_value is updated accordingly
* MV_OBST: Movable obstacle
* NONMV_OBST: Non-movable obstacle */
int obstacle_type(int *sonar_value) {
    int distance, new_distance, color;
    distance = *sonar_value;

    // Move forward and stop at about 4cm of the obstacle
    if (distance > 4) {
        distance = distance - 4;
    }
    forward((float)distance);

    // Check if there is really an obstacle
    new_distance = get_distance(); //TODO: Pass the correct argument
    *sonar_value = distance + new_distance;
    if (new_distance > 4) {
        backward((float)distance);
        return NO_OBST;
    }

    color = get_color(); //TODO: Pass the correct argument
    backward((float)distance);
    if (color == RED) {
        return MV_OBST;
    }
    return NONMV_OBST;
}

/* Analyse all four directions and write the corresponding sonar value into
the given array */
void analyse_env(int mesures[NB_DIRECTION]) {
    int sonar_value, obstacle;
    int i;
    for (i = 0; i < NB_DIRECTION; i++) {
        current_direction = (current_direction + i) % NB_DIRECTION;
        sonar_value = get_distance(); //TODO: Pass the correct argument
        // If non-movable obstacle detected, place obstacle
        if (sonar_value < DIST_TRESHOLD && obstacle_type(&sonar_value) == 1) {
            //TODO: Calculate the obstacle position and update image
        }
        mesures[current_direction] = sonar_value;
        turn_rigth(90);
    }
}

/* Update the last two movements */
void update_history(int new_direction) {
    mv_history[1] = mv_history[0];
    mv_history[0] = new_direction;
}

/* Return the direction with the largest free space, or -1
if there are none with at least 20cm of free space */
void choose_direction(int mesures[NB_DIRECTION]) {
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
    update_history(direction);
    return direction;
}


void move(int direction) {
    turn_rigth(ANGLES[(current_direction + direction) % NB_DIRECTION];);
    current_direction = direction;
    forward(DIST_TRESHOLD);
    // TODO: Update image
}

void map_area() {

}

int main(int argc, char *argv[]) {
    int is_stuck;
    time_t start_time;
    int mesures[NB_DIRECTION];

    is_stuck = 0;
    start_time = time(NULL);
    /*
    while (difftime(time(NULL), start_time) < EXPLORATION_TIME && !is_stuck) {

    }
    */
    return 0;
}
