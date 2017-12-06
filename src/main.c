#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "main.h"
/*
#include "const.h"
#include "config.g"
#include "tacho.h"
#include "sensors.h"
#include "position.h"
#include "image.h"
#include "client.h"

int obstacle_type(int distance) {
    if (distance > 3) {
        distance = distance - 3;
    }
}

void analyse_env() {

}

void choose_direction() {

}

void move() {

}

void map_area() {

}

*/

int main(int argc, char *argv[]) {
    int is_stuck;
    time_t start_time;

    is_stuck = 0;
    start_time = time(NULL);
    /*
    while (difftime(time(NULL), start_time) < EXPLORATION_TIME && !is_stuck) {

    }
    */
    return 0;
}
