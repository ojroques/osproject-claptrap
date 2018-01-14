#ifndef __POSITION_H
#define __POSITION_H

#include <stdint.h>

void *position_thread(void *arg);
void update_theta(int angle);
void update_coordinate(int distance);
void get_obst_position(int r, int theta, int16_t *x_obst, int16_t *y_obst);
void recalibrate_theta(uint8_t compass_id, int compass_starting_angle);
float get_coordinate_y();
float get_coordinate_x();

#define DIST_CENTER_SONAR 100;

typedef struct coordinate_t{
    double x, y;
    int theta;
    pthread_mutex_t coordinate_lock;
} coordinate_t;

#endif
