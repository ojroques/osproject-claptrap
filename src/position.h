#ifndef __POSITION_H
#define __POSITION_H

void *position_thread(void *arg);
void update_theta(int angle);
void update_coordinate(int distance);
void get_obst_position(int r, int theta, int16_t *x_obst, int16_t *y_obst);

typedef struct coordinate_t{
    int x, y, theta;
    pthread_mutex_t coordinate_lock;
} coordinate_t;

#endif
