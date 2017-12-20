#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "const.h"
#include "client.h"
#include "position.h"

extern coordinate_t coordinate;
extern volatile int quit_request;

//Erwan
void *position_thread(void *arg) {
    (void) arg;   // To avoid the warning message
    while (!quit_request) {
        pthread_mutex_lock(&(coordinate.coordinate_lock));
        send_position((int16_t)(coordinate.x), (int16_t)(coordinate.y));
        pthread_mutex_unlock(&(coordinate.coordinate_lock));
        Sleep(1500);
    }
    pthread_exit(NULL);
}

//Erwan
void update_theta(int angle) {
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    coordinate.theta = coordinate.theta + angle;
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void update_coordinate(int distance) {
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    float rad = M_PI * (float)coordinate.theta / 180 ;
    coordinate.x = coordinate.x + (round)(distance * cos(rad));
    coordinate.y = coordinate.y + (round)(distance * sin(rad));
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void get_obst_position(int r, int theta, int16_t *x_obst, int16_t *y_obst) {
    float rad = M_PI * (float)theta / 180 ;
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    *x_obst = coordinate.x + (round)(r * cos(rad));
    *y_obst = coordinate.y + (round)(r * sin(rad));
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}
