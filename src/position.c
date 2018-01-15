#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "sensors.h"
#include "const.h"
#include "client.h"
#include "position.h"
#include <stdio.h>

#define POSITION_DEBUG 0

extern coordinate_t coordinate;
extern volatile int quit_request;

//Erwan
void *position_thread(void *arg) {
    (void) arg;   // To avoid the warning message
    while (!quit_request) {
        pthread_mutex_lock(&(coordinate.coordinate_lock));
        send_position(coord_to_index((int16_t)coordinate.x), coord_to_index((int16_t)coordinate.y));
        pthread_mutex_unlock(&(coordinate.coordinate_lock));
        Sleep(2000);
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
void recalibrate_theta(uint8_t compass_id, int compass_starting_angle){
    Sleep(4000); // We let the time for the compass to calibrate itself
    int compass_angle = get_compass_direction(compass_id);
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    // We check if the derivation of theta is not too much
    if (abs((coordinate.theta - 90) - (compass_angle - compass_starting_angle)) > 2){
      if (POSITION_DEBUG) printf("modification of theta, old value = %d, new value = %d\n", coordinate.theta,compass_angle - compass_starting_angle + 90);
      coordinate.theta = compass_angle - compass_starting_angle + 90; // if it is we recalibrate by trusting the compass
    }
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void update_coordinate(int distance) {
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    float rad = M_PI * (float)coordinate.theta / 180. ;
    coordinate.x = coordinate.x + (distance * cos(rad));
    coordinate.y = coordinate.y + (distance * sin(rad));
    if (POSITION_DEBUG) printf("coordinate X = %lf, coordinate Y = %lf\n", coordinate.x, coordinate.y);
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void get_obst_position(int r, int theta, int16_t *x_obst, int16_t *y_obst) {
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    float rad_robot = M_PI * (float)coordinate.theta / 180.;
    float rad_ultrasonic = M_PI * (float)(theta + coordinate.theta) / 180.;
    *x_obst = (int16_t)(coordinate.x + (round)(DIST_CENTER_SONAR * cos(rad_robot) + (r * cos(rad_ultrasonic))));
    *y_obst = (int16_t)(coordinate.y + (round)(DIST_CENTER_SONAR * sin(rad_robot) + (r * sin(rad_ultrasonic))));
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
float get_coordinate_x(){
  pthread_mutex_lock(&(coordinate.coordinate_lock));
  float buffer = (float)coordinate.x;
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
  return buffer;
}

//Erwan
float get_coordinate_y(){
  pthread_mutex_lock(&(coordinate.coordinate_lock));
  float buffer = (float)coordinate.y;
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
  return buffer;
}
