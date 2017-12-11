#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include "const.h"
#include "client.h"
#include "position.h"

extern coordinate_t coordinate;

//Erwan
void *position_thread(void *arg) {
  (void) arg;
  while(1){
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    send_position((int16_t)(coordinate.x), (int16_t)(coordinate.y));
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
    Sleep(1000);
  }
  pthread_exit(NULL);
}

//Erwan
void update_theta(float angle){
  pthread_mutex_lock(&(coordinate.coordinate_lock));
  coordinate.theta = coordinate.theta + angle;
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void update_coordinate(float distance){
  pthread_mutex_lock(&(coordinate.coordinate_lock));
  double rad = M_PI*coordinate.theta/180 ;
  coordinate.x = coordinate.x + distance*cos(rad);
  coordinate.y = coordinate.y + distance*sin(rad);
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
}

//Erwan
void get_obst_position(float r, float theta, int16_t *x_obst, int16_t *y_obst){
  double rad = M_PI*theta/180 ;
  pthread_mutex_lock(&(coordinate.coordinate_lock));
  *x_obst = coordinate.x + r*cos(rad);
  *y_obst = coordinate.y + r*sin(rad);
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
}
