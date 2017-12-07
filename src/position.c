#include <stdlib.h>
#include "const.h"
#include <pthread.h>
#include "client.h"
#include "position.h"
#include <stdint>
#include <math.h>

//Erwan
void *position_thread(void *arg){
  while(1){
    pthread_mutex_lock(&(coordinate.coordinate_lock));
    //send_position((int16_t)(coordinate.x),(int16_t)(coordinate.x));
    printf("coordonnée X = %d, coordonnée Y = %d\n",(int16_t)(coordinate.x),(int16_t)(coordinate.x))
    pthread_mutex_unlock(&(coordinate.coordinate_lock));
    Sleep(1000);
  }
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
  coordinate.x = coordinate.x + distance*cos(coordinate.theta);
  coordinate.y = coordinate.y + distance*sin(coordinate.theta);
  pthread_mutex_unlock(&(coordinate.coordinate_lock));
}
