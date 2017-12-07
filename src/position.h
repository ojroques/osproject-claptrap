#ifndef __POSITION_H
#define __POSITION_H

typedef struct coordinate_t{
  float x,y,theta;
  pthread_mutex_t coordinate_lock;
}coordinate_t;

void *position_thread(void *arg);
void update_theta(float angle);
void update_coordinate(float distance);

#endif
