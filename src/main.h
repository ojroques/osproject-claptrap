#ifndef MAIN_H
#define MAIN_H

#include "const.h"

int obstacle_type(int *sonar_value, uint8_t sonar_id, uint8_t color_id);
void analyse_env(int mesures[NB_DIRECTION], uint8_t sonar_id, uint8_t color_id);
int choose_direction(int mesures[NB_DIRECTION]);
void update_history(int new_direction);
void move(int direction, int mesures[NB_DIRECTION]);

#endif
