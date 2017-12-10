#ifndef MAIN_H
#define MAIN_H

int obstacle_type(int *sonar_value);
void analyse_env(int mesures[NB_DIRECTION]);
int choose_direction(int mesures[NB_DIRECTION])
void update_history(int new_direction);
void move(int direction);

#endif
