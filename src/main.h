#ifndef MAIN_H
#define MAIN_H

#define EXPLORATION_TIME  225 /* The duration of exploration, in seconds */
#define RED_ID            5
#define NO_OBST          -1
#define MV_OBST           0
#define NONMV_OBST        1
#define NB_DIRECTION      4
#define NORTH             1
#define WEST              2
#define SOUTH             3
#define EAST              0
#define DIST_TRESHOLD     200   // In millimeters
#define DIST_COLOR        40    // In millimeters

void drop_obstacle();
int obstacle_type(int *sonar_value);
void analyse_env(int mesures[NB_DIRECTION]);
int choose_direction(int mesures[NB_DIRECTION]);
void update_history(int new_direction);
void check_obstacle();
void move(int direction, int mesures[NB_DIRECTION]);

#endif
