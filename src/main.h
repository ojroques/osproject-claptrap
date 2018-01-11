#ifndef MAIN_H
#define MAIN_H

#define EXPLORATION_TIME  230   // The duration of exploration, in seconds
#define RED_ID              5
#define NO_OBST            -1
#define MV_OBST             0
#define NONMV_OBST          1
#define NB_DIRECTION        4
#define NORTH               1
#define WEST                2
#define SOUTH               3
#define EAST                0
#define NB_ANALYSIS         4

void drop_obstacle();
int obstacle_type(int *sonar_value);
void analyse_env(int mesures[NB_DIRECTION]);
int choose_direction(int mesures[NB_DIRECTION]);
void update_history(int new_direction);
void move(int direction, int mesures[NB_DIRECTION]);
int goto_area(int16_t x_unexp, int16_t y_unexp); 
int is_rotation_impossible();
int get_dir_distance();

#endif
