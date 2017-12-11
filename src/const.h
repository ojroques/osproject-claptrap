#ifndef CONST_H
#define CONST_H

/* GENERAL */
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define SIZE_OBSTACLE 2 /* The size of an obstacle's side */
#define EXPLORATION_TIME 210 /* The duration of exploration, in seconds */

/* USED BY MAIN.C */
#define RED_ID        5
#define NO_OBST      -1
#define MV_OBST       0
#define NONMV_OBST    1
#define NB_DIRECTION  4
#define NORTH         1
#define WEST          2
#define SOUTH         3
#define EAST          0
#define DIST_TRESHOLD 200

/* USED BY CLIENT.C */
#define SERV_ADDR        "00:28:f8:51:71:4b"    /* The address of the server */
#define TEAM_ID          7                      /* The team ID */
#define MSG_ACK          0
#define MSG_START        1
#define MSG_STOP         2
#define MSG_KICK         3
#define MSG_POSITION     4
#define MSG_MAPDATA      5
#define MSG_MAPDONE      6
#define MSG_OBSTACLE     7
#define CONNECTION_ERROR 0
#define WRONG_MESSAGE    1
#define START_MESSAGE    2

/* USED BY IMAGE.C */
#define ERR_OUT_OF_BOUNDS -1
#define IMG_WIDTH         24
#define IMG_HEIGHT        40

/* USED BY TACHO.C */
#define LEFT_WHEEL_PORT  66
#define RIGHT_WHEEL_PORT 67
#define ROBOT_RADIUS     6.05
#define WHEEL_PERIMETER  17.5929188601

#endif
