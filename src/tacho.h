#ifndef __TACHO_H
#define __TACHO_H

#include <stdint.h>

#define ROTATION_SPEED             10   // A percentage of max_speed
#define TRANSLATION_SPEED          50   // A percentage of max_speed
#define UP_DOWN_SPEED              10
#define OPEN_CLOSE_SPEED           20
#define LEFT_WHEEL_PORT            66
#define RIGHT_WHEEL_PORT           67
#define UP_DOWN_TONG_PORT          65
#define OPEN_CLOSE_TONG_PORT       68
#define ROBOT_RADIUS               6.525
#define WHEEL_PERIMETER            17.593
#define TONGS_UP_DOWN_DISTANCE     220
#define TONGS_OPEN_CLOSE_DISTANCE  120
#define TACHO_BUFFER_SIZE          256

void wait_tachos();
void wait_tongs();
void turn_left(float angle);
void turn_right(float angle);
void forward(float distance);
void backward(float distance);
void stop_moving();
void down_tongs(uint8_t sonar_id);
void up_tongs(uint8_t sonar_id);
void close_tongs();
void open_tongs();
void turn_left_gyro(float angle, uint8_t gyro_id);

#endif
