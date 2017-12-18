#ifndef __TACHO_H
#define __TACHO_H

#include <stdint.h>

#define ROTATION_SPEED             15   // A percentage of max_speed
#define TRANSLATION_SPEED          30   // A percentage of max_speed
#define UP_DOWN_ID                 0
#define UP_DOWN_TONG_PORT          65
#define UP_DOWN_SPEED              20
#define OPEN_CLOSE_ID              1
#define OPEN_CLOSE_TONG_PORT       68
#define OPEN_CLOSE_SPEED           10
#define LEFT_WHEEL_PORT            66
#define RIGHT_WHEEL_PORT           67
#define ROBOT_RADIUS               4.65
#define WHEEL_PERIMETER            17.593
#define TACHO_BUFFER_SIZE          256
#define TONGS_OPEN_CLOSE_DISTANCE  120
#define TONGS_UP_DOWN_DISTANCE     220

void wait_tachos();
void wait_tongs(int id);
void turn_left(float angle);
void turn_right(float angle);
void forward(float distance);
void backward(float distance);
void stop_moving();
void stop_tongs();
void down_tongs(uint8_t sonar_id);
void up_tongs(uint8_t sonar_id);
void close_tongs();
void open_tongs();
void turn_left_gyro(float angle, uint8_t gyro_id);
void turn_gyro_left(float angle, uint8_t gyro_id);
void turn_gyro(float angle, uint8_t gyro_id);

#endif
