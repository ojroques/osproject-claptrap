#ifndef __TACHO_H
#define __TACHO_H

#include <stdint.h>

#define ROTATION_SPEED     15   // A percentage of max_speed
#define TRANSLATION_SPEED  30   // A percentage of max_speed
#define RAMP_DURATION      100
#define ROBOT_RADIUS       46.5
#define WHEEL_PERIMETER    175.93
#define TACHO_BUFFER_SIZE  256

void stop_tacho(uint8_t tacho_id);
void wait_wheels(uint8_t right_wheel, uint8_t left_wheel);
void waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id);
void translation(uint8_t right_wheel, uint8_t left_wheel, int distance);
void rotation(uint8_t right_wheel, uint8_t left_wheel, int angle);
void rotation_gyro(uint8_t right_wheel, uint8_t left_wheel, uint8_t gyro_id, int angle);

#endif
