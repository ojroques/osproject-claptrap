#ifndef __TACHO_H
#define __TACHO_H

#include <stdint.h>

#define ROTATION_SPEED     15   // A percentage of max_speed
#define TRANSLATION_SPEED  30   // A percentage of max_speed
#define RAMP_DURATION      100
#define ROBOT_RADIUS       46.5
#define WHEEL_PERIMETER    169.5
#define LANE_WIDTH 170
#define TACHO_BUFFER_SIZE  256

void stop_tacho(uint8_t tacho_id);
void wait_wheels(uint8_t right_wheel, uint8_t left_wheel);
int  waitncheck_wheels(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id);
void wait_tacho(uint8_t tacho);
int translation(uint8_t right_wheel, uint8_t left_wheel, int distance, uint8_t ultrasonic_tacho, uint8_t ultrasonic_id);
void rotation(uint8_t right_wheel, uint8_t left_wheel, int angle);
void rotation_gyro(uint8_t right_wheel, uint8_t left_wheel, uint8_t gyro_id, int angle);

void operate_tacho(uint8_t tacho, int angle);

void carrier_middle_position(uint8_t obstacle_carrier);
void carrier_down_position(uint8_t obstacle_carrier);
void carrier_up_position(uint8_t obstacle_carrier);

int single_scan(uint8_t ultrasonic_tacho, uint8_t sonar_id, int angle);
void scan_distance(uint8_t ultrasonic_tacho, uint8_t sonar_id, int number_of_scan, int min_angle, int max_angle, int * array_of_scan_values);
int update_distance(uint8_t right_wheel, uint8_t left_wheel, uint8_t ultrasonic_id, int position_start, int count_per_rot, int previous_traveled_distance, int previous_distance);

#endif
