#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

#define RIGHT_WHEEL_PORT       67
#define LEFT_WHEEL_PORT        66
#define ULTRASONIC_TACHO_PORT  65
#define CARRIER_PORT           68

// Nathan
// Struct with the id of each sensors
typedef struct sensors_t {
    uint8_t color_sensor;
    uint8_t gyro_sensor;
    uint8_t ultrasonic_sensor;
    uint8_t compass_sensor;
    uint8_t touch_sensor;
} sensors_t;

// Olivier
// Struct with the id of each tachos
typedef struct tachos_t {
    uint8_t right_wheel;
    uint8_t left_wheel;
    uint8_t ultrasonic_tacho;
    uint8_t obstacle_carrier;
} tachos_t;

void search_sensor(uint8_t sensor_type, uint8_t *sensor_id, char *sensor_name);
void search_tacho(uint8_t motor_port, uint8_t *tacho_id, char *tacho_name);
void config_sensors(sensors_t *sensors_id);
void config_tacho(tachos_t *tachos_id);
int config_all(sensors_t *sensors_id, tachos_t *tachos_id, int map_width, int map_height);
void clean_exit(int signum);

#endif
