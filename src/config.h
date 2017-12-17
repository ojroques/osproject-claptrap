#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

//Nathan
//Struct with the id of each sensors
typedef struct _sensors_t {
    int is_null;
    uint8_t color_sensor;
    uint8_t gyro_sensor;
    uint8_t ultrasonic_sensor;
    uint8_t compass_sensor;
    uint8_t touch_sensor;
} sensors_t;

void search_sensor(uint8_t sensor_type, uint8_t *sensor_id, char *sensor_name);
int config_tacho(uint8_t sonar_id);
sensors_t config();
void clean_exit();

#endif
