#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "const.h"
#include "config.h"
#include "tacho.h"
#include "client.h"
#include "image.h"
#include "ev3.h"
#include "ev3_tacho.h"
#include "ev3_port.h"
#include "ev3_sensor.h"


void search_sensor(uint8_t sensor_type, uint8_t *sensor_id, char *sensor_name) {
    if (ev3_search_sensor(sensor_type, sensor_id, 0)) {
        printf("    [OK] %s found!\n", sensor_name);
    } else {
        printf("    [ERR] %s not found\n", sensor_name);
    }
}

void search_tacho(uint8_t motor_port, uint8_t *tacho_id, char *tacho_name) {
    if (ev3_search_tacho_plugged_in(motor_port, 0, tacho_id, 0)) {
        printf("    [OK] %s found!\n", tacho_name);
    } else {
        printf("    [ERR] %s not found\n", tacho_name);
    }
}

void config_sensors(sensors_t *sensors_id) {
    printf("Initializing sensors...\n");
    ev3_sensor_init();
    search_sensor(LEGO_EV3_COLOR, &(sensors_id->color_sensor), "Color sensor");
    search_sensor(LEGO_EV3_GYRO, &(sensors_id->gyro_sensor), "Gyro sensor");
    search_sensor(LEGO_EV3_US, &(sensors_id->ultrasonic_sensor), "Ultrasonic sensor");
    search_sensor(HT_NXT_COMPASS, &(sensors_id->compass_sensor), "Compass sensor");
    search_sensor(LEGO_EV3_TOUCH, &(sensors_id->touch_sensor), "Touch sensor");
    printf("Done.\n");
}

void config_tacho(tachos_t *tachos_id) {
    printf("Initializing tachos...\n");
    ev3_tacho_init();
    search_tacho(RIGHT_WHEEL_PORT, &(tachos_id->right_wheel), "Right wheel");
    search_tacho(LEFT_WHEEL_PORT, &(tachos_id->left_wheel), "Left wheel");
    search_tacho(ULTRASONIC_TACHO_PORT, &(tachos_id->ultrasonic_tacho), "Ultrasonic tacho");
    search_tacho(CARRIER_PORT, &(tachos_id->obstacle_carrier), "Obstacle carrier");
    printf("Done.\n");
}

int config_all(sensors_t *sensors_id, tachos_t *tachos_id) {
    int is_ok;
    printf("---------- CLAPTRAP INITIALIZATION ----------\n");

    config_sensors(sensors_id);
    config_tacho(tachos_id);

    printf("Initializing image... ");
    init_image();
    printf("Done.\n");

    printf("Establishing connection... ");
    is_ok = open_connection();
    if (is_ok) {
        printf("Done.\n");
    } else {
        printf("Error!\n");
    }

    printf("----------- END OF INITIALIZATION -----------\n\n");

    return is_ok;
}

void clean_exit(int signum) {
    uint8_t right_wheel, left_wheel, us_tacho, obst_carrier;

    ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &right_wheel, 0);
    ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &left_wheel, 0);
    ev3_search_tacho_plugged_in(ULTRASONIC_TACHO_PORT, 0, &us_tacho, 0);
    ev3_search_tacho_plugged_in(CARRIER_PORT, 0, &obst_carrier, 0);

    printf("\n");
    if (signum) printf("Signal %d detected!\n", signum);

    printf("Freeing the sensors... ");
    ev3_uninit();
    printf("Done.\n");

    printf("Freeing the tachos... ");
    set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
    set_tacho_stop_action_inx(left_wheel, TACHO_COAST);
    set_tacho_stop_action_inx(us_tacho, TACHO_COAST);
    set_tacho_stop_action_inx(obst_carrier, TACHO_COAST);
    stop_tacho(right_wheel);
    stop_tacho(left_wheel);
    stop_tacho(us_tacho);
    stop_tacho(obst_carrier);
    printf("Done.\n");

    printf("See you later!\n");
    exit(EXIT_SUCCESS);
}
