#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "const.h"
#include "config.h"
#include "tacho.h"
#include "client.h"
#include "image.h"
#include "ev3.h"
#include "ev3_tacho.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

//Nathan
void search_sensor(uint8_t sensor_type, uint8_t *sensor_id, char *sensor_name) {
    if (ev3_search_sensor(sensor_type, sensor_id, 0)) {
        printf("    [OK] %s found!\n", sensor_name);
    } else {
        printf("    [ERR] %s not found\n", sensor_name);
    }
}

//Erwan
int config_tacho() {
    uint8_t lsn;
    uint8_t rsn;
    printf("Initializing tachos... ");
    for (int i = 0; i < 5 && ev3_tacho_init() < 1; i++) Sleep(1000);
    if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0)) {
        if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &rsn, 0)) {
            set_tacho_stop_action_inx(lsn,TACHO_HOLD);
            set_tacho_stop_action_inx(rsn,TACHO_HOLD);
            printf("Done.\n");
            return 1;
        }
    }
    printf("ERROR.\n");
    return 0;
}

sensors_t config() {
    printf("---------- CLAPTRAP INITIALIZATION ----------\n");
    //Nathan
    sensors_t sensors = {0, 0, 0, 0, 0, 0};
    printf("Initializing the sensors...\n");
    ev3_sensor_init();
    search_sensor(LEGO_EV3_COLOR, &(sensors.color_sensor), "Color sensor");
    search_sensor(LEGO_EV3_GYRO, &(sensors.gyro_sensor), "Gyro sensor");
    search_sensor(LEGO_EV3_US, &(sensors.ultrasonic_sensor), "Ultrasonic sensor");
    search_sensor(HT_NXT_COMPASS, &(sensors.compass_sensor), "Compass sensor");
    search_sensor(LEGO_EV3_TOUCH, &(sensors.touch_sensor), "Touch sensor");
    printf("Done.\n");
    if (!config_tacho()) {
        sensors.is_null = 1;
    }
    init_image();
    if (open_connection() != START_MESSAGE) {
        sensors.is_null = 1;
    }
    printf("----------- END OF INITIALIZATION -----------\n\n");
    return sensors;
}

void clean_exit() {
    printf("Freeing the sensors... ");
    ev3_uninit();
    printf("Done.\n");
}
