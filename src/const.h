#ifndef CONST_H
#define CONST_H

#include <unistd.h>

/* GENERAL */
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define NB_SENSOR_MESURE       8    // Number of mesures to average
#define TRESHOLD_COLOR        30    // Dist in mm to detect color reliably
#define TRESHOLD_MANEUVER    150    // Dist in mm to maneuver correctly
#define TRESHOLD_CHECK_OBST  200    // Dist in mm to go check the obst type

#endif
