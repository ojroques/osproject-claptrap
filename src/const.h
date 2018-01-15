#ifndef CONST_H
#define CONST_H

#include <unistd.h>

/* GENERAL */
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define NB_SENSOR_MESURE                   4    // Number of mesures to average
#define TRESHOLD_COLOR                    30    // Dist in mm to detect color reliably
#define TRESHOLD_MANEUVER                150    // Dist in mm to maneuver correctly
#define TRESHOLD_CHECK_OBST              250    // Dist in mm to go check the obst type
#define TRESHOLD_SIDE                     70    // Dist in mm to allow rotation
#define SONAR_PRECISION_THRESHOLD       1800    // Dist max at which we consider the sonar is precise
#define THRESHOLD_ULTRASONIC_TACHO_SUP   100    // Max angle of rotation of the tacho rotating the head of the robot
#define THRESHOLD_ULTRASONIC_TACHO_INF  -100    // Min angle of rotation of the tacho rotating the head of the robot
#define NAV_ANGLE_RANGE                  110    // Min/max angle for the scan
#define TRESHOLD_MANEUVER_SIDE            60    // Min/max angle for the scan
#define NAV_DIST_RANGE                     5    // Min/max angle for the scan
#define COUNT_THRESHOLD                   30    // Max number of loop before considering the angle is the same and the robot is stuck
#define TURNING_TIME                      12    // Max number of seconds allowed for the robot to turn
#endif
