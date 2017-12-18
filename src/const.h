#ifndef CONST_H
#define CONST_H

#include <unistd.h>

/* GENERAL */
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define NB_SENSOR_MESURE  8    // Number of mesures to average

#endif
