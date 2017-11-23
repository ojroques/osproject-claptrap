#include <stdlib.h>
#include <math.h>
#include "../src/robot_movement/movements.h"
#include "../src/robot_movement/configuration.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

//Erwan
int main(void){
  turn_left(360);
  return 0;
}
