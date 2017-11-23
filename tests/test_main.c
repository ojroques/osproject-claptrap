#include <stdlib.h>
#include <math.h>
#include "../src/robot_movement/tacho.h"
#include "../src/robot_movement/const.h"
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
  forward(15);
  Sleep (2000);
  backward(15);
  Sleep(2000);
  turn_left(180);
  Sleep (1000);
  turn_rigth(180);
  Sleep (1000);
  return 0;
}
