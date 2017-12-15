#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include "../src/tacho.h"
#include "../src/const.h"
#include "../src/position.h"
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

coordinate_t coordinate = {0,0,90,PTHREAD_MUTEX_INITIALIZER};

//Erwan
int main(void){

  pthread_t pos_thread;

  if(pthread_create(&pos_thread, NULL, position_thread, NULL) == -1) {

    return EXIT_FAILURE;

    }

  forward(15);
  Sleep (3000);
  backward(15);
  Sleep(3000);
  turn_left(180);
  Sleep (2000);
  turn_right(180);
  Sleep (2000);
  return 0;
}
