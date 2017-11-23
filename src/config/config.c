#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "../robot_movement/configuration.h"
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
void config(){

  uint8_t lsn;
  uint8_t rsn;
  while (ev3_tacho_init() < 1) Sleep(1000);
  if (ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT,0, &lsn, 0 )){
    if (ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT,0,&rsn,0 )){
      set_tacho_stop_action_inx(lsn,TACHO_HOLD);
      set_tacho_stop_action_inx(rsn,TACHO_HOLD);
    }
  }
}
