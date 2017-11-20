#include "capteurs.h"
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

void set_mode_gyro(int mode){
  int8_t sn_mag;
  if (ev3_search_sensor(NXT_ANALOG, &sn_mag,0)){

    switch (mode){

      case 0 :
        set_sensor_mode(sn_mag, GYRO-ANG);
        break;

      case 1 :
        set_sensor_mode(sn_mag, GYRO-RATE);
        break;

      case 2 :
        set_sensor_mode(sn_mag, GYRO-FAS);
        break;

      case 3 :
        set_sensor_mode(sn_mag, GYRO-G&A);
        break;

      case 4 :
        set_sensor_mode(sn_mag, GYRO-CAL);
        break;
    }
  }
  else{
    printf("error sensor not found");
    exit(1);
  }
}

float gyro_value0(){
  uint8_t sn_mag;
  float value;
  if (ev3_search_sensor(NXT_ANALOG, &sn_mag,0)){
    if ( !get_sensor_value0(sn_mag, &value )) {
				value = 0;
			}
    return value;
  }
  printf("error sensor not found");
  exit(1);
}
