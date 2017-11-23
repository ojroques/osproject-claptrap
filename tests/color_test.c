#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "sensors.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

static bool _check_pressed( uint8_t sn )
{
	int val;

	if ( sn == SENSOR__NONE_ ) {
		return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
	}
	return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}


int main( void )
{
	uint8_t sn_touch;
	uint8_t sn_color;
	char s[ 256 ];
	int val;

#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";

#endif
	if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
	printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
	printf( "Waiting tacho is plugged...\n" );

#endif

	printf( "*** ( EV3 ) Hello! ***\n" );

//Run all sensors
	ev3_sensor_init();

	if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )) {
		printf( "TOUCH sensor is found, press BUTTON for EXIT...\n" );
	}
    if ( ev3_search_sensor( LEGO_EV3_COLOR, &sn_color, 0 )) {

				printf("COLOR sensor is found\n" );
				printf("sn number for color sensor is %d \n", sn_color);
        printf("    Port = %s\n", ev3_sensor_port_name(sn_color, s ));
        set_sensor_mode_inx(sn_color, LEGO_EV3_COLOR_COL_AMBIENT);
        if (get_sensor_mode(sn_color, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

    	for ( ; ; ){
            /*printf("    Reading COLOR...\n");*/
			//if ( !get_sensor_value( 0, sn_color, &val ) || ( val < 0 ) ) {
			//	val = 0;
			//}
			val = get_color(sn_color);
			printf( "\r(%d, %s) \n", val, color[val]);
			//printf( "\r%d \n", val);
			fflush( stdout );

    		if ( _check_pressed( sn_touch )) break;
    		Sleep( 200 );
    		printf( "\r        " );
    		fflush( stdout );
    		if ( _check_pressed( sn_touch )) break;
    		Sleep( 200 );
    	}
    }

	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	return ( 0 );
}
