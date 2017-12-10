#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "../src/sensors.h"
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


//Function to check if the touch sensor is pressed
static bool _check_pressed( uint8_t sn )
{
	int val;

	if ( sn == SENSOR__NONE_ ) {
		return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
	}
	return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}


//#########MAIN#########

int main(int argc, char* argv[])
{
	if (argc != 3){
		printf("Wrong number of argument, exiting ! \n");
		return (0);
	}

//SOME MORE INITIALISATION
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

// TOUCH INITIALISATION
	uint8_t sn_touch;
	ev3_sensor_init();

	if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )) {
		printf( "TOUCH sensor is found, press BUTTON for EXIT...\n" );
	}

//###################FIN INITIALIZATON#########################################
printf( "*** ( EV3 ) Hello! ***\n" );

	uint8_t sn_tested;
	char s[ 256 ];

//take the parameter on the command line
// first paramet = sensor tested
//{color = 1, gyro = 2, ultrasonic = 3, compass = 4}
//
int tested_sensor = atoi(argv[1]);
int tested_mode = atoi(argv[2]);

switch (tested_sensor){
	case 1 :
		if ( ev3_search_sensor( LEGO_EV3_COLOR, &sn_tested, 0 )) {
				printf("COLOR sensor is found\n" );
				printf("sn number for color sensor is %d \n", sn_tested);
        printf("    Port = %s\n", ev3_sensor_port_name(sn_tested, s ));
		}
		break;

	case 2 :
		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_tested, 0 )) {
				printf("GYRO sensor is found\n" );
				printf("sn number for gyro sensor is %d \n", sn_tested);
				printf("    Port = %s\n", ev3_sensor_port_name(sn_tested, s ));
		}
		break;

	case 3 :
		if ( ev3_search_sensor( LEGO_EV3_US, &sn_tested, 0 )) {
				printf("ULTRAONIC sensor is found\n" );
				printf("sn number for ultrasonic sensor is %d \n", sn_tested);
				printf("    Port = %s\n", ev3_sensor_port_name(sn_tested, s ));
		}
		break;

	case 4 :
		if ( ev3_search_sensor( LEGO_EV3_US, &sn_tested, 0 )) {
				printf("COMPASS sensor is found\n" );
				printf("sn number for compass sensor is %d \n", sn_tested);
				printf("    Port = %s\n", ev3_sensor_port_name(sn_tested, s ));
		}
		break;

	default :
		printf("Input not recognized , exiting.\n");
		return ( 0 );
}


switch (tested_mode){
	//########		color		##########################################
	//################GET COLOR###################################
	case 11 :
		printf("Test de get_color \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_COLOR_COL_COLOR);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		int color_value;
		for ( ; ; ){

			//get sensor value
			color_value = get_color(sn_tested);
			//print values
			printf( "\r color = %s \n", color[color_value]);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;
	//################GET REFLECTION###################################
	case 12 :
		printf("Test de get_reflection \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_COLOR_COL_REFLECT);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		int reflection_value;
		for ( ; ; ){

			//get sensor value
			reflection_value = get_reflection(sn_tested);
			//print values
			printf( "\r reflect = %d \n", reflection_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;

	//################GET AMBIENT###################################
	case 13 :
		printf("Test de get_ambient \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_COLOR_COL_AMBIENT);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

			//test du capteur !
		int ambient_value;
		for ( ; ; ){

			//get sensor value
			ambient_value = get_ambient(sn_tested);
			//print values
			printf( "\r ambiant = %d \n", ambient_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;

	//################GET RAW REFLECTED###################################
	case 14 :
		printf("Test de get_raw_reflected \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_COLOR_REF_RAW);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		DoubleValue raw_reflection_value;
		for ( ; ; ){

			//get sensor value
			raw_reflection_value = get_raw_reflected(sn_tested);
			//print values
			printf( "\r raw reflected 1 = %d , raw reflected 2 = %d\n", raw_reflection_value.value0, raw_reflection_value.value1);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}
		break;

	//################GET RAW RGB###################################
	case 15 :
		printf("Test de get_raw_reflected \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_COLOR_RGB_RAW);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		TripleValue raw_rgb_value;
		for ( ; ; ){

			//get sensor value
			raw_rgb_value = get_raw_rgb(sn_tested);
			//print values
			printf( "\r raw rgb 1 = %d , raw rgb 2 = %d , raw rgb 3 = %d\n", raw_rgb_value.value0, raw_rgb_value.value1, raw_rgb_value.value2);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}
		break;

  //########		Gyro	##########################################
	//################GET ANGLE###################################
	case 21 :
		printf("Test de get_angle \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_GYRO_GYRO_ANG);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

			//test du capteur !
		int angle_value;
		for ( ; ; ){

			//get sensor value
			angle_value = get_angle(sn_tested);
			//print values
			printf( "\r angle = %d \n", angle_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;

//################GET ROT SPEED###################################
	case 22 :
		printf("Test de get_rot_speed \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_GYRO_GYRO_RATE);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		int rot_speed_value;
		for ( ; ; ){

			//get sensor value
			rot_speed_value = get_rot_speed(sn_tested);
			//print values
			printf( "\r rot_speed_value = %d \n", rot_speed_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;

	//################GET RAW GYRO###################################
	case 23 :
		printf("Test de get_raw_gyro \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_GYRO_GYRO_FAS);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
			}

		//test du capteur !
		int raw_gyro_value;
		for ( ; ; ){

			//get sensor value
			raw_gyro_value = get_raw_gyro(sn_tested);
			//print values
			printf( "\r raw_gyro_value = %d \n", raw_gyro_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
			Sleep( 200 );
			printf( "\r        " );
			fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
			Sleep( 200 );
			}

			break;

//################GET ANGLE AND ROT SPEED###################################
	case 24 :
		printf("Test de get_angle_and_rot_speed \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_GYRO_GYRO_G_AND_A);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
		}

		//test du capteur !
		DoubleValue angle_rot_speed_value;
		for ( ; ; ){

			//get sensor value
			angle_rot_speed_value = get_angle_and_rot_speed(sn_tested);
			//print values
			printf( "\r angle = %d, rot_speed = %d \n", angle_rot_speed_value.value0, angle_rot_speed_value.value1);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

		break;

//########		Ultrasonic	##########################################
//################GET DISTANCE###################################
	case 31 :
		printf("Test de get_distance \n");
		set_sensor_mode_inx(sn_tested, LEGO_EV3_US_US_DIST_CM);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
			}

		//test du capteur !
		int distance_value;
		for ( ; ; ){

			//get sensor value
			distance_value = get_distance(sn_tested);
			//print values
			printf( "\r distance_value = %d \n", distance_value);
			fflush( stdout );

			if ( _check_pressed( sn_touch )) break;
			Sleep( 200 );
			printf( "\r        " );
			fflush( stdout );
			if ( _check_pressed( sn_touch )) break;
	 		Sleep( 200 );
			}

		break;

	//################GET SINGLE DISTANCE###################################
		case 32 :
			printf("Test de get_single_dist \n");
			set_sensor_mode_inx(sn_tested, LEGO_EV3_US_US_SI_CM);
			if (get_sensor_mode(sn_tested, s, sizeof(s))) {
				printf("    Mode = %s\n", s);
				}

			//test du capteur !
			int single_distance_value;
			for ( ; ; ){

				//get sensor value
				single_distance_value = get_single_dist(sn_tested);
				//print values
				printf( "\r single_distance_value = %d \n", single_distance_value);
				fflush( stdout );

				if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
				if ( _check_pressed( sn_touch )) break;
		 		Sleep( 200 );
				}

			break;

//################GET CONTINUOUS DISTANCE###################################
		case 33 :
			printf("Test de get_single_dist \n");
			set_sensor_mode_inx(sn_tested, LEGO_EV3_US_US_DC_CM);
			if (get_sensor_mode(sn_tested, s, sizeof(s))) {
				printf("    Mode = %s\n", s);
				}

			//test du capteur !
			int continuous_distance_value;
			for ( ; ; ){

				//get sensor value
				continuous_distance_value = get_continuous_distance(sn_tested);
				//print values
				printf( "\r continuous_distance_value = %d \n", continuous_distance_value);
				fflush( stdout );

				if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
				if ( _check_pressed( sn_touch )) break;
		 		Sleep( 200 );
				}

			break;

//########		compass	##########################################
//################GET COMPASS DIRECTION###################################
	case 41 :
		printf("Test de get_compass_direction \n");
		set_sensor_mode_inx(sn_tested, HT_NXT_COMPASS_COMPASS);
		if (get_sensor_mode(sn_tested, s, sizeof(s))) {
			printf("    Mode = %s\n", s);
			}

			//test du capteur !
			int compass_direction_value;
			for ( ; ; ){

				//get sensor value
				compass_direction_value = get_compass_direction(sn_tested);
				//print values
				printf( "\r compass_direction_value = %d \n", compass_direction_value);
				fflush( stdout );

				if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
				printf( "\r        " );
				fflush( stdout );
				if ( _check_pressed( sn_touch )) break;
				Sleep( 200 );
			}

			break;


//################DEFAULT###################################
	default :
		printf("Input not recognized , exiting.\n");
		return ( 0 );

//##############END OF SWITCH###############################
}

	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
	return ( 0 );
}
