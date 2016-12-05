// 12月5日（月）改訂
// メインプログラム

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/joystick.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../Navio2/C++/Navio/MPU9250.h"
#include "../Navio2/C++/Navio/LSM9DS1.h"
#include "../Navio2/C++/Navio/PWM.h"
#include "../Navio2/C++/Navio/RGBled.h"
#include "../Navio2/C++/Navio/Util.h"
#include "AHRS.hpp"

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define FRONT_MOTOR 2
#define REAR_MOTOR 3
#define SERVO_MIN 1.0 /*ms*/
#define SERVO_MAX 2.0 /*ms*/
#define G_SI 9.80665
#define PI   3.14159
#define JOY_DEV "/dev/input/js0"

using namespace std;

// Objects

InertialSensor *imu;
AHRS    ahrs;   // Mahony AHRS

// Sensor data

float ax , ay , az;
float gx , gy , gz;
float mx , my , mz;

// Orientation data

float roll, pitch, yaw;

// Timing data

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

void imuSetup ( void ) {

	//MPU initialization

	imu->initialize();

	printf( "Beginning Gyro calibration...\n" );

	for( int i = 0 ; i < 100; i++ ) {

		imu->update();
		imu->read_gyroscope( &gx , &gy , &gz);

		gx *= 180 / PI;
		gy *= 180 / PI;
		gz *= 180 / PI;

		offset[0] += ( -gx * 0.0175 );
		offset[1] += ( -gy * 0.0175 );
		offset[2] += ( -gz * 0.0175 );

		usleep(10000);

	}

	offset[0] /= 100.0;
	offset[1] /= 100.0;
	offset[2] /= 100.0;

	printf( "Offsets are: %f %f %f\n" ,offset[0] ,offset[1] ,offset[2] );
	ahrs.setGyroOffset( offset[0] , offset[1] , offset[2] );
}

void imuLoop ( void ) {

	//Calculate delta time

	gettimeofday( &tv , NULL );
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = ( currenttime - previoustime ) / 1000000.0;
	if( dt < 1/1300.0 )  usleep( ( 1/1300.0 - dt ) * 1000000);
	gettimeofday( &tv , NULL) ;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = ( currenttime - previoustime ) / 1000000.0;

	//Read raw measurements from the MPU and update AHRS
	// Accel + gyro + mag.
	// Soft and hard iron calibration required for proper function.

	imu -> update();
	imu -> read_accelerometer( &ax , &ay , &az );
	imu -> read_gyroscope( &gx , &gy , &gz );
	imu -> read_magnetometer( &mx , &my , &mz );

	ax /= G_SI;
	ay /= G_SI;
	az /= G_SI;
	gx *= 180 / PI;
	gy *= 180 / PI;
	gz *= 180 / PI;

	ahrs.update( ax , ay , az , gx * 0.0175 , gy * 0.0175 , gz * 0.0175 , my , mx , -mz , dt );

	//Read Euler angles

	ahrs.getEuler( &pitch , &roll , &yaw );

	//Discard the time of the first cycle

	if ( !isFirst )	{

		if ( dt > maxdt ) maxdt = dt;
		if ( dt < mindt ) mindt = dt;

	}
	isFirst = 0;

	//Console and network output with a lowered rate

	dtsumm += dt;

	if( dtsumm > 0.05 ) {

		//Console output
		//printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, gz/*yaw * -1*/, dt, int(1/dt));

		// Network output
		//sprintf( sendline , "%10f %10f %10f %10f %dHz\n" ,ahrs.getW() ,ahrs.getX() ,ahrs.getY() ,ahrs.getZ() ,int( 1/dt ) );
		//sendto( sockfd , sendline , strlen( sendline ) , 0 ,  ( struct sockaddr * )&servaddr , sizeof( servaddr ) );

		dtsumm = 0;

	}

}

InertialSensor* create_inertial_sensor ( char *sensor_name ) {

	InertialSensor *imu;

	if ( !strcmp( sensor_name , "mpu" ) ) {

		printf( "Selected: MPU9250\n" );
		imu = new MPU9250();

	}
	else if ( !strcmp( sensor_name , "lsm" ) ) {

		printf( "Selected: LSM9DS1\n" );
		imu = new LSM9DS1();

	}
	else {

		return NULL;

	}

	return imu;

}

int main ( void ) {

	//DualShock3 setting

	int joy_fd( -1 ) , num_of_axis( 0 ) , num_of_buttons( 0 );
	char name_of_joystick[80];
	vector<char> joy_button;
	vector<int> joy_axis;
	js_event js;

	if ( ( joy_fd = open( JOY_DEV ,O_RDONLY ) ) < 0 ) {
		printf( "Failed to open %s" ,JOY_DEV );
		cerr << "Failed to open " << JOY_DEV << endl;
		return -1;
	}

	ioctl( joy_fd , JSIOCGAXES , &num_of_axis );
	ioctl( joy_fd , JSIOCGBUTTONS , &num_of_buttons );
	ioctl( joy_fd , JSIOCGNAME(80) , &name_of_joystick );

	joy_button.resize( num_of_buttons , 0 );
	joy_axis.resize( num_of_axis , 0 );

	printf( "Joystick: %s axis: %d buttons: %d\n" ,name_of_joystick ,num_of_axis ,num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );

	//IMU setting

	char sensor_name[] = "mpu";

	imu = create_inertial_sensor( sensor_name );

	if ( !imu ) {
		printf( "Wrong sensor name. Select: mpu or lsm\n" );
		return EXIT_FAILURE;
	}

	if ( !imu -> probe() ) {
		printf( "Sensor not enable\n") ;
		return EXIT_FAILURE;
	}

	imuSetup();

	PWM pwm;
	RGBled led;

	if( !led.initialize() ) return EXIT_FAILURE;

	if ( check_apm() ) {
		return 1;
	}

	for ( int i = 0 ; i < 10000 ; i++ ) {
		imuLoop();
		usleep( 1000 );
		if( i % 1000 == 0 ) {
			printf( "#" );
			fflush( stdout );
		}
	}

	for ( int i = 0 ; i < 4 ; i++ ) {
		if ( !pwm.init( i ) ) {
			fprintf( stderr , "Output Enable not set. Are you root?\n" );
			return 0;
		}
		pwm.enable( i );
		pwm.set_period( i , 500 );
	}
	printf( "\nReady to Fly !\n" );

	//flightlog setting
	char filename[] = "flightlog.txt";
	char outstr[255];
	std::ofstream fs(filename);

	//clock setting
	struct timeval tval;
	unsigned long now_time,past_time,interval;

	gettimeofday( &tval , NULL );
	now_time  = 1000000 * tval.tv_sec + tval.tv_usec;
	past_time = now_time;
	interval  = now_time - past_time;

	short PauseFlag = 1 , EndFlag = 0;
	float StickRx , StickRy , StickLx , StickLy;
	float throttle;
	float rroll , rpitch , ryaw;
	float roll_rad , pitch_rad , yaw_rad;
	float R = 1.0 , L = 1.0 , F = 1.0 , B = 1.0;

	pwm.set_duty_cycle ( RIGHT_MOTOR , R );
	pwm.set_duty_cycle ( LEFT_MOTOR  , L );
	pwm.set_duty_cycle ( FRONT_MOTOR , F );
	pwm.set_duty_cycle ( REAR_MOTOR  , B );

	//main loop

	while ( EndFlag == 0 ) {

		led.setColor ( Colors :: Red );

		while ( PauseFlag == 0 ) {

			gettimeofday ( &tval , NULL );
			past_time = now_time;
			now_time  = 1000000 * tval.tv_sec + tval.tv_usec;
			interval  = now_time - past_time;

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {

				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;

				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					if( js.value == 1 ) {
						if ( joy_button[3] == 1 ) {
							PauseFlag = 1;
							printf ( "PAUSE\n" );
						}
					}
					break;

			}

			StickRx =  joy_axis[2] / 23170.0;
			StickRy = -joy_axis[3] / 23170.0;
			StickLx =  joy_axis[0] / 23170.0;
			StickLy =  joy_axis[1] / 23170.0;

			if ( StickRx >  1.0 ) StickRx =  1.0;
			if ( StickRx < -1.0 ) StickRx = -1.0;
			if ( StickRy >  1.0 ) StickRy =  1.0;
			if ( StickRy < -1.0 ) StickRy = -1.0;
			if ( StickLx >  1.0 ) StickLx =  1.0;
			if ( StickLx < -1.0 ) StickLx = -1.0;
			if ( StickLy >  1.0 ) StickLy =  1.0;
			if ( StickLy < -1.0 ) StickLy = -1.0;

			throttle = StickRy;
			rpitch   = StickLy;
			rroll    = StickLx;
			ryaw     = StickRx;

			imuLoop ();

			roll_rad  = roll  * PI / 180;
			pitch_rad = pitch * PI / 180;
			yaw_rad   = yaw   * PI / 180;

			//regulator
			R =   0.858 * ax * 0.001 + 0.011 * ay * 0.001 - 1.919 * az * 0.001 + 7.816 * roll_rad + 0.028 * pitch_rad - 5.042 * yaw_rad;
			L = - 0.696 * ax * 0.001 + 0.013 * ay * 0.001 - 2.414 * az * 0.001 - 6.238 * roll_rad + 0.035 * pitch_rad - 6.295 * yaw_rad;
			F =   0.007 * ax * 0.001 - 0.749 * ay * 0.001 + 1.687 * az * 0.001 + 0.017 * roll_rad - 6.664 * pitch_rad + 4.385 * yaw_rad;
			B =   0.006 * ax * 0.001 + 0.824 * ay * 0.001 + 1.506 * az * 0.001 + 0.016 * roll_rad + 7.456 * pitch_rad + 3.967 * yaw_rad;

			R = R + 0.128 + throttle - rroll  + yaw;
			L = L + 0.013 + throttle + rroll  + yaw;
			F = F + 0.013 + throttle + rpitch - yaw;
			B = B + 0.128 + throttle - rpitch - yaw;

			//limitter
			if ( R > 2.0 ) R = 2.0;
			if ( R < 1.0 ) R = 1.0;
			if ( L > 2.0 ) L = 2.0;
			if ( L < 1.0 ) L = 1.0;
			if ( F > 2.0 ) F = 2.0;
			if ( F < 1.0 ) F = 1.0;
			if ( B > 2.0 ) B = 2.0;
			if ( B < 1.0 ) B = 1.0;

			pwm.set_duty_cycle ( RIGHT_MOTOR , R );
			pwm.set_duty_cycle ( LEFT_MOTOR  , L );
			pwm.set_duty_cycle ( FRONT_MOTOR , F );
			pwm.set_duty_cycle ( REAR_MOTOR  , B );

			sprintf( outstr , "%lu %lu %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
					,now_time ,interval
					,gx ,gy ,gz ,ax ,ay ,az ,mx ,my ,mz
					,roll ,pitch ,yaw
					,R ,L ,F ,B
			       );
			fs << outstr << endl;

			do{
				gettimeofday ( &tval , NULL );
				interval = 1000000 * tval.tv_sec + tval.tv_usec - now_time;
			}while( interval < 2000 );

		}

		R = 1.0;
		L = 1.0;
		F = 1.0;
		B = 1.0;

		pwm.set_duty_cycle ( RIGHT_MOTOR , R );
		pwm.set_duty_cycle ( LEFT_MOTOR  , L );
		pwm.set_duty_cycle ( FRONT_MOTOR , F );
		pwm.set_duty_cycle ( REAR_MOTOR  , B );

		led.setColor ( Colors :: Blue );

		usleep ( 1000 );

		while ( PauseFlag == 1 ) {

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {

				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;

				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					if( js.value == 1 ) {
						if ( joy_button[3] == 1 ) {
							PauseFlag = 0;
							printf ( "RESTART\n" );
						}
						if ( joy_button[0] == 1 ) {
							PauseFlag = 0;
							EndFlag = 1;
							printf ( "END\n" );
						}
					}
					break;

			}
	
		}

		usleep ( 1000 );

	}

	fs.close();

	led.setColor ( Colors :: Green );

	return 0;

}
