// 12月11日 作成
// 構成済みセンサ値確認プログラム

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/joystick.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../../Navio2/C++/Navio/MPU9250.h"
#include "../../Navio2/C++/Navio/LSM9DS1.h"
#include "../../Navio2/C++/Navio/PWM.h"
#include "../../Navio2/C++/Navio/RGBled.h"
#include "../../Navio2/C++/Navio/Util.h"
#include "../AHRS.hpp"

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define FRONT_MOTOR 2
#define REAR_MOTOR 3
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

float cax = - 1.003202 , cay = - 1.006379 , caz = - 0.977109;
float cmx = 73.389450 , cmy = 0.753407 , cmz = 7.400240;

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

		offset[0] += ( -gx );
		offset[1] += ( -gy );
		offset[2] += ( -gz );

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
	imu -> read_accelerometer( &ay , &ax , &az );
	imu -> read_gyroscope( &gy , &gx , &gz );
	imu -> read_magnetometer( &mx , &my , &mz );

	ax /= G_SI;
	ay /= G_SI;
	az /= G_SI;

	ax = - ax;
	ay = - ay;

	gz = - gz;

	ax *= cax;
	ay *= cay;
	az *= caz;

	mx -= cmx;
	my -= cmy;
	mz -= cmz;

	ahrs.update( ax , ay , az , gx , gy , gz , mx , my , mz , dt );

	//Read Euler angles

	ahrs.getEuler( &roll , &pitch , &yaw );

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

	//main loop

	short c = 0 , PauseFlag = 1 , EndFlag = 0;
	float min = 1.0 , max = 2.0;
	float cyaw;
	float SRx , SRy , SLx , SLy;
	float throttle;
	float rroll , rpitch , ryaw;
	float R , L , F , B;
	float dR , dL , dF , dB;
	float Ra = 4.0857 , Rb = - 4.6865382;
	float La = 3.5419 , Lb = - 4.0627676;
	float Fa = 3.5419 , Fb = - 4.0627676;
	float Ba = 3.7375 , Bb = - 3.7608000;

	pwm.set_duty_cycle ( RIGHT_MOTOR , min );
	pwm.set_duty_cycle ( LEFT_MOTOR  , min );
	pwm.set_duty_cycle ( FRONT_MOTOR , min );
	pwm.set_duty_cycle ( REAR_MOTOR  , min );

	while ( EndFlag == 0 ) {

		led.setColor ( Colors :: Red );

		imuLoop();

		cyaw = yaw;

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
					if ( js.value == 0 ) {
						c = 0;
					}
					if ( js.value == 1 && c == 0 ) {
						c = 1;
						if ( joy_button[3] == 1 ) {
							PauseFlag = 1;
							printf ( "PAUSE\n" );
						}
					}
					break;

			}

			SRx =  joy_axis[2] / 23170.0;
			SRy = -joy_axis[3] / 23170.0;
			SLx =  joy_axis[0] / 23170.0;
			SLy =  joy_axis[1] / 23170.0;

			if ( SRx >  1.0 ) SRx =  1.0;
			if ( SRx < -1.0 ) SRx = -1.0;
			if ( SRy >  1.0 ) SRy =  1.0;
			if ( SRy <  0.0 ) SRy =  0.0;
			if ( SLx >  1.0 ) SLx =  1.0;
			if ( SLx < -1.0 ) SLx = -1.0;
			if ( SLy >  1.0 ) SLy =  1.0;
			if ( SLy < -1.0 ) SLy = -1.0;

			throttle = SRy * 7.0 / 4.0;
/*			rroll    = SLx;
			rpitch   = SLy;
			ryaw     = SRx;
*/
			dR = ( throttle - Rb ) / Ra;
			dL = ( throttle - Lb ) / La;
			dF = ( throttle - Fb ) / Fa;
			dB = ( throttle - Bb ) / Ba;

			//limitter
			if ( dR > max ) dR = max;
			if ( dR < min ) dR = min;
			if ( dL > max ) dL = max;
			if ( dL < min ) dL = min;
			if ( dF > max ) dF = max;
			if ( dF < min ) dF = min;
			if ( dB > max ) dB = max;
			if ( dB < min ) dB = min;

			imuLoop ();

			yaw -= cyaw;

			//regulator
			R =   0.8628886 * gy * 0.001 + 0.0042907 * gx * 0.001 - 0.7707670 * gz * 0.001 + 7.8315342 * roll + 0.0059907 * pitch - 0.5017307 * yaw;
			L = - 0.6905217 * gy * 0.001 + 0.0054213 * gx * 0.001 - 0.9708275 * gz * 0.001 - 6.2182832 * roll + 0.0075532 * pitch - 0.6314131 * yaw;
			F =   0.0027362 * gy * 0.001 - 0.7430635 * gx * 0.001 + 0.6788208 * gz * 0.001 + 0.0038083 * roll - 6.6448231 * pitch + 0.4413308 * yaw;
			B =   0.0024225 * gy * 0.001 + 0.8292352 * gx * 0.001 + 0.6042195 * gz * 0.001 + 0.0033931 * roll + 7.4730337 * pitch + 0.3934600 * yaw;

			R = dR + R * 1.000;
			L = dL + L * 1.000;
			F = dF + F * 1.000;
			B = dB + B * 1.000;

			//limitter
			if ( R > max ) R = max;
			if ( R < min ) R = min;
			if ( L > max ) L = max;
			if ( L < min ) L = min;
			if ( F > max ) F = max;
			if ( F < min ) F = min;
			if ( B > max ) B = max;
			if ( B < min ) B = min;

/*			pwm.set_duty_cycle ( RIGHT_MOTOR , R );
			pwm.set_duty_cycle ( LEFT_MOTOR  , L );
			pwm.set_duty_cycle ( FRONT_MOTOR , F );
			pwm.set_duty_cycle ( REAR_MOTOR  , B );
*/
			pwm.set_duty_cycle ( RIGHT_MOTOR , min );
			pwm.set_duty_cycle ( LEFT_MOTOR  , min );
			pwm.set_duty_cycle ( FRONT_MOTOR , min );
			pwm.set_duty_cycle ( REAR_MOTOR  , min );

			printf ( "roll=%f pitch=%f yaw=%f cyaw=%f R=%f L=%f F=%f B=%f throttle=%f\n" ,roll ,pitch ,yaw ,cyaw ,R ,L ,F ,B ,throttle );

			sprintf( outstr , "%lu %lu %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f"
					,now_time ,interval
					,ax ,ay ,az ,gx ,gy ,gz ,mx ,my ,mz
					,roll ,pitch ,yaw
					,R ,L ,F ,B
			       );
			fs << outstr << endl;

			do{
				gettimeofday ( &tval , NULL );
				interval = 1000000 * tval.tv_sec + tval.tv_usec - now_time;
			}while( interval < 2000 );

		}

		pwm.set_duty_cycle ( RIGHT_MOTOR , min );
		pwm.set_duty_cycle ( LEFT_MOTOR  , min );
		pwm.set_duty_cycle ( FRONT_MOTOR , min );
		pwm.set_duty_cycle ( REAR_MOTOR  , min );

		led.setColor ( Colors :: Blue );

		while ( PauseFlag == 1 ) {

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {

				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;

				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					if ( js.value == 0 ) {
						c = 0;
					}
					if ( js.value == 1 && c == 0 ) {
						c = 1;
						if ( joy_button[3] == 1 ) {
							PauseFlag = 0;
							printf ( "START\n" );
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

	}

	fs.close();

	led.setColor ( Colors :: Green );

	return 0;

}
