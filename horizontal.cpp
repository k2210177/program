
/*
   Itolab drone motor control sample

   build
   make

   Usage
   sudo sixad -start &
   sudo ./dronemotor

   Copyright K.ITO,2016

 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <fstream>
#define JOY_DEV "/dev/input/js0"

using namespace std;

#include <unistd.h>
#include "../Navio2/C++/Navio/PWM.h"
#include "../Navio2/C++/Navio/RGBled.h"
#include "../Navio2/C++/Navio/Util.h"
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <sys/time.h>
#include "../Navio2/C++/Navio/MPU9250.h"
#include "../Navio2/C++/Navio/LSM9DS1.h"
#include "AHRS.hpp"

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define FRONT_MOTOR 2
#define REAR_MOTOR 3
#define SERVO_MIN 1.0 /*mS*/
#define SERVO_MAX 2.0 /*mS*/
#define G_SI 9.80665
#define PI   3.14159

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

//============================= Initial setup =================================

void imuSetup()
{
	//----------------------- MPU initialization ------------------------------

	imu->initialize();

	//-------------------------------------------------------------------------

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i < 100; i++)
	{
		imu->update();
		imu->read_gyroscope(&gx, &gy, &gz);

		gx *= 180 / PI;
		gy *= 180 / PI;
		gz *= 180 / PI;

		offset[0] += ( -gx * 0.0175);
		offset[1] += ( -gy * 0.0175);
		offset[2] += ( -gz * 0.0175);
		usleep(10000);
	}
	offset[0] /= 100.0;
	offset[1] /= 100.0;
	offset[2] /= 100.0;

	printf( "Offsets are: %f %f %f\n" ,offset[0] ,offset[1] ,offset[2] );
	ahrs.setGyroOffset( offset[0] , offset[1] , offset[2] );
}


void imuLoop()
{
	//----------------------- Calculate delta time ----------------------------

	gettimeofday( &tv , NULL );
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = ( currenttime - previoustime ) / 1000000.0;
	if( dt < 1/1300.0 )  usleep( ( 1/1300.0 - dt ) * 1000000);
	gettimeofday( &tv , NULL) ;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = ( currenttime - previoustime ) / 1000000.0;

	//-------- Read raw measurements from the MPU and update AHRS --------------

	// Accel + gyro + mag.
	// Soft and hard iron calibration required for proper function.

	   imu->update();
	   imu->read_accelerometer( &ax , &ay , &az );
	   imu->read_gyroscope( &gx , &gy , &gz );
	   imu->read_magnetometer( &mx , &my , &mz );

	   ax /= G_SI;
	   ay /= G_SI;
	   az /= G_SI;
	   gx *= 180 / PI;
	   gy *= 180 / PI;
	   gz *= 180 / PI;

	   ahrs.update( ax , ay , az , gx * 0.0175 , gy * 0.0175 , gz * 0.0175 , my , mx , -mz , dt );

	//------------------------ Read Euler angles ------------------------------

	ahrs.getEuler( &pitch , &roll , &yaw );

	//------------------- Discard the time of the first cycle -----------------

	if ( !isFirst )
	{
		if ( dt > maxdt ) maxdt = dt;
		if ( dt < mindt ) mindt = dt;
	}
	isFirst = 0;

	//------------- Console and network output with a lowered rate ------------

#if 1
	dtsumm += dt;
	if( dtsumm > 0.05 )
	{
		// Console output
		//printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, gz/*yaw * -1*/, dt, int(1/dt));

		// Network output
		//sprintf(sendline,"%10f %10f %10f %10f %dHz\n", ahrs.getW(), ahrs.getX(), ahrs.getY(), ahrs.getZ(), int(1/dt));
		//sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

		dtsumm = 0;
	}
#endif
}

InertialSensor* create_inertial_sensor( char *sensor_name )
{
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

//==================== Start Main function =========================

int main()
{

	char sensor_name[] = "mpu";
	float R , L , F , B;
	int cnt = 0;
	int breakflag = 0;

	//-------- IMU setting -------------------

	imu = create_inertial_sensor( sensor_name );

	if ( !imu ) {
		printf( "Wrong sensor name. Select: mpu or lsm\n" );
		return EXIT_FAILURE;
	}

	if ( !imu->probe() ) {
		printf( "Sensor not enable\n") ;
		return EXIT_FAILURE;
	}

	imuSetup();

	PWM pwm;
	RGBled led;
	js_event js;

	if( !led.initialize() ) return EXIT_FAILURE;

	//-------- PS3 Controller setting --------

	int joy_fd( -1 ) , num_of_axis( 0 ) , num_of_buttons( 0 );
	char name_of_joystick[80];
	vector<char> joy_button;
	vector<int> joy_axis;

	if ( ( joy_fd = open( JOY_DEV, O_RDONLY ) ) < 0 ) {
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

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); // using non-blocking mode

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

	////// Log system setting
	char filename[] = "flightlog.txt";
	char outstr[255];
	std::ofstream fs(filename);

	///// Clock setting
	struct timeval tval;
	unsigned long now_time,past_time,interval;

	gettimeofday( &tval , NULL );
	now_time  = 1000000 * tval.tv_sec + tval.tv_usec;
	past_time = now_time;
	interval  = now_time - past_time;

//==========================  Main Loop ==============================

	while ( true ) {

		led.setColor ( Colors :: Red );

		while ( true ) {

			gettimeofday ( &tval , NULL );
			past_time = now_time;
			now_time  = 1000000 * tval.tv_sec + tval.tv_usec;
			interval  = now_time - past_time;

			imuLoop ();

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {
				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					//printf("%5d\n %5d\n",( int )js.number,js.value);
					break;
			}

			/*
			   Itolab Drone Configuration

			        F
			        |
			        |
			   L----+----R
			        |
			        |
			        B
			*/

			//------------- Stabilize Control -----------------

			R =   roll  * 0.0222 + 1.00; 
			L = - roll  * 0.0222 + 1.00;
			F = - pitch * 0.0222 + 1.00;
			B =   pitch * 0.0222 + 1.00;

			//Limitter
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

			sprintf( outstr , "%lu %lu %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f"
					,now_time ,interval
					,roll ,pitch ,yaw
					,gx ,gy ,gz ,ax ,ay ,az ,mx ,my ,mz
					,R ,L ,F ,B
			       );
			fs << outstr << endl;

			if ( joy_button[3] == 1 ) {
				break;
			}

			do{
				gettimeofday ( &tval , NULL );
				interval = 1000000 * tval.tv_sec + tval.tv_usec - now_time;
			}while( interval < 2000 );

		}

		led.setColor ( Colors :: Blue );

		while ( joy_button[3] == 1 ){

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {
				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					//printf( "%5d\n %5d\n" ,( int )js.number ,js.value );
					break;
			}

		}

		while ( true ){

			read ( joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					//printf( "%5d\n %5d\n" ,( int )js.number ,js.value );
					break;
			}

			if ( joy_button[3] == 1 ) {
				break;
			}
			if ( joy_button[0] == 1 ) {
				breakflag = 1;
				break;
			}
			else breakflag = 0;
		}

		if ( breakflag == 1 ) break;
		else breakflag = 0;

		while ( joy_button[3] == 1 ) {

			read (joy_fd , &js , sizeof ( js_event ) );

			switch ( js.type & ~JS_EVENT_INIT ) {
				case JS_EVENT_AXIS:
					joy_axis[( int )js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[( int )js.number] = js.value;
					break;
			}
		}

	}

	fs.close();

	led.setColor ( Colors :: Green );

	return 0;
}
