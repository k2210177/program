// 12月10日 作成
// 加速度センサ較正プログラム

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
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../../Navio2/C++/Navio/MPU9250.h"
#include "../../Navio2/C++/Navio/LSM9DS1.h"
#include "../AHRS.hpp"

#define G_SI 9.80665
#define PI   3.14159

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
	imu -> read_accelerometer( &ax , &ay , &az );
	imu -> read_gyroscope( &gx , &gy , &gz );
	imu -> read_magnetometer( &mx , &my , &mz );

	ax /= G_SI;
	ay /= G_SI;
	az /= G_SI;

	ahrs.update( ax , ay , az , gx , gy , gz , mx , my , mz , dt );

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

	//flightlog setting
	char filename[] = "accelerometer.txt";
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

	float ad;
	float Kx = 0.0 , Ky = 0.0 , Kz = 0.0;

	for ( int i = 0 ; i < 1000 ; i++ ) {

		gettimeofday ( &tval , NULL );
		past_time = now_time;
		now_time  = 1000000 * tval.tv_sec + tval.tv_usec;
		interval  = now_time - past_time;

		imuLoop ();

		ad =   ax;
		ax =   ay;
		ay =   ad;
		az = - az;

		Kx += G_SI / ax / 1000;
		Ky += G_SI / ay / 1000;
		Kz += G_SI / az / 1000;

		do{
			gettimeofday ( &tval , NULL );
			interval = 1000000 * tval.tv_sec + tval.tv_usec - now_time;
		}while( interval < 2000 );

	}

	printf ( "Kx = %f , Ky = %f , Kz = %f\n" ,Kx ,Ky ,Kz ); 

	return 0;

}
