
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
#include <fstream>

using namespace std;

#include <unistd.h>
#include "../Navio2/C++/Navio/PWM.h"
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

float ax, ay, az;
float gx, gy, gz;

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
	for(int i = 0; i<100; i++)
	{
		imu->update();
		imu->read_gyroscope(&gx, &gy, &gz);

		gx *= 180 / PI;
		gy *= 180 / PI;
		gz *= 180 / PI;

		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}


void imuLoop()
{
	//----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
	gettimeofday(&tv,NULL);
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

	//-------- Read raw measurements from the MPU and update AHRS --------------

	// Accel + gyro.
	imu->update();
	imu->read_accelerometer(&ax, &ay, &az);
	imu->read_gyroscope(&gx, &gy, &gz);

	ax /= G_SI;
	ay /= G_SI;
	az /= G_SI;
	gx *= 180 / PI;
	gy *= 180 / PI;
	gz *= 180 / PI;

	ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

	//------------------- Discard the time of the first cycle -----------------

	if (!isFirst)
	{
		if (dt > maxdt) maxdt = dt;
		if (dt < mindt) mindt = dt;
	}
	isFirst = 0;

	//------------- Console and network output with a lowered rate ------------
#if 1
	dtsumm += dt;
	if(dtsumm > 0.05)
	{
		dtsumm = 0;
	}
#endif
}


InertialSensor* create_inertial_sensor(char *sensor_name)
{
	InertialSensor *imu;

	if (!strcmp(sensor_name, "mpu")) {
		printf("Selected: MPU9250\n");
		imu = new MPU9250();
	}
	else if (!strcmp(sensor_name, "lsm")) {
		printf("Selected: LSM9DS1\n");
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
	char sensor_name[]="mpu";
	
	//-------- IMU setting -------------------
	imu = create_inertial_sensor(sensor_name);

	if (!imu) {
		printf("Wrong sensor name. Select: mpu or lsm\n");
		return EXIT_FAILURE;
	}

	if (!imu->probe()) {
		printf("Sensor not enable\n");
		return EXIT_FAILURE;
	}

	imuSetup();


	PWM pwm;

	if (check_apm()) {
		return 1;
	}

	
	for (int i=0; i<10000; i++){
		imuLoop();
		usleep(1000);
		if(i%1000==0){
			printf("#");
			fflush(stdout);
				
		}
	}
	
	
	for (int i=0;i<4;i++){
		if (!pwm.init(i)) {
			fprintf(stderr, "Output Enable not set. Are you root?\n");
			return 0;
		}
		pwm.enable(i);
		pwm.set_period(i, 500);
	}
	printf("\nReady to Fly !\n");

	////// Log system setting
	char filename[] = "motor-vivration.txt";
	char outstr[255];
	std::ofstream fs(filename);

	///// Clock setting
	struct timeval tval;
	unsigned long now_time,past_time,interval;

	gettimeofday(&tval,NULL);
	now_time=1000000 * tval.tv_sec + tval.tv_usec;
	past_time = now_time;
	interval = now_time - past_time;

	int n = 0;
	double t = 0.0;
	float u[4]; 
	int i = 0;

//==========================  Main Loop ==============================
	
	while(n<4){

		t = 0.0;

		while(t<10000000.0) {

			do{
				gettimeofday(&tval,NULL);
				past_time = now_time;
				now_time=1000000 * tval.tv_sec + tval.tv_usec;
				interval = interval + now_time - past_time;
			}while(interval<=2000);

			imuLoop();

			t = t + interval;

			if ( t > 10000000.0 ){
				t=10000000.0;
			}

			u[n]= t / 10000000.0 + 1.0;

			if ( n == 0 ) {
				u[n] = u[n] + 0.130;
			}
			if ( n == 1 ) {
				u[n] = u[n] + 0.055;
			}
			if ( n == 2 ) {
				u[n] = u[n] + 0.000;
			}
			if ( n == 3 ) {
				u[n] = u[n] + 0.095;
			}

			if ( u[n] > 2.0 ){
				u[n] = 2.0;
			}

			for(i=0;i<4;i++){
				if(i==n){
					i++;
				}
				u[i]=1.0;
			}
		
			pwm.set_duty_cycle(RIGHT_MOTOR, u[0]);
			pwm.set_duty_cycle(LEFT_MOTOR , u[1]);
			pwm.set_duty_cycle(FRONT_MOTOR, u[2]);
			pwm.set_duty_cycle(REAR_MOTOR , u[3]);

			sprintf(outstr,"%lu %lu %f %f %f",now_time,interval,t,gz,u[n]);
			fs << outstr << endl;

			interval = 0;
	
		}

		t = 0.0;

		for(i=0;i<4;i++){
			u[i]=1.0;
		}

		pwm.set_duty_cycle(RIGHT_MOTOR, u[0]);
		pwm.set_duty_cycle(LEFT_MOTOR , u[1]);
		pwm.set_duty_cycle(FRONT_MOTOR, u[2]);
		pwm.set_duty_cycle(REAR_MOTOR , u[3]);

		do{
			do{
				gettimeofday(&tval,NULL);
				past_time = now_time;
				now_time=1000000 * tval.tv_sec + tval.tv_usec;
				interval = interval + now_time - past_time;
			}while(interval<=2000);
			t = t + interval;
			imuLoop();
			sprintf(outstr,"%lu %lu %f %f %f",now_time,interval,t,gz,u[n]);
			fs << outstr << endl;
			interval = 0;
		}while(t<=5000000.0);

		n++;

	}
		
	fs.close();
	return 0;

}
