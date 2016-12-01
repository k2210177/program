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

#include "../Navio2/C++/Navio/PWM.h"
#include "../Navio2/C++/Navio/RGBled.h"
#include "../Navio2/C++/Navio/Util.h"
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <sys/time.h>

#define SERVO_MIN 1.0 /*mS*/
#define SERVO_MAX 2.0 /*mS*/
#define G_SI 9.80665
#define PI   3.14159

// Timing data

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

//==================== Start Main function =========================

int main()
{

	int breakflag=0;
	float M = 1.0;
	short m = 0;
	short cnt = 0;

	PWM pwm;
	RGBled led;
	js_event js;

	if(!led.initialize()) return EXIT_FAILURE;


	//-------- PS3 Controller setting --------
	int joy_fd(-1), num_of_axis(0), num_of_buttons(0);
	char name_of_joystick[80];
	vector<char> joy_button;
	vector<int> joy_axis;

	if((joy_fd = open(JOY_DEV, O_RDONLY)) < 0) {
		printf("Failed to open %s", JOY_DEV);
		cerr << "Failed to open " << JOY_DEV << endl;
		return -1;
	}

	ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
	ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
	ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

	joy_button.resize(num_of_buttons, 0);
	joy_axis.resize(num_of_axis, 0);

	printf("Joystick: %s axis: %d buttons: %d\n", name_of_joystick, num_of_axis, num_of_buttons);

	fcntl(joy_fd, F_SETFL, O_NONBLOCK); // using non-blocking mode

	for ( int i = 0 ; i < 4 ; i++ ) {

		if ( !pwm.init(i) ) {
			fprintf(stderr, "Output Enable not set. Are you root?\n");
			return 0;
		}
		pwm.enable(i);
		pwm.set_period(i, 500);

	}

	printf("\nStart thrust test !\n");

	///// Clock setting
	struct timeval tval;
	unsigned long now_time,past_time,interval;

	gettimeofday(&tval,NULL);
	now_time=1000000 * tval.tv_sec + tval.tv_usec;
	past_time = now_time;
	interval = now_time - past_time;

	printf("set motor 'RIGHT'\n");

//==========================  Main Loop ==============================
	while(true) {

		led.setColor(Colors::Red);

		while(true) {

			gettimeofday(&tval,NULL);
			past_time = now_time;
			now_time=1000000 * tval.tv_sec + tval.tv_usec;
			interval = now_time - past_time;

			//js_event js;

			read(joy_fd, &js, sizeof(js_event));

			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joy_axis[(int)js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[(int)js.number] = js.value;
					//printf("%5d\n %5d\n",(int)js.number,js.value);
					break;
			}

			if (joy_button[12]==1){

				if (joy_button[4]==1){

					M = M + 0.1;
					printf ( "PWM UP : %f\n" ,M );

				}
				if (joy_button[6]==1){

					M = M - 0.1;
					printf ( "PWM down : %f\n" ,M );

				}

			}
			if (joy_button[13]==1){

				if (joy_button[4]==1){

					M = M + 0.001;
					printf ( "PWM UP : %f\n" ,M );

				}
				if (joy_button[6]==1){

					M = M - 0.001;
					printf ( "PWM down : %f\n" ,M );

				}

			}
			if (joy_button[14]==1){

				if (joy_button[4]==1){

					M = M + 0.0001;
					printf ( "PWM UP : %f\n" ,M );

				}
				if (joy_button[6]==1){

					M = M - 0.0001;
					printf ( "PWM down : %f\n" ,M );

				}

			}
			if (joy_button[15]==1){

				if (joy_button[4]==1){

					M = M + 0.00001;
					printf ( "PWM UP : %f\n" ,M );

				}
				if (joy_button[6]==1){

					M = M - 0.0001;
					printf ( "PWM down : %f\n" ,M );

				}

			}

			if (joy_button[11]==1){

				cnt++;

				if (cnt>30){

					M = 2.0;
					printf( "Set PWM MAX!\n" );
					cnt = 0;

				}

			}
			if (joy_button[10]==1){

				cnt++;

				if (cnt>30){

					M = 1.0;
					printf( "Set PWM MIN!\n" );
					cnt = 0;

				}

			}

			if ( M > 2.0 ) M = 2.0;
			if ( M < 1.0 ) M = 1.0;

			pwm.set_duty_cycle(m, M);

			if (joy_button[3]==1){
				break;
			}

			do{
				gettimeofday(&tval,NULL);
				interval=1000000 * tval.tv_sec + tval.tv_usec - now_time;

			}while(interval<2000);

		}
		led.setColor(Colors::Blue);
		while (joy_button[3]==1){
			read(joy_fd, &js, sizeof(js_event));
			
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joy_axis[(int)js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[(int)js.number] = js.value;
					//printf("%5d\n %5d\n",(int)js.number,js.value);
					break;
			}
		}
		while (true){
			read(joy_fd, &js, sizeof(js_event));
			
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joy_axis[(int)js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[(int)js.number] = js.value;
					//printf("%5d\n %5d\n",(int)js.number,js.value);
					break;
			}

			if (joy_button[4]==1){
				m = 2;
				printf("set motor 'FRONT'\n");
			}
			if (joy_button[5]==1){
				m = 0;
				printf("set motor 'RIGHT'\n");
			}
			if (joy_button[6]==1){
				m = 3;
				printf("set motor 'REAR'\n");
			}
			if (joy_button[7]==1){
				m = 1;
				printf("set motor 'LEFT'\n");
			}
			if (joy_button[3]==1){
				break;
			}
			if (joy_button[0]==1){
				breakflag=1;
				break;
			}
			else breakflag = 0;
		}
		if (breakflag == 1)break;
		else breakflag = 0;

		while (joy_button[3]==1){
			read(joy_fd, &js, sizeof(js_event));
			
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joy_axis[(int)js.number] = js.value;
					break;
				case JS_EVENT_BUTTON:
					joy_button[(int)js.number] = js.value;
					//printf("%5d\n %5d\n",(int)js.number,js.value);
					break;
			}
		}
	}

	led.setColor(Colors::Green);

	return 0;

}
