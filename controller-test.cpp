
/*
   Itolab controller check program

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
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>

//==================== Start Main function =========================

int main()
{

	int breakflag = 0;
	js_event js;	

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

//==========================  Main Loop ==============================

	while ( true ) {

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

		float stickRx =  joy_axis[2] / 23170.0;
		float stickRy = -joy_axis[3] / 23170.0;
		float stickLx =  joy_axis[0] / 23170.0;
		float stickLy =  joy_axis[1] / 23170.0;
		if ( stickRx >  1.0 ) stickRx =  1.0;
		if ( stickRx < -1.0 ) stickRx = -1.0;
		if ( stickRy >  1.0 ) stickRy =  1.0;
		if ( stickRy < -1.0 ) stickRy = -1.0;
		if ( stickLx >  1.0 ) stickLx =  1.0;
		if ( stickLx < -1.0 ) stickLx = -1.0;
		if ( stickLy >  1.0 ) stickLy =  1.0;
		if ( stickLy < -1.0 ) stickLy = -1.0;

		if ( stickLx > 0.01 || stickLx < -0.01 ) {
			printf("stickLx = %f\n\n",stickLx);
			usleep(100000);
		}
		if ( stickLy > 0.01 || stickLy < -0.01 ) {
			printf("stickLy = %f\n\n",stickLy);
			usleep(100000);
		}
		if ( stickRx > 0.01 || stickRx < -0.01 ) {
			printf("stickRx = %f\n\n",stickRx);
			usleep(100000);
		}
		if ( stickRy > 0.01 || stickRy < -0.01 ) {
			printf("stickRy = %f\n\n",stickRy);
			usleep(100000);
		}
		if ( joy_button[0] == 1 ) {
			printf("select\n\n");
			usleep(100000);
		}
		if ( joy_button[1] == 1 ) {
			printf("stickL click\n\n");
			usleep(100000);
		}
		if ( joy_button[2] == 1 ) {
			printf("stickR click\n\n");
			usleep(100000);
		}
		if ( joy_button[3] == 1 ) {
			printf("start\n\n");
			usleep(100000);
		}
		if ( joy_button[4] == 1 ) {
			printf("up button\n\n");
			usleep(100000);
		}
		if ( joy_button[5] == 1 ) {
			printf("right button\n\n");
			usleep(100000);
		}
		if ( joy_button[6] == 1 ) {
			printf("down button\n\n");
			usleep(100000);
		}
		if ( joy_button[7] == 1 ) {
			printf("left button\n\n");
			usleep(100000);
		}
		if ( joy_button[8] == 1 ) {
			printf("L2 button\n\n");
			usleep(100000);
		}
		if ( joy_button[9] == 1 ) {
			printf("R2 button\n\n");
			usleep(100000);
		}
		if ( joy_button[10] == 1 ) {
			printf("L1 button\n\n");
			usleep(100000);
		}
		if ( joy_button[11] == 1 ) {
			printf("R1 button\n\n");
			usleep(100000);
		}
		if ( joy_button[12] == 1 ) {
			printf("sankaku button\n\n");
			usleep(100000);
		}
		if ( joy_button[13] == 1 ) {
			printf("maru button\n\n");
			usleep(100000);
		}
		if ( joy_button[14] == 1 ) {
			printf("batsu button\n\n");
			usleep(100000);
		}
		if ( joy_button[15] == 1 ) {
			printf("sikaku button\n\n");
			usleep(100000);
		}

	}

	return 0;

}
