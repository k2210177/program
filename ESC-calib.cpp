/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <unistd.h>
#include "Navio/PWM.h"
#include "Navio/Util.h"

#define PWM_OUTPUT 0
#define SERVO_MIN 1.000 /*mS*/
#define SERVO_MAX 2.000 /*mS*/

int main()
{
    PWM pwm;

    if (check_apm()) {
        return 1;
    }    

    if (!pwm.init(0)) {
        fprintf(stderr, "Output Enable not set. Are you root?\n");
        return 0;
    }
    if (!pwm.init(1)) {
        fprintf(stderr, "Output Enable not set. Are you root?\n");
        return 0;
    }
    if (!pwm.init(2)) {
        fprintf(stderr, "Output Enable not set. Are you root?\n");
        return 0;
    }
    if (!pwm.init(3)) {
        fprintf(stderr, "Output Enable not set. Are you root?\n");
        return 0;
    }

    pwm.enable(0);
    pwm.set_period(0, 50);

    pwm.enable(1);
    pwm.set_period(1, 50);

    pwm.enable(2);
    pwm.set_period(2, 50);

    pwm.enable(3);
    pwm.set_period(3, 50);


    short i=0;

    pwm.set_duty_cycle(0, SERVO_MAX);
    pwm.set_duty_cycle(1, SERVO_MAX);
    pwm.set_duty_cycle(2, SERVO_MAX);
    pwm.set_duty_cycle(3, SERVO_MAX);
    sleep(5);
    pwm.set_duty_cycle(0, SERVO_MIN);
    pwm.set_duty_cycle(1, SERVO_MIN);
    pwm.set_duty_cycle(2, SERVO_MIN);
    pwm.set_duty_cycle(3, SERVO_MIN);
    sleep(10);
    printf("Finish\n");
    return 0;
}
