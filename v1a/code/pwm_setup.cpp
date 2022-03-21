#include <wiringPi.h>
#include <softPwm.h>
#include <cstdio>
#include <stdio.h>

#include <math.h>

#define MOTOR1_PWM_PIN 12
#define MOTOR1_DIR_PIN 24
#define MOTOR1_FLT_PIN 5
#define MOTOR1_SLP_PIN 22

#define MOTOR2_PWM_PIN 13
#define MOTOR2_DIR_PIN 25
#define MOTOR2_FLT_PIN 6
#define MOTOR2_SLP_PIN 23

#define PWM_INIT_VALUE 0
#define PWM_MAX 480
#define PWM_FREQ 20000

int motor_to_command = 0;
int pwm_command = 0;
int dir_command = 0;
// SHOULD SET UP FOR INDIVIDUAL CONTROL


void setup_pwm();
void command_motors();

int main(){
    setup_pwm();

    while(1){
        printf("Motor#(1,2,3)  PWM(0->%d) DIR(0,1):\n", PWM_MAX);
        // scanf("%d %d ", &pwm_command, &dir_command);
        scanf("%d %d %d", &motor_to_command, &pwm_command, &dir_command);
        command_motors();
    }
}

void setup_pwm(){
    wiringPiSetupGpio();
    softPwmCreate(MOTOR1_PWM_PIN, PWM_INIT_VALUE, PWM_MAX) ;
    softPwmCreate(MOTOR2_PWM_PIN, PWM_INIT_VALUE, PWM_MAX) ;

    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    
}

void command_motors(){
    switch (motor_to_command)
    {
    case 1:
        digitalWrite(MOTOR1_DIR_PIN, dir_command);
        softPwmWrite (MOTOR1_PWM_PIN, pwm_command);
        break;
    
    case 2:
        digitalWrite(MOTOR2_DIR_PIN, dir_command);
        softPwmWrite (MOTOR2_PWM_PIN, pwm_command);
        break;
    
    case 3:
        digitalWrite(MOTOR1_DIR_PIN, dir_command);
        softPwmWrite (MOTOR1_PWM_PIN, pwm_command);
        digitalWrite(MOTOR2_DIR_PIN, ~dir_command);
        softPwmWrite (MOTOR2_PWM_PIN, pwm_command);
        break;
    
    default:
        digitalWrite(MOTOR1_DIR_PIN, 0);
        softPwmWrite (MOTOR1_PWM_PIN, 0);
        digitalWrite(MOTOR2_DIR_PIN, 0);
        softPwmWrite (MOTOR2_PWM_PIN, 0);
        break;
    }
}

// gcc -o pwm pwm_setup.cpp -lwiringPi -lm -lncurses -lrt -lcrypt -lpthread