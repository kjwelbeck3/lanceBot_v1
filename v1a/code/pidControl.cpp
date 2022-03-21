#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <softPwm.h>
#include <cstdio>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

/*****************************
 * CONSTANTS
 ****************************/
// CONTROL
 #define GYRO_MAX 300
 #define PITCH_MAX 45

// IMU
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19 // A DIVIDER FOR SCALING THE GYRO SAMPLE RATE: set to 0 for max
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C
#define COMP_FILTER_PARAM 0.02

// PWM and MOTORS
#define MOTOR1_PWM_PIN 26 //BCM 12
#define MOTOR1_DIR_PIN 5  //BCM 24
#define MOTOR1_FLT_PIN 21 //BCM 5
#define MOTOR1_SLP_PIN 3  //BCM 22
#define MOTOR2_PWM_PIN 23 //BCM 13
#define MOTOR2_DIR_PIN 6  //BCM 25
#define MOTOR2_FLT_PIN 22 //BCM 6
#define MOTOR2_SLP_PIN 4  //BCM 23
#define PWM_INIT_VALUE 0
#define PWM_MAX 480
#define PWM_FREQ 20000

// ENCODERS
#define M1ENC_SIG1_PIN 7 // BCM 4
#define M1ENC_SIG2_PIN 0 // BCM 17
#define M2ENC_SIG1_PIN 28 // BCM 20
#define M2ENC_SIG2_PIN 24 // BCM 19
#define QUADRATURE 16
#define GEAR_RATIO 50.0
#define DEGS_PER_REV 360

// PID CONTROL
#define PITCH_TOTAL_MAX 5
#define ROLL_TOTAL_MAX 5
#define P 70//100//120 fine//150 fine 200 #70 worked
#define I 0//.1//.05
#define D 0.5//0//3//10//3  0.5worked
#define SETPOINT 88.3

/*****************************
 * ENUMS
 ****************************/

// IMU
enum Ascale
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

/******************************
 * GLOBAL VARIABLES
 *****************************/

// CONTROL
bool run_program = 1;

// TIMING
long int time_curr;
long int time_prev;
struct timespec te;

// IMU
int imu;
// float x_gyro_calibration = 0.001602;                //0.001495;
// float y_gyro_calibration = 0.001587;                //0.001556;
// float z_gyro_calibration = 0.000885;                //0.000885;
// float roll_calibration = -0.234493;                 //-0.308980;
// float pitch_calibration = -0.816976;                //-2.083809;
// float accel_z_calibration = -1.117346;              //-1.115622;

float x_gyro_calibration = 0.001556;                //0.001495;
float y_gyro_calibration = 0.001892;                //0.001556;
float z_gyro_calibration = 0.001007;                //0.000885;
float roll_calibration = -0.037427;                 //-0.308980;
float pitch_calibration = -0.211945;                //-2.083809;
float accel_z_calibration = -1.119305;              //-1.115622;



// float x_gyro_calibration = 0;
// float y_gyro_calibration = 0;
// float z_gyro_calibration = 0;
// float roll_calibration = 0;
// float pitch_calibration = 0;
// float accel_z_calibration = 0;
float imu_data[6]; //gyro xyz, accel xyz

float pitch_accel = 0;
float roll_accel = 0;
float pitch_gyro = 0;
float roll_gyro = 0;
float filtered_pitch = 0;
float filtered_roll = 0;

// PID CONTROL
float prev_pitch;
float prev_roll;
float pitch_rate;
float roll_rate;
float pitch_total = 0;
float roll_total = 0;
int pid;


// MOTORS and PWM
// int motor_to_command = 0;
int pwm_command = 0;
int dir_command = 0;

// ENCODERS
long motor1_counts = 0;
long motor2_counts = 0;

// data runs
// FILE *imu_calibration_file = fopen("imu_calibration.csv", "w");
// FILE *imu_post_calibration_file = fopen("imu_post_calibration.csv", "w");
// FILE *imu_pre_calibration_file = fopen("imu__pre_calibration.csv", "w");
// FILE *comp_filter_file = fopen("comp_filter_data.csv", "w");
// FILE *p_file = fopen("p_data.csv", "w");
// FILE *d_file = fopen("d_data.csv", "w");
FILE *pd_file = fopen("pd_data.csv", "w");
// FILE *pid_file = fopen("pid_data.csv", "w");

/******************************
 * HELPER FUNCTION PROTOTYPES
 *****************************/
// CONTROL
void safety_check();

// IMU
int setup_imu();
void calibrate_imu();
void read_imu();

// CONTROL
void update_state();
void pid_control();

// PWM and MOTORS
void setup_pwm();
void command_motors();
void stop_motors();

// ENCODERS
void setup_encoders();
void handleM1ENC1_rise();
void handleM2ENC1_rise();

/******************************
 * MAIN FUNCTION
 *****************************/

int main(int argc, char *argv[])
{
    wiringPiSetup();
    setup_imu();
    calibrate_imu();
    setup_pwm();
    // setup_encoders();

    //   Forwarding Ctrl+C SIGINT to stop motors and then program
    // signal(SIGINT, &stop_motors);

    while (run_program==1)
    {
        read_imu();
        update_state();
        // safety_check();
        pid_control();
        command_motors();
        usleep(10000);
        // printf("X:%10.5f \tY:%10.5f \tZ:%10.5f \troll:%10.5f \tpitch:%10.5f\n\r", imu_data[0], imu_data[1], imu_data[2], filtered_roll, filtered_pitch);
        // fprintf(imu_pre_calibration_file, "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f\n", imu_data[0], imu_data[1], imu_data[2], filtered_roll, filtered_pitch);
        // fprintf(imu_post_calibration_file, "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f\n", imu_data[0], imu_data[1], imu_data[2], filtered_roll, filtered_pitch);
        // fprintf(comp_filter_file, "pitch_accel:%10.5f,pitch_gyro:%10.5f \tfiltered_pitch:%10.5f \troll_accel:%10.5f \troll_gyro:%10.5f \tfiltered_roll:%10.5f\n", pitch_accel, pitch_gyro, filtered_pitch, roll_accel, roll_gyro, filtered_roll);
        // printf("%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f\n", pitch_accel, pitch_gyro, filtered_pitch, roll_accel, roll_gyro, filtered_roll);
        // fprintf(comp_filter_file, "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f\n", pitch_accel, pitch_gyro, filtered_pitch, roll_accel, roll_gyro, filtered_roll);
        // printf("pitch: %10.5f\n", filtered_pitch);
    }
    stop_motors();
    stop_motors();
    return 0;
}

/******************************
 * HELPER FUNCTIONS
 *****************************/

int setup_imu()
{
    //setup imu on I2C
    imu = wiringPiI2CSetup(0x68); //accel/gyro address

    if (imu == -1)
    {
        printf("-----cant connect to I2C device %d --------\n", imu);
        return -1;
    }
    else
    {
        printf("connected to i2c device %d\n", imu);
        printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu, 0x75));

        uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
        uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

        //init imu
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x00);
        printf("                    \n\r");
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x01);
        wiringPiI2CWriteReg8(imu, CONFIG, 0x00);     // DISABLING ext Frame sync pin and DigLowPassFil --> Sets GYRO OUTPUT RATE TO 8KHz instead of 1kHz (accel rate is unchangeable at 1KHz)
        wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04 SETTING DIVIDER to 0 for MAX SAMPLING RATE (8Khz)
        int c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0xE0);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
        // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F); //
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);
    }
    return 0;
}

void calibrate_imu()
{
    // clearing calibration variable for fresh callibration
    x_gyro_calibration = 0;
    y_gyro_calibration = 0;
    z_gyro_calibration = 0;
    accel_z_calibration = 0;
    pitch_calibration = 0;
    roll_calibration = 0;

    // summing data over a 1000 data reads
    for (int i = 0; i < 1000; i++)
    {
        read_imu();
        x_gyro_calibration -= imu_data[0];
        y_gyro_calibration -= imu_data[1];
        z_gyro_calibration -= imu_data[2];
        accel_z_calibration -= imu_data[5];

        // Calculating the roll and pitch angles in degree,
        // to trend in expected positive directions
        pitch_calibration += (std::atan2(imu_data[3], imu_data[5]) * 180 / M_PI);
        roll_calibration += (std::atan2(imu_data[4], imu_data[5]) * 180 / M_PI);
        // fprintf(imu_calibration_file, "%f, %f, %f, %f, %f, %f\n", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration );
    }
    // Dividing by 1000 for the averages

    x_gyro_calibration /= 1000;
    y_gyro_calibration /= 1000;
    z_gyro_calibration /= 1000;
    accel_z_calibration /= 1000;
    pitch_calibration /= 1000;
    roll_calibration /= 1000;

    
    printf("calibration complete, %f, %f, %f, %f, %f, %f\n\r", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration);
}

void read_imu()
{
    int address = 0x3B; //address value for accel x value
    float ax = 0;
    float az = 0;
    float ay = 0;
    int vh, vl;

    //read in data
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    //convert 2 complement
    int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-2g
    imu_data[3] = 2.0 / 32767 * vw;

    address = 0x3D; // address value for accel y value
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-2g
    imu_data[4] = 2.0 / 32767 * vw;

    address = 0x3F; //addres value for accel z value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-2g
    imu_data[5] = 2.0 / 32767 * vw;

    address = 0x43; // address value for gyro x value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-500dps
    imu_data[0] = x_gyro_calibration + 500.0 / 32767 * vw;

    address = 0x45; // address value for gyro y value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-500dps
    imu_data[1] = y_gyro_calibration + 500.0 / 32767 * vw;

    address = 0x47; ////todo: set addres value for gyro z value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // scaling between +/-500dps
    imu_data[2] = z_gyro_calibration + 500.0 / 32767 * vw;

    // calcultaing roll and pitch angles in degrees with calibration offsets
    pitch_accel = (std::atan2(imu_data[3], imu_data[5]) * 180 / M_PI) - pitch_calibration;
    roll_accel = ((std::atan2(imu_data[4], imu_data[5]) * 180 / M_PI) - roll_calibration);
}

void update_state()
{

    timespec_get(&te, TIME_UTC);
    time_curr = te.tv_nsec;
    //compute time since last execution
    float time_diff = time_curr - time_prev;
    // printf("dt: %f\n\r", time_diff);

    //check for rollover
    if (time_diff <= 0)
    {
        time_diff += 1000000000;
    }
    //convert to seconds
    time_diff = time_diff / 1000000000;
    time_prev = time_curr;

   

    // The pitch (and roll) calculated
    pitch_gyro += time_diff * -1*imu_data[1];
    roll_gyro += time_diff * imu_data[0];
    filtered_pitch = COMP_FILTER_PARAM * pitch_accel + (1 - COMP_FILTER_PARAM) * (filtered_pitch -  time_diff * imu_data[1]);
    filtered_roll = COMP_FILTER_PARAM * roll_accel + (1 - COMP_FILTER_PARAM) * (filtered_roll + time_diff * imu_data[0]);

    printf("pitch: %f\n\r", filtered_pitch);
    // The pitch (and roll) rate(s)
    pitch_rate = (filtered_pitch - prev_pitch) / time_diff;
    // printf("pitchrate: %f", pitch_rate);
    roll_rate = (filtered_roll - prev_roll) / time_diff;
    
    prev_pitch = filtered_pitch;
    prev_roll = filtered_roll;

    // Aggregating pitch (and roll)
    pitch_total += (filtered_pitch - SETPOINT);// * time_diff;
    roll_total += filtered_roll;// * time_diff;

    if (pitch_total < -1 * PITCH_TOTAL_MAX)
    {
        pitch_total = -1 * PITCH_TOTAL_MAX;
    }
    if (pitch_total > PITCH_TOTAL_MAX)
    {
        pitch_total = PITCH_TOTAL_MAX;
    }

    if (roll_total < -1 * ROLL_TOTAL_MAX)
    {
        roll_total = -1 * PITCH_TOTAL_MAX;
    }
    if (roll_total > ROLL_TOTAL_MAX)
    {
        roll_total = PITCH_TOTAL_MAX;
    }

    // The
}

void pid_control(){
    pid = P*(filtered_pitch - SETPOINT) + I* pitch_total + D*pitch_rate;
    dir_command = 0 <= pid ? 1 : 0;
    pwm_command = abs(pid) > PWM_MAX ? PWM_MAX : abs(pid);
    
    // fprintf(p_file, "%10.5f, %10.5f, %d, %d\n", pitch_accel, filtered_pitch, pid, pwm_command);
    // fprintf(d_file, "%10.5f, %10.5f, %10.5f, %d, %d\n", pitch_accel, filtered_pitch, pitch_rate, pid, pwm_command);
    fprintf(pd_file, "%10.5f, %10.5f, %10.5f, %d, %d\n", pitch_accel, filtered_pitch, pitch_rate, pid, pwm_command);
    // fprintf(pid_file, "%10.5f, %10.5f, %d, %d\n", pitch_accel, filtered_pitch, pid, pwm_command);

    
}

void setup_pwm(){
    softPwmCreate(MOTOR1_PWM_PIN, PWM_INIT_VALUE, PWM_MAX) ;
    softPwmCreate(MOTOR2_PWM_PIN, PWM_INIT_VALUE, PWM_MAX) ;

    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR1_SLP_PIN, OUTPUT);
    pinMode(MOTOR2_SLP_PIN, OUTPUT);
    digitalWrite(MOTOR1_SLP_PIN, 1);
    digitalWrite(MOTOR2_SLP_PIN, 1);
    
}

void command_motors(){
    // printf("pwm: %d\t dir: %d\n\r", pwm_command, dir_command);
    digitalWrite(MOTOR1_DIR_PIN, dir_command);
    softPwmWrite (MOTOR1_PWM_PIN, pwm_command);
    digitalWrite(MOTOR2_DIR_PIN, !dir_command);
    softPwmWrite (MOTOR2_PWM_PIN, pwm_command);
}

void stop_motors(){
    // digitalWrite(MOTOR1_DIR_PIN, dir_command);
    softPwmWrite (MOTOR1_PWM_PIN, 0);
    // digitalWrite(MOTOR2_DIR_PIN, !dir_command);
    printf("sigint command");
    softPwmWrite (MOTOR2_PWM_PIN, 0);
    run_program = 0;
    sleep(2);
}


void setup_encoders(){
    pinMode(M1ENC_SIG2_PIN, INPUT);
    pinMode(M2ENC_SIG2_PIN, INPUT);

    wiringPiISR (M1ENC_SIG1_PIN, INT_EDGE_RISING, &handleM1ENC1_rise) ;
    wiringPiISR (M2ENC_SIG1_PIN, INT_EDGE_RISING, &handleM2ENC1_rise) ;

    // return 0;
}

void handleM1ENC1_rise(){
    if (digitalRead(M1ENC_SIG2_PIN) == 1){
        ++motor1_counts;
    }else
    {
        --motor1_counts;
    }
}

void handleM2ENC1_rise(){
    if (digitalRead(M2ENC_SIG2_PIN) == 1){
        ++motor2_counts;
    }else
    {
        --motor2_counts;
    }
}

// gcc -o pid pidControl.cpp -lwiringPi -lm -lncurses -lrt -lcrypt -lpthread


// NEED a SigINT command

void safety_check(){

   // Check for gyro rate limits
    // if ((imu_data[0] > GYRO_MAX) || (imu_data[1] > GYRO_MAX) || (imu_data[2] > GYRO_MAX))
    // {
    //   run_program = 0;
    //   printf("[SAFETY_ERROR_2]: Gyro rate [x:%f, y:%f, z:%f] exceeds %d limit. \n", imu_data[0], imu_data[1], imu_data[2], GYRO_MAX);
    // } 

   // Check for pitch angle limits
    if ((filtered_pitch > PITCH_MAX) || (filtered_pitch < -1*PITCH_MAX))
    {
      run_program = 0;
      printf("[SAFETY_ERROR_4]: Pitch angle %f execeeds +/-%d limits.\n", filtered_pitch, PITCH_MAX);
    }
}