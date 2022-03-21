// Calibrating the IMU

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
// #include <time.h>
#include <math.h>
// #include <sys/time.h>
#include <stdint.h>
// #include <signal.h>
// #include <sys/shm.h>
// #include <sys/stat.h>
// #include <curses.h>

/*****************************
 * IMU CONSTANTS
 ****************************/

// #define frequency 25000000.0
#define CONFIG 0x1A 
#define SMPLRT_DIV 0x19 // A DIVIDER FOR SCALING THE GYRO SAMPLE RATE: set to 0 for max
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
// #define ACCEL_CONFIG2 0x1D
// #define USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C

#define COMP_FILTER_PARAM 0.02

/*****************************
 * ENUMS
 ****************************/

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

int imu;
float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; //gyro xyz, accel xyz
long int time_curr;
long int time_prev;
struct timespec te;
float yaw = 0;

float pitch_angle = 0;
float roll_angle = 0;
float pitch_angle_gyro = 0;
float roll_angle_gyro = 0;
float pitch_angle_filter = 0;
float roll_angle_filter = 0;

/******************************
 * HELPER FUNCTION PROTOTYPES
 *****************************/

int setup_imu();
void calibrate_imu();
void read_imu();

/******************************
 * MAIN FUNCTION
 *****************************/

int main(int argc, char *argv[])
{
  setup_imu();
  calibrate_imu();
  printf("First set of calibrated values:\n\r");
  while(1)
  {
    read_imu();
    printf("X:%10.5f \tY:%10.5f \tZ:%10.5f \troll:%10.5f \tpitch:%10.5f\n\r", imu_data[0], imu_data[1], imu_data[2], roll_angle, pitch_angle);
  }
//   return 0;
}

/******************************
 * HELPER FUNCTIONS
 *****************************/

int setup_imu()
{
  wiringPiSetup();

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
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00); // DISABLING ext Frame sync pin and DigLowPassFil --> Sets GYRO OUTPUT RATE TO 8KHz instead of 1kHz (accel rate is unchangeable at 1KHz)
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
  }
  // Dividing by 1000 for the averages

  x_gyro_calibration /= 1000;
  y_gyro_calibration /= 1000;
  z_gyro_calibration /= 1000;
  accel_z_calibration /= 1000;
  pitch_calibration /= 1000;
  roll_calibration /= 1000;

  printf("calibration complete, %f %f %f %f %f %f\n\r", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration);
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
  pitch_angle = (std::atan2(imu_data[3], imu_data[5]) * 180 / M_PI) - pitch_calibration;
  roll_angle = ((std::atan2(imu_data[4], imu_data[5]) * 180 / M_PI) - roll_calibration);
}

// gcc -o setup setup.cpp -lwiringPi -lm -lncurses -lrt -lcrypt
