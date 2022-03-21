#include <Wire.h>
#include <LSM6.h>


#define CALIBRATION_ANGLE -90
#define FILTER_PARAM 0.02

#define PWM_1 3
#define PWM_2 6
#define DIR_1 8
#define DIR_2 13

#define P 25
#define I 2
#define D 0
#define PITCH_SUM_MAX 2

LSM6 imu;

double imu_data[6];
char report[150];
char report2[150];
char calibrated_info[200];


double x_gyro_calibration = 0;
double y_gyro_calibration = 0;
double z_gyro_calibration = 0;
double accel_z_calibration = 0;
double pitch_calibration = 0;
double roll_calibration = 0;

double pitch = 0;
double pitch_rate = 0;
double pitch_summed = 0;
double pitch_prev = 0;
double pitch_angle = 0;
double pitch_gyro = 0; // CALIBRATION_ANGLE;
double roll_angle = 0;
unsigned long last_read_time = 0; 
double dt;
bool measuring = 0;

double pid = 0;
double desired_pitch = 0;
int volatile motor_pwm = 0;
int volatile motor_dir = 0;






void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init()) {
//    Serial.println("Failed to detect and initialize IMU");
    while (1);
  }
  else {
//    Serial.println("Detected and initialized IMU");
  }
  imu.enableDefault();
  calibrate_imu(CALIBRATION_ANGLE,0);

  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  read_imu();
  filter();
  pid_controller();
  command_motors();

//  Serial.print(pitch_angle, 6);
//  Serial.print(", ");
//  Serial.print(pitch_gyro, 6);
//  Serial.print(", ");
//  Serial.println(pitch, 6);  

  Serial.print(pitch, 6);
  Serial.print(", ");
  Serial.println(motor_pwm, 6);  

//  delay(1000);
}

void calibrate_imu(double expected_pitch, double expected_roll)
{
  x_gyro_calibration = 0;
  y_gyro_calibration = 0;
  z_gyro_calibration = 0;
  accel_z_calibration = 0;
  pitch_calibration = 0;
  roll_calibration = 0;

  double x_gyro_sum = 0;


  for (int i = 0; i < 1000; i++)
  {
    read_imu();
    x_gyro_sum -= imu_data[3];
    pitch_calibration += atan2(1.0 * imu_data[1], -imu_data[2]) * 180 / M_PI - expected_pitch;
    roll_calibration += atan2(1.0 * imu_data[0], -imu_data[2]) * 180 / M_PI - expected_roll;
//    Serial.println(x_gyro_sum, 6);
  }
  
  x_gyro_calibration = x_gyro_sum/1000;
  pitch_calibration /= 1000;
  roll_calibration /= 1000;

//    Serial.println("calibrated_info: ");
//    Serial.print("x_gyro: ");
//    Serial.print(x_gyro_calibration, 6);
//    Serial.print("pitch_calibration: ");
//    Serial.print(pitch_calibration, 6);
//    Serial.print("| roll_calibration: ");
//    Serial.println(roll_calibration, 6);
}

void read_imu(){
  imu.read();
  unsigned long now = micros();
  dt = (now - last_read_time)/1e6;
  last_read_time = now;
  
  imu_data[0] = imu.a.x * 0.000061;
  imu_data[1] = imu.a.y * 0.000061;
  imu_data[2] = imu.a.z * 0.000061;
  //  imu_data[3] = (imu.g.x * 0.007477 + x_gyro_calibration);
  imu_data[3] = (imu.g.x * 0.00875 + x_gyro_calibration);
  imu_data[4] = imu.g.y * 0.00875 + y_gyro_calibration;
  imu_data[5] = imu.g.z * 0.00875 + z_gyro_calibration;


  pitch_angle = atan2(1.0 * imu_data[1], -imu_data[2]) * 180 / M_PI - pitch_calibration;
  roll_angle = atan2(1.0 * imu_data[0], -imu_data[2]) * 180 / M_PI - roll_calibration;
}
void filter(){
  if (measuring)
  {
    pitch_gyro -= imu_data[3] * dt;
    pitch = FILTER_PARAM * pitch_angle + (1 - FILTER_PARAM) * (pitch + dt * -imu_data[3]);
    pitch_rate = (pitch - pitch_prev)/dt;
    pitch_prev = pitch; 
  }
  else{
    pitch_gyro = pitch_angle;
    pitch = pitch_angle;
    measuring = 1;
    }
  
  pitch_summed += (pitch - desired_pitch); 
  if (pitch_summed > PITCH_SUM_MAX){pitch_summed = PITCH_SUM_MAX;}
  if (pitch_summed < -1.0*PITCH_SUM_MAX) {pitch_summed = -1.0 * PITCH_SUM_MAX;}
  
}

void pid_controller(){
  if (measuring){
    pid = P*(pitch - desired_pitch) + I*pitch_summed + D*pitch_rate;
      if (measuring){
        motor_dir = pid > 0 ? 1 : 0;
        motor_pwm = abs(pid);
      }
    }
  }

void command_motors(){
  digitalWrite(DIR_1, motor_dir);
  digitalWrite(DIR_2, !motor_dir);
  analogWrite(PWM_1, motor_pwm);
  analogWrite(PWM_2, motor_pwm);
  }
