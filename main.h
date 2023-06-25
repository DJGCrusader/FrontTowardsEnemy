/*
Header file for FrontTowardsEnemy main.cpp
https://os.mbed.com/teams/TVZ-Mechatronics-Team/code/HCSR04/#cf3e4e307d15

*/

#include "mbed.h"
#include "FastMath.h"
#include <math.h>

#include "Servo.h"
#include "PPM.h"
#include "MPU6050.h"

//------------------    Debug Mode
#define USESERIAL 1

//------------------    Pins

//F446RE
//#define DRIVE0  D2
//#define DRIVE1  D3
//#define DRIVE2  D4
//#define WEAPON0 D8
//#define PPM_IN   D12
//#define IMU_SDA D5
//#define IMU_SCL D7

//L432KC
#define DRIVE0  D3
#define DRIVE1  D5
#define DRIVE2  D6
#define WEAPON0 D9
#define PPM_IN   D10
#define IMU_SDA D12
#define IMU_SCL A6

//------------------    RC Calibration Constants
#define LUD_MIN     -1
#define LUD_MAX     112
#define LRL_MIN     -1
#define LRL_MAX     112
#define RRL_MIN     -1
#define RRL_MAX     112

#define DEADBAND    10 //Percent

#define CONTROL_MODE 1
//0 means don't do anything (Debug Mode)
//1 means FPS Controls (No Weapon)
                        
//------------------    Mathematical Constants
#define DEG2ENC 16384.0f/360.0f
//#define PI 3.14159265359
#define ENC2DEG 360.0f/16384.0f
#define DEG2RAD ((2.0f*PI)/360.0f)
#define RAD2DEG (1/(DEG2RAD))
#define KT  0.45f
#define KV  (1/KT)
#define GRAV 9.80665f

//------------------    Implementation Constants
#define FREQ 1000
#define PERIOD (1.0f/FREQ)

#define SERIAL_FREQ 100
#define SERIAL_PERIOD (1.0f/SERIAL_FREQ)
#define SERIAL_RATIO (FREQ/SERIAL_FREQ)

#define W_MAX_DEG 1776.0f // in degrees/second
#define W_MAX 30.739f // in rad/s

#define K_W 100.0f/W_MAX
#define KI_W 100.0f
#define W_INT_LIMIT 0.0f

//Serial pcSerial(PC_10, PC_11);    // USE FOR LOGOMATIC! CANNOT USE USB+LOGO AT SAME TIME
Serial pcSerial(USBTX, USBRX);


bool isRunning = 1;
long teleCounter = 0;
long loopCounter = 0;
int IMUCounter = 0;

Timer timer;
double t = 0; 
double tAirStart = 0;
double dt = 0;
double tPrev = 0;
double tPrevSerial = 0;
int badTime = 0;

bool inInt = 0; //Are we in an interrupt?

//PPM object
PPM *ppmInputs;
InterruptIn *PPMinterruptPin = new InterruptIn(PPM_IN);
//Array to hold RC commands
float rcCommandInputsRaw[8] = {0,0,0,0,0,0,0,0};
float rcCommandInputs[8] = {0,0,0,0,0,0,0,0};

MPU6050 mpu(IMU_SDA,IMU_SCL);
float gx, gy, gz, gx_z, gy_z, gz_z, ax, ay, az;
double roll, pitch, yaw, rollDot, pitchDot, yawDot, gyro_comp;
double w_cmd;
double w_int;

//DigitalOut t1(D10);
//InterruptIn e1(D8);
//bool inInterrupt1 = 0;
//bool pulse1 = 0;
//void e1IntRise();
//void e1IntFall();
//int t_e1;
//double d_e1;

Servo drive0(DRIVE0);
Servo drive1(DRIVE1);
Servo drive2(DRIVE2);
Servo weapon(WEAPON0);
double driveCmds[3] = {0,0,0};

double Tx;
double Ty;

int enabled;

double debugVal = 0;
double lastCommand[8] = {0,0,0,0,0,0,0,0};

void getRCInputs();
void loop();
void setup();
void setupServo();
void scaleSkidSteer(int arrayLen);