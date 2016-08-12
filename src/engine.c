/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "engine.h"
#include "adc.h"
#include "gyro.h"
#include "utils.h"
#include "config.h"
#include "pwm.h"
#include "rc.h"
#include "comio.h"
#include "stopwatch.h"
#include "i2c.h"
#include "definitions.h"
#include "usb.h"
#include "main.h"

int debugPrint   = 0;
int debugPerf    = 0;
int debugSense   = 0;
int debugCnt     = 0;
int debugRC      = 0;
int debugOrient  = 0;
int debugAutoPan = 0;
int nathanPrint = 0;

float /*pitch, Gyro_Pitch_angle,*/ pitch_setpoint = 0.0f, pitch_Error_last = 0.0f,  pitch_angle_correction;
float /*roll,  Gyro_Roll_angle,*/  roll_setpoint  = 0.0f,  roll_Error_last = 0.0f,   roll_angle_correction;
float /*yaw,   Gyro_Yaw_angle,*/   yaw_setpoint   = 0.0f,   yaw_Error_last = 0.0f,    yaw_angle_correction;

//float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

static int printcounter = 0;

static int rollcounter=0;

float Output[EULAR];

float CameraOrient[EULAR];
float AccAngleSmooth[EULAR];

float AccData[NUMAXIS]  = {0.0f, 0.0f, 0.0f};
float GyroData[NUMAXIS] = {0.0f, 0.0f, 0.0f};

float Step[NUMAXIS]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXIS] = {0.0f, 0.0f, 0.0f};

void roll_Nathan(void)
{
	roll_rotate();
}

void roll_rotate(void)
{
	float f = M_TWOPI*rollcounter/100.0;
	Output[ROLL] = f;
	SetRollMotor(f,configData[7]);

	if(nathanPrint)
	{
	    	print("%8.4f\n",f);
	}
}

void pitch_Nathan(void)
{
	pitch_fixed();
}

void pitch_fixed(void) {
	float f = 0;
	Output[PITCH] = f;
	SetPitchMotor(f,configData[6]);
}


float constrain(float value, float low, float high)
{
    if (value < low)
        return low;

    if (value > high)
        return high;

    return value;
}

/*
  Limits the Pitch angle
*/
float Limit_Pitch(float step, float pitch)
{
    if (pitch < PITCH_UP_LIMIT && step > 0)
    {
        step = 0.0;
    }

    if (pitch > PITCH_DOWN_LIMIT && step < 0)
    {
        step = 0.0;
    }

    return step;
}

void Fake_Orientation(float *SmoothAcc, float *Orient, float *AccData, float *GyroData, float dt)
{
	Orient[PITCH] = 0.0;
	Orient[ROLL] = 0.0;
	Orient[YAW] = 0.0;
}

//--------------------Engine Process-----------------------------//
void engineProcess(float dt)
{
    static int loopCounter;
    tStopWatch sw;

    loopCounter++;
    LEDon();
    DEBUG_LEDoff();

    StopWatchInit(&sw);


    Fake_Orientation(AccAngleSmooth, CameraOrient, AccData,GyroData,dt);

    // Meaningless stopwatch intervals
    unsigned long tGyroGet = StopWatchLap(&sw);
    unsigned long tAccGet = StopWatchLap(&sw);
    unsigned long tAccAngle = StopWatchLap(&sw);
    unsigned long tCalc = StopWatchLap(&sw);

    rollcounter++;

    pitch_Nathan();
    roll_Nathan();
// Responsible for calling Set[Axis]Motor(phi,power) for each motor
//    pitch_PID(); // 500Hz
//    roll_PID();
//    yaw_PID();

    unsigned long tPID = StopWatchLap(&sw);
    unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;

    //if (printcounter >= 500 || dt > 0.0021)
    if (printcounter >= 500) // should be executed at about 1Hz
    {
        if (debugPrint)
        {
            print("Loop: %7d, I2CErrors: %d, angles: roll %7.2f, pitch %7.2f, yaw %7.2f\r\n",
                  loopCounter, I2Cerrorcount, Rad2Deg(CameraOrient[ROLL]),
                  Rad2Deg(CameraOrient[PITCH]), Rad2Deg(CameraOrient[YAW]));
        }

        if (debugSense)
        {
            print(" dt %f, AccData: %8.3f | %8.3f | %8.3f, GyroData %7.3f | %7.3f | %7.3f \r\n",
                  dt, AccData[X_AXIS], AccData[Y_AXIS], AccData[Z_AXIS], GyroData[X_AXIS], GyroData[Y_AXIS], GyroData[Z_AXIS]);
        }

        if (debugPerf)
        {
            print("idle: %5.2f%%, time[µs]: attitude est. %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n",
                  GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
        }

        if (debugRC)
        {
            print(" RC2avg: %7.2f |  RC3avg: %7.2f |  RC4avg: %7.2f | RStep:%7.3f  PStep: %7.3f  YStep: %7.3f\r\n",
                  RCSmooth[ROLL], RCSmooth[PITCH], RCSmooth[YAW], Step[ROLL], Step[PITCH], Step[YAW]);
        }

        if (debugOrient)
        {
            print("Roll_setpoint:%12.4f | Pitch_setpoint:%12.4f | Yaw_setpoint:%12.4f\r\n",
                  roll_setpoint, pitch_setpoint, yaw_setpoint);
        }

        if (debugCnt)
        {
            print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n",
                  MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                  MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                  IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
                  usbOverrun());
        }

//        if (debugAutoPan)
//        {
//            print("Pitch_output:%3.2f | Roll_output:%3.2f | Yaw_output:%3.2f | centerpoint:%4.4f\n\r",
//                  Output[PITCH],
//                  Output[ROLL],
//                  Output[YAW],
//                  centerPoint);
//        }

        printcounter = 0;
    }

    LEDoff();
}


void roll_PID(void)
{
    float Error_current = roll_setpoint + CameraOrient[ROLL] * 1000.0;
    float KP = Error_current * ((float)configData[1] / 1000.0);
    float KD = ((float)configData[4] / 100.0) * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

    Output[ROLL] = KD + KP;
    SetRollMotor(KP + KD, configData[7]);
}

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + CameraOrient[PITCH] * 1000.0;
    float KP = Error_current * ((float)configData[0] / 1000.0);
    float KD = ((float)configData[3] / 100.0) * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    Output[PITCH] = KD + KP;

    // phi = KP+KD
    // power = pitchPWR
    SetPitchMotor(KP+KD, configData[6]);
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + CameraOrient[YAW] * 1000.0;
    float KP = Error_current * ((float)configData[2] / 1000.0);
    float KD = ((float)configData[5] / 100.0) * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    Output[YAW] = KD + KP;
    SetYawMotor(KP + KD, configData[8]);
}
