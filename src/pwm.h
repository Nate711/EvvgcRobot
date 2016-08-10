/*
 *  pwm.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

#ifndef PWM_H_
#define PWM_H_

#define PWM_PERIODE 1000

typedef enum
{
    ROLL,
    PITCH,
    YAW,
    NUMAXIS
} tAxis;

extern int MaxCnt[NUMAXIS];
extern int MinCnt[NUMAXIS];
extern int IrqCnt[NUMAXIS];

extern int timer_4_5_deadtime_delay;
extern float testPhase;

void MaxCntClear(void);
void SetRollMotor(float phi, int power);
void SetPitchMotor(float phi, int power);
void SetYawMotor(float phi, int power);

void PWMOff(void);
void PWMConfig(void);

#endif /* PWM_H_ */
