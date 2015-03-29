/* 
 * File:   motor.h
 * Author: 
 *
 * Created on March 28, 2015, 9:49 PM
 */

#ifndef MOTOR_H
#define	MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <plib.h>
#include "config.h"
#include "stdtypes.h"
    
int DesiredTimeLeft, DesiredTimeRight;// Time between movement interrupts
//int DirectionLeft = 0, DirectionRight = 0;// 0 = Forward, 1 = Backward

void Motor_Left_Stop();
void Motor_Right_Stop();
void Motors_Stop();

void Motor_Left_Backward();
void Motor_Right_Backward();
void Motors_Backward();

void Motor_Left_Forward();
void Motor_Right_Forward();
void Motors_Forward();

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

