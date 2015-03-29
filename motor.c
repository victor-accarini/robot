
#include "motor.h"

void Motor_Left_Stop()
{
    DesiredTimeLeft = 0;
    OC2R  = 0;
    OC2RS = 0;
}
void Motor_Right_Stop()
{
    DesiredTimeRight = 0;
    OC3R  = 0;
    OC3RS = 0;
}
void Motors_Stop(){
    DesiredTimeLeft = 0;
    DesiredTimeRight = 0;
    OC2R  = 0;
    OC2RS = 0;
    OC3R  = 0;
    OC3RS = 0;
}

void Motor_Left_Backward()
{
    Motor_Left_Stop();
    trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
    prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// backward
}
void Motor_Right_Backward()
{
    Motor_Right_Stop();
    trisMtrRightDirClr	= ( 1 << bnMtrRightDir );
    prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// backward
}
void Motors_Backward()
{
    Motors_Stop();
    trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
    prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// backward
    trisMtrRightDirClr	= ( 1 << bnMtrRightDir );
    prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// backward
}

void Motor_Left_Forward()
{
    Motor_Left_Stop();
    trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
    prtMtrLeftDirSet	= ( 1 << bnMtrLeftDir );	// forward
}
void Motor_Right_Forward()
{
    Motor_Right_Stop();
    trisMtrRightDirClr	= ( 1 << bnMtrRightDir );
    prtMtrRightDirClr	= ( 1 << bnMtrRightDir );	// forward
}
void Motors_Forward()
{
    Motors_Stop();
    trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
    prtMtrLeftDirSet	= ( 1 << bnMtrLeftDir );	// forward
    trisMtrRightDirClr	= ( 1 << bnMtrRightDir );
    prtMtrRightDirClr	= ( 1 << bnMtrRightDir );	// forward
}
