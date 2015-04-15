/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include <stdio.h>
#include <math.h>
#include "stdtypes.h"
#include "config.h"
#include "motor.h"
#include "spi.h"
#include "util.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable

#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/*
#define Kp 100
#define Kpslow 50
#define Kpsuperslow 10
 
#define Ki 50
#define Kislow 25
#define Kisuperslow 5

#define Kd 25
#define Kdslow 12
#define Kdsuperslow 1//*/

#define	stPressed	1
#define	stReleased	0

#define	cstMaxCnt	10 // number of consecutive reads required for
					   // the state of a button to be updated

struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button
					// state
};

/* ------------------------------------------------------------ */
/*			PmodCLS Instructions                    */
/* ------------------------------------------------------------ */

static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0};
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0};
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0};

static	char szCursorPos[] = {0x1B, '[', '1', ';', '0', 'H', 0};
static	char szCursorPosC2[] = {0x1B, '[', '0', ';', '0', 'H', 0};
static	char szCursorPosC3[] = {0x1B, '[', '1', ';', '0', 'H', 0};

/* ------------------------------------------------------------ */
/*			Global Variables                        */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;

// State Machine
typedef enum RobotState {Start, WallCheck, ForwardFast, ForwardMedium, ForwardSlow, TurnLeft45,
        TurnLeft90, TurnRight45, TurnRightSmall, TurnLeftSmall, MotorChange} RobotState;
//Motors interrupts variables
int TimerCounter, IC2Counter, IC3Counter, BaseC2, BaseC3, C2, C3, C2Counter = 0, C3Counter = 0;
int TimesC2[10], TimesC3[10];
//PID variables
float error2, error3, lasterror2, lasterror3, sumerror2, sumerror3, interror2 = 0, interror3 = 0, temp2, temp3;
float error2v[1000], error3v[1000];
int errorcount2 = 0, errorcount3 = 0;
int Kp = 100, Ki = 50, Kd = 25, Kpslow = 50, Kislow = 25, Kdslow = 12, Kpsuperslow = 18, Kisuperslow = 10, Kdsuperslow = 5;
//Desired times between interrupts
//int DesiredTime2, DesiredTime3;
//Stopped motors check variables
int LastInt2, LastInt3, Timer5LastInt2, Timer5LastInt3, LastIntCount2=0, LastIntCount3=0;

//ADC variables
int ADCValue_0[25], ADCValue_1[25], ADCValue_2[25], ADCValue_3[25]; //will store ADC results in these variables
int adc_index = 0;
int adc_counter = 0;
int ADC0avg, ADC1avg, ADC2avg, ADC3avg, ADC0sum, ADC1sum, ADC2sum, ADC3sum;

// Sensor variables
int RZ;
int LZ;
/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);
void	Wait_ms(WORD ms);
void    CheckStoppedWheel2(int delay);
void    CheckStoppedWheel3(int delay);
void    ErrorCalcPID2();
void    ErrorCalcPID3();
float   LeftSensorFormula(int Sensor);
float   RightSensorFormula(int Sensor);
void    Zone(int RSensor, int LSensor);
/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */
/***	Timer5Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
**              100 us per interrupt
*/

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{
        // PID for Right Wheel
        if (DesiredTimeRight < 20 || DesiredTimeRight > 110){
            error3 = 0;
            lasterror3 = 0;
            errorcount3 = 0;
            sumerror3 = 0;
            temp3 = 0;
            OC3R = temp3;
            OC3RS = temp3;
            C3Counter = 0;
            BaseC3 = 0;
            C3 = 0;
        }
        else if (DesiredTimeRight <= 50) //When its Fast
        {
            if (C3Counter++ > 299)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt3 == LastInt3)
                {
                    if (LastIntCount3 < 10)
                    {
                        LastIntCount3++;
                    }
                    else
                    {
                        BaseC3 = 120;
                    }
                } else {
                    LastIntCount3 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID3();
                // Calculate new speed
                temp3 = (float)Kp*error3 + sumerror3 + (float)Kd*(error3-lasterror3);
                //Limit temp values
                if (temp3 > 9999)
                {
                    temp3 = 9999;
                }
                else if (temp3 < 0)
                {
                    temp3 = 0;
                }
                // Set new motor speed
                OC3R 	= temp3;
                OC3RS	= temp3;
                C3Counter = 0;
                errorcount3++;
                if (errorcount3 > 299)
                    errorcount3 = 0;
               }
        }
        else if (DesiredTimeRight > 50 && DesiredTimeRight < 95) //When its slow
        {
            if (C3Counter++ > 99)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt3 == LastInt3)
                {
                    if (LastIntCount3 < 25)
                    {
                        LastIntCount3++;
                    }
                    else
                    {
                        BaseC3 = 120;
                    }
                } else {
                    LastIntCount3 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID3();
                // Calculate new speed
                temp3 = (float)Kpslow*error3 + sumerror3 + (float)Kdslow*(error3-lasterror3);
                //Limit temp values
                if (temp3 > 9999)
                {
                    temp3 = 9999;
                }
                else if (temp3 < 0)
                {
                    temp3 = 0;
                }
                // Set new motor speed
                OC3R 	= temp3;
                OC3RS	= temp3;
                C3Counter = 0;
                errorcount3++;
                if (errorcount3 > 299)
                    errorcount3 = 0;
                }
        }
        else //110 is the max
        {
            if (C3Counter++ > 99)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt3 == LastInt3)
                {
                    if (LastIntCount3 < 25)
                    {
                        LastIntCount3++;
                    }
                    else
                    {
                        BaseC3 = 120;
                    }
                } else {
                    LastIntCount3 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID3();
                // Calculate new speed
                temp3 = (float)Kpsuperslow*error3 + sumerror3 + (float)Kdsuperslow*(error3-lasterror3);
                //Limit temp values
                if (temp3 > 9999)
                {
                    temp3 = 9999;
                }
                else if (temp3 < 0)
                {
                    temp3 = 0;
                }
                // Set new motor speed
                OC3R 	= temp3;
                OC3RS	= temp3;
                C3Counter = 0;
                errorcount3++;
                if (errorcount3 > 299)
                    errorcount3 = 0;
                }
        }

        // PID for Left Wheel
        if (DesiredTimeLeft < 20 || DesiredTimeLeft > 110){
            error2 = 0;
            lasterror2 = 0;
            errorcount2 = 0;
            sumerror2 = 0;
            temp2 = 0;
            OC2R = temp2;
            OC2RS = temp2;
            C2Counter = 0;
            BaseC2 = 0;
            C2 = 0;
        }
        else if (DesiredTimeLeft <= 50) //When its Fast
        {
            if (C2Counter++ > 299)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt2 == LastInt2)
                {
                    if (LastIntCount2 < 50)
                    {
                        LastIntCount2++;
                    }
                    else
                    {
                        BaseC2 = 120;
                    }
                } else {
                    LastIntCount2 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID2();
                // Calculate new speed
                temp2 = (float)Kp*error2 + sumerror2 + (float)Kd*(error2-lasterror2);
                //Limit temp values
                if (temp2 > 9999)
                {
                    temp2 = 9999;
                }
                else if (temp2 < 0)
                {
                    temp2 = 0;
                }
                // Set new motor speed
                OC2R	= temp2;
                OC2RS	= temp2;
                C2Counter = 0;
                errorcount2++;
                if (errorcount2 > 299)
                    errorcount2 = 0;
            }
        }
        else if (DesiredTimeLeft > 50 && DesiredTimeLeft < 95)//When its slow
        {
            if (C2Counter++ > 99)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt2 == LastInt2)
                {
                    if (LastIntCount2 < 25)
                    {
                        LastIntCount2++;
                    }
                    else
                    {
                        BaseC2 = 120;
                    }
                } else {
                    LastIntCount2 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID2();
                // Calculate new speed
                temp2 = (float)Kpslow*error2 + sumerror2 + (float)Kdslow*(error2-lasterror2);
                //Limit temp values
                if (temp2 > 9999)
                {
                    temp2 = 9999;
                }
                else if (temp2 < 0)
                {
                    temp2 = 0;
                }
                // Set new motor speed
                OC2R	= temp2;
                OC2RS	= temp2;
                C2Counter = 0;
                errorcount2++;
                if (errorcount2 > 299)
                    errorcount2 = 0;
                }
        }
        else //110 is the max
        {
            if (C2Counter++ > 99)
            {
                //Check if wheel is stopped
                /*if (Timer5LastInt2 == LastInt2)
                {
                    if (LastIntCount2 < 25)
                    {
                        LastIntCount2++;
                    }
                    else
                    {
                        BaseC2 = 120;
                    }
                } else {
                    LastIntCount2 = 0;
                }*/
                //Calculate Error
                ErrorCalcPID2();
                // Calculate new speed
                temp2 = (float)Kpsuperslow*error2 + sumerror2 + (float)Kdsuperslow*(error2-lasterror2);
                //Limit temp values
                if (temp2 > 9999)
                {
                    temp2 = 9999;
                }
                else if (temp2 < 0)
                {
                    temp2 = 0;
                }
                // Set new motor speed
                OC2R	= temp2;
                OC2RS	= temp2;
                C2Counter = 0;
                errorcount2++;
                if (errorcount2 > 299)
                    errorcount2 = 0;
                }
        }
        
        Timer5LastInt2 = LastInt2;
        Timer5LastInt3 = LastInt3;
        TimerCounter++;
        mT5ClearIntFlag();
}

/* ADC ISR */
void __ISR(_ADC_VECTOR, ipl3) _ADC_IntHandler(void){

        if (adc_index >= 10)
            {  adc_index = 0;}
    	/*
        //if (AD1CHS == 0x00) //Right Front
        //{
            ADCValue_0[adc_index] = ADC1BUF0;
            ADC0sum += ADCValue_0[adc_index];
            if (adc_counter % 10 == 0)
            {
                ADC0avg = ADC0sum/10;
                ADC0sum = 0;
                //Set the next channel to be sampled
                //AD1CHSCLR = (7 << 16);
                //AD1CHSSET = (1 << 16);
            }
            
        //}
        //else if (AD1CHS == 0x01) //Front
        //{
            ADCValue_1[adc_index] = ADC1BUF1;
            ADC1sum += ADCValue_1[adc_index];
            if (adc_counter % 10 == 0)
            {
                ADC1avg = ADC1sum/10;
                ADC1sum = 0;
                //AD1CHSCLR = (7 << 16);
                //AD1CHSSET = (1 << 17);
            }
        //}
        //else if (AD1CHS == 0x02) //Right Back
        //{
            ADCValue_2[adc_index] = ADC1BUF2;
            ADC2sum += ADCValue_2[adc_index];
            if (adc_counter % 10 == 0)
            {
                ADC2avg = ADC2sum/10;
                ADC2sum = 0;
                //AD1CHSCLR = (7 << 16);
                //AD1CHSSET = (3 << 16);
            }
        //}
        //else //if (AD1CHS == 0x03) //Left
        //{
            ADCValue_3[adc_index] = ADC1BUF3;
            ADC3sum += ADCValue_3[adc_index];
            if (adc_counter % 10 == 0)
            {
                ADC3avg = ADC3sum/10;
                ADC3sum = 0;
                //AD1CHSCLR = (7 << 16);
            }
        //}*/

        ADCValue_0[adc_index] = ADC1BUF0;
        ADC0sum += ADCValue_0[adc_index];
        ADCValue_1[adc_index] = ADC1BUF1;
        ADC1sum += ADCValue_1[adc_index];
        ADCValue_2[adc_index] = ADC1BUF2;
        ADC2sum += ADCValue_2[adc_index];
        ADCValue_3[adc_index] = ADC1BUF3;
        ADC3sum += ADCValue_3[adc_index];
        if (adc_counter % 10 == 0)
        {
            ADC0avg = ADC0sum/10;
            ADC1avg = ADC1sum/10;
            ADC2avg = ADC2sum/10;
            ADC3avg = ADC3sum/10;
            adc_index = 0;
            ADC0sum = 0;
            ADC1sum = 0;
            ADC2sum = 0;
            ADC3sum = 0;
        }
        adc_index++;
        adc_counter++;
    	AD1CON1CLR = 0x00000001; //clear the DONE bit
    	IFS1CLR = (1 << 1); //clear interrupt flag for 'ADC1 Conversion Complete'
    	//AD1CON1SET = 0x0002; //start sampling ??
	
	
}

/***	IC2Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for IC2 interrupt.
*/
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl7) _IC2_IntHandler(void)
{
        unsigned int buf;
        int i;
        INTDisableInterrupts();
        BaseC2 = TimerCounter - C2;
        C2 = TimerCounter;
        LastInt2 = TimerCounter;
        INTEnableInterrupts();
        
        buf = IC2BUF;           // Read the buffer to clear space
        IFS0CLR	= ( 1 << 9 );	// Clear interrupt flag for Input Capture 2

}

/***	IC3Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for IC3 interrupt.
*/
void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl7) _IC3_IntHandler(void)
{
	unsigned int buf;
        int i;
        INTDisableInterrupts();
        BaseC3 = TimerCounter - C3;
        C3 = TimerCounter;
        LastInt3 = TimerCounter;
        INTEnableInterrupts();

        buf = IC3BUF;
        IFS0CLR	= ( 1 << 13 );	// clear interrupt flag for Input Capture 3
}

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/

int main(void)
{
        char str2[12], str3[12];
        int n2, n3;
        float LeftSensor, RightSensor;
        RobotState state = Start , nextstate, laststate = Start;
        //RobotState state;
	DeviceInit();
        INTDisableInterrupts();
	AppInit();
        INTEnableInterrupts();
        state = Start;

        
        
        INTDisableInterrupts();
	//write to PmodCLS
        SpiEnable();
        SpiPutBuff(szClearScreen, 3);//Clear everything and put cursor on line 0 col 0
        DelayMs(4);
        SpiPutBuff(szBacklightOn, 4);//Back light
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);//Hide the cursor
        DelayMs(4);
        SpiDisable();
	prtLed1Set	= ( 1 << bnLed1 );
	INTEnableInterrupts();
        DelayMs(2000);

	while (fTrue)
	{
                INTDisableInterrupts();
                LeftSensor = LeftSensorFormula(ADC1avg);
                RightSensor = RightSensorFormula(ADC0avg);
                INTEnableInterrupts();
                // Check the Sensors
                Zone(RightSensor, LeftSensor);

                n2 = sprintf(str2, "%5d %5d",ADC0avg, ADC1avg);//Left distance sensor
                n3 = sprintf(str3, "%5d %5d ", ADC2avg, ADC3avg);//Right distance sensor

                SpiEnable();
                SpiPutBuff(szCursorPosC2, 6);//First counter
                DelayMs(4);
                SpiPutBuff(str2, n2);
                DelayMs(4);
                SpiPutBuff(szCursorPosC3, 6);//First counter
                DelayMs(4);
                SpiPutBuff(str3, n3);
                SpiDisable();

		INTEnableInterrupts();

                
                switch (state)
                {
                    case Start:
                        Motors_Stop();
                        state = WallCheck;
                        break;
                    case WallCheck:
                        if (RZ == 3 && LZ == 3)
                        {
                            nextstate = ForwardFast;
                        }
                        else if (RZ == 3 && LZ == 2)
                        {
                            nextstate = ForwardMedium;
                        }
                        else if (RZ == 3 && LZ == 1)
                        {
                            nextstate = TurnLeftSmall;
                        }
                        else if (RZ == 3 && LZ == 0)
                        {
                            nextstate = TurnLeft45;
                        }
                        else if (RZ == 2 && LZ == 3)
                        {
                            nextstate = TurnRightSmall;
                        }
                        else if (RZ == 2 && LZ == 2)
                        {
                            nextstate = TurnRightSmall;
                        }
                        else if (RZ == 2 && LZ == 1)
                        {
                            nextstate = ForwardSlow;
                        }
                        else if (RZ == 2 && LZ == 0)
                        {
                            nextstate = TurnLeft45;
                        }
                        else if (RZ == 1 && LZ == 3)
                        {
                            nextstate = ForwardFast;
                        }
                        else if (RZ == 1 && LZ == 2)
                        {
                            nextstate = ForwardMedium;
                        }
                        else if (RZ == 1 && LZ == 1)
                        {
                            nextstate = ForwardSlow;
                        }
                        else if (RZ == 1 && LZ == 0)
                        {
                            nextstate = TurnLeft45;
                        }
                        else if (RZ == 0 && LZ == 3)
                        {
                            nextstate = TurnLeftSmall;
                        }
                        else if (RZ == 0 && LZ == 2)
                        {
                            nextstate = TurnLeftSmall;
                        }
                        else if (RZ == 0 && LZ == 1)
                        {
                            nextstate = TurnLeftSmall;
                        }
                        else if (RZ == 0 && LZ == 0)
                        {
                            nextstate = TurnLeft45;
                        }
                        state = MotorChange;
                        break;
                    case MotorChange:
                        if (laststate != nextstate)
                        {
                            //BaseC3 = 120;
                            //BaseC2 = 120;
                            if (nextstate == ForwardFast)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 100;
                            }
                            else if (nextstate == ForwardMedium)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 100;
                            }
                            else if (nextstate == ForwardSlow)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 100;
                            }
                            else if (nextstate == TurnRight45)
                            {
                                DesiredTimeLeft = 80;
                                DesiredTimeRight = 100;
                            }
                            else if (nextstate == TurnLeft45)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 80;
                            }
                            else if (nextstate == TurnLeft90)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 70;
                            }
                            else if (nextstate == TurnRightSmall)
                            {
                                DesiredTimeLeft = 90;
                                DesiredTimeRight = 100;
                            }
                            else if (nextstate == TurnLeftSmall)
                            {
                                DesiredTimeLeft = 100;
                                DesiredTimeRight = 90;
                            }
                        }
                        laststate = nextstate;
                        state = WallCheck;
                        break;
                }
                
	}  //end while
}  //end main

/*
 *  Initialization Functions
 */

void DeviceInit() {

	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward

	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= ( 1 << 2 ) | ( 1 << 1 );	// PWM - Select Timer2 and Enable PWM(110)
	OC2R	= 0;
	OC2RS	= 0;

	// Configure Output Compare 3.
	OC3CON  = ( 1 << 2 ) | ( 1 << 1 );	// PWM - Select Timer2 and Enable PWM(110)
	OC3R	= 0;
	OC3RS	= 0;

	// Configure Timer 2. (PWM)
	TMR2	= 0;									// clear timer 2 count
	PR2     = 9999;

	// Configure Timer 3. (Sensors)
	TMR3	= 0;
	PR3	= 299;

	// Start timers and output compare units.
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
        T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 ) | ( 1 << TCKPS21);	// timer 2 prescale = 8
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8

	// Configure Timer 5.
	TMR5	= 0;
	PR5     = 99; // period match every 100 us
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
	IFS0CLR = ( 1 << 20);
	IEC0SET	= ( 1 << 20);

	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8

        //Configure IC2 and IC3
        IC2CONSET = ( 1 << 15 ) | ( 2 << 0 );// Set IC ON and FallingEdgeTrigger
        IC3CONSET = ( 1 << 15 ) | ( 2 << 0 );// Set IC ON and FallingEdgeTrigger
        //Set IC2 and IC3 priority
        IPC2SET = ( 7 << 10 ) | ( 2 << 8 );// IC2 Interrupt priority 7 and subpriority 2
        IPC3SET = ( 7 << 10 ) | ( 2 << 8 );// IC3 Interrupt priority 7 and subpriority 2
        //Enable IC2 and IC3 interrupts
        IEC0SET = ( 1 << 13 ) | ( 1 << 9 );

        /******** ADC Config *********/

        // Configure ADC inputs
        TRISBSET = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3); //Set JJ-01 (AN0) and JJ-02 (AN1) as digital inputs
        AD1PCFG = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3); //Set AN0 and AN1 as analog inputs
                
        // Configure ADC Control Registers
        AD1CON1SET = (2 << 5)|(1 << 2);
        // ^ Set bits 5-7 which sets the ADC's auto convert
        // ^ Set bit 2 which makes the ADC sample immediately after previous conversion.(Timer 3)
        AD1CON2SET = (1 << 10) | (3 << 2);//(15 << 2);
        // ^ Trigger ADC interrupt after every 16th conversion.
        AD1CON3SET = (15 << 0);
        // ^ Since bit 15 is cleared, ADC clock source is PBclock (peripheral bus)
        // Bits 0-7 determine how to scale ADC clock (see pg 13 of ADC chapter).
        AD1CHSSET = 0x0000; //Input Select Register
        AD1CSSLSET = 0x000F; //Set AN0, AN1, AN2 and AN3 analog inputs in digital mode

        // Configure ADC interrupt
        IPC6SET =  (1 << 27) | (1 << 26); // ADC Int. Priority = 3, subpri = 0.
        IFS1CLR = 2; //Clear interrupt flag
        IEC1SET = 2; //Enable ADC Interrupt

        //Set PBC and ADC clock prescaller
        OSCCON ^= (1<<19)|(1<<20);
        AD1CON1SET = (1 << 15);  //Turn ADC on
        
	//enable SPI
	SpiInit();

	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
}

void AppInit() {
    
    TimerCounter = 0;
    IC2Counter = 0;
    IC3Counter = 0;
    error3 = 0;
    lasterror3 = 0;
    error2 = 0;
    lasterror2 = 0;
    DesiredTimeLeft = 0;
    DesiredTimeRight = 0;
    BaseC3 = TimerCounter;
    BaseC2 = TimerCounter;
    Motors_Forward();
    LastInt2 = 0;
    LastInt3 = 0;
    
}

/*
 *  PID Functions
 */

void CheckStoppedWheelLeft(int delay) {
    /*if (LastIntCount2 > 10){
        MtrCtrlFwd();
        //OC2R = 9999;
        //OC2RS = 9999;
        BaseC2 = 50;
    }//*/
}

void CheckStoppedWheelRight(int delay) {
    /*if (LastIntCount3 > 10){
        MtrCtrlFwd();
        //OC3R = 9999;
        //OC3RS = 9999;
        BaseC3 = 50;
    }//*/
}

void ErrorCalcPID2() {
    // Set previous error
    lasterror2 = error2;
    // Calculate Error
    error2 = BaseC2 - DesiredTimeLeft;
    error2v[errorcount2] = error2;
    // Calculate Integral part
    sumerror2 += (float)Kisuperslow*error2;
    // Bound sumerror value
    if (sumerror2 > 20000)
    {
        sumerror2 = 20000;
    }
    else if (sumerror2 < -20000)
    {
        sumerror2 = -20000;
    }
}

void ErrorCalcPID3() {
    // Set previous error
    lasterror3 = error3;
    // Calculate Error
    error3 = BaseC3 - DesiredTimeRight;
    error3v[errorcount3] = error3;
    // Calculate Integral part
    sumerror3 += (float)Kisuperslow*error3;
    // Bound sumerror value
    if (sumerror3 > 20000)
    {
        sumerror3 = 20000;
    }
    else if (sumerror3 < -20000)
    {
        sumerror3 = -20000;
    }
}

/*
 *  Sensors Functions
 */

/* Returns the distance from the left sensor */
float LeftSensorFormula(int SensorValue)
{
    return (float)(7525.3*pow(SensorValue,-1.157));
}

/* Returns the distance from the right sensor*/
float RightSensorFormula(int SensorValue)
{
    return (float)(46.5511*exp(-0.0044*SensorValue));
}

void Zone(int RSensor, int LSensor)
{
    if (RSensor < 5) // First zone
    {
        RZ = 0;
    }
    else if (RSensor < 15) // Second zone
    {
        RZ = 1;
    }
    else if (RSensor < 40) // Third zone
    {
        RZ = 2;
    }
    else // Out of range
    {
        RZ = 3;
    }

    if (LSensor < 15) // First zone
    {
        LZ = 0;
    }
    else if (LSensor < 25) // Second zone
    {
        LZ = 1;
    }
    else if (LSensor < 40) // Third zone
    {
        LZ = 2;
    }
    else // Out of range
    {
        LZ = 3;
    }
}
