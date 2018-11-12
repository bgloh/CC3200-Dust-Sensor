//*****************************************************************************
// pwm.cpp
//
// configure the device pins for different peripheral signals
// 6.29.2016
// 
// function definition for pwm

// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// // The PWM works based on the following settings:
//     Timer reload interval -> determines the time period of one cycle
//     Timer match value -> determines the duty cycle 
//                          range [0, timer reload interval]
// The computation of the timer reload interval and dutycycle granularity
// is as described below:
// Timer tick frequency = 80 Mhz = 80000000 cycles/sec
// For a time period of 0.5 ms, 
//      Timer reload interval = 80000000/2000 = 40000 cycles
// MAXIMUM PERIOD IS 2^24 / 80E6 = 0.209 SEC ==> ABOUT 200 ms

// RED LED: GPIO 9 (P64) :     TIMERA2 (TIMER_B) PWM_5
// GREEM LED: GPI0 11 (P2) :   TIMERA3 (TIMER_A) PWM_7
// YELLOW : GPIO 10 (P1) :     TIMERA3 (TIMER_B) PWM_6 

// To support steps of duty cycle update from [0, 255]
//      duty cycle granularity = ceil(40000/255) = 157
// Based on duty cycle granularity,
//      New Timer reload interval = 255*157 = 40035
//      New time period = 0.5004375 ms
//      Timer match value = (update[0, 255] * duty cycle granularity)
//*********************************************************************

#include "pwm.h"

// Constructor
// fPeriod : period in ms, fDuty : Duty in ms
Timer::Timer(unsigned long ulBase, unsigned long ulTimer, unsigned long ulInversion)
{
  
  _ulBase = ulBase;
  _ulTimer = ulTimer;
  _preScale = 0 ;
  _inversion = ulInversion ;
  _timerIntervalReload = TIMER_TICK / 1000;  // default period     : 1 ms
  _matchSetValue = _timerIntervalReload / 2; // defualt duty cyle   : 50 %
 }

unsigned long Timer::getTimerIntervalReload(void)
{
     return _timerIntervalReload;
}

unsigned long Timer::getMatchSetValue(void)
{
     return _matchSetValue;
}

   void Timer::configure(unsigned long ulConfig)
{
  //Configure timer  ;
  TimerConfigure(_ulBase, ulConfig);
  //inverting the timer output if required
  TimerControlLevel(_ulBase, _ulTimer, _inversion); 

} 

void Timer::prescaleSet()
{
  // configure clock scale
  TimerPrescaleSet(_ulBase, _ulTimer, _preScale);
}

void Timer::controlLevel()
{
  //inverting the timer output if required
  TimerControlLevel(_ulBase, _ulTimer, _inversion);
} 
  // Set timer period in mili-second
void Timer::setPeriod(float fPeriod)
{
  unsigned long periodCount;
  periodCount = (TIMER_TICK * fPeriod /1000);
  _timerIntervalReload = periodCount & 0x0000FFFF;  
  _preScale = periodCount >> 16;
  TimerPrescaleSet(_ulBase, _ulTimer, _preScale);
  TimerLoadSet(_ulBase, _ulTimer, _timerIntervalReload);
}

// Set Duty in mili-second
void Timer::setDuty(float fDuty)
{
  unsigned long dutyCount;
  dutyCount = (TIMER_TICK * fDuty /1000);
  _matchSetValue = dutyCount & 0x0000FFFF;
  _preScale = dutyCount >> 16;
  TimerPrescaleMatchSet(_ulBase, _ulTimer, _preScale);
  TimerMatchSet(_ulBase, _ulTimer, _matchSetValue);
}

void Timer::matchSet(unsigned long matchSetValue)
{
  // Match value set to reflect duty cycle
  _matchSetValue = matchSetValue;
  TimerMatchSet(_ulBase, _ulTimer, _matchSetValue);
}

void Timer::enable()  
{
  // Enable Timer
  TimerEnable(_ulBase, _ulTimer);
}

// Configure pin mux
void Timer::pinMuxAndPeripheralClockConfigure(unsigned long ulPRCM_Timer, unsigned char ucPin, unsigned char ucPinMode)
{
  // Enable clock to peripheral
  PRCMPeripheralClkEnable(ulPRCM_Timer, PRCM_RUN_MODE_CLK);
  
  // Configure PIN_64 for TIMER PWM5 GT_PWM05
  PinTypeTimer(ucPin, ucPinMode);   
}
  
 // Update duty cycle
 void Timer::updateDutyCycle(unsigned long dutyCycle)
{
  TimerMatchSet(_ulBase,_ulTimer,dutyCycle);
}

// Interrupt


void Timer::setInterrupt(unsigned long ulEvent, unsigned long ulControlEvent, void (*intCallBack)())
{
 
 // unsigned long ulstatus;  
 // IntPrioritySet(INT_TIMERA2B, INT_PRIORITY_LVL_7);
 // ulstatus =  TimerIntStatus(ulBase,false);
  TimerControlEvent( _ulBase, _ulTimer, ulControlEvent);  
  TimerIntRegister(_ulBase, _ulTimer, intCallBack);
  HWREG(_ulBase + 0x08) |=  (1<<9); // enable interrupt FOR PWM EVENT
  TimerIntEnable(_ulBase,ulEvent);
 }


void Timer::setCallBack(unsigned long ulBase, unsigned long ulTimer, unsigned long ulIntFlags, void (*intCallBack)())
{
  //unsigned long   ulIntFlags = TIMER_CAPB_EVENT ;  
  TimerIntRegister  ( ulBase,  ulTimer,  intCallBack);
    TimerIntEnable  ( ulBase,  ulIntFlags);
  //  IntPrioritySet(INT_TIMERA0B, INT_PRIORITY_LVL_7);
}

unsigned long Timer::intStatus(unsigned long ulBase)
{
  unsigned long ulstatus;
  ulstatus = TimerIntStatus(ulBase,false);
  return ulstatus;
}
void Timer::intClear(unsigned long ulBase, unsigned long ulflag)
{
  TimerIntClear(ulBase, ulflag);
 }
 
 void Timer::intEnable(unsigned long ulBase, unsigned long ulflag)
{
  TimerIntEnable(ulBase,ulflag);
}

// read register
unsigned long Timer::readRegister(unsigned long ulBase, unsigned char shift)
{
  unsigned long registerVal;
 // registerVal = (*((volatile unsigned long *)(TIMERA2_BASE + 8)));
 registerVal = HWREG(ulBase + shift);
  return registerVal;
}
   
// software rest

void Timer::softReset(unsigned long long  ullTicks ) 
{
  PRCMHibernateIntervalSet  ( ullTicks ) ;
  PRCMHibernateWakeupSourceEnable  (PRCM_HIB_SLOW_CLK_CTR) ;
  PRCMHibernateEnter  ( ) ;
}




