//*****************************************************************************
// pwm.h
// 6.29.2016
//
// function prototype for pwm
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
//*****************************************************************************


#include "Energia.h"
#include "inc\hw_memmap.h"
#include "inc\hw_types.h"
#include "inc\hw_ints.h"
#include "inc\hw_timer.h"

#include "driverlib\debug.h"
#include "driverlib\interrupt.h"
#include "driverlib\timer.h"
#include "driverlib\rom.h"
#include "driverlib\rom_map.h"
#include "driverlib\timer.h"
#include "driverlib\utils.h"
#include "driverlib\prcm.h"
#include "driverlib\pin.h"

#define TIMER_TICK              80000000UL    // 80 MHz = 8000000 cycles / sec 
#define INVERSION_YES            0            // PWM SIGNAL INVERSION
#define INVERSION_NO             1


class Timer {
  private:
    unsigned long _ulBase; // TIMER_BASE
    unsigned long _ulTimer; // TIMER
    unsigned char _preScale ;
    unsigned char _inversion ;
    unsigned long _timerIntervalReload;
    unsigned long _matchSetValue;
    void (*intCallBack)();
  
  public: 
   unsigned long timerIntervalReload;
   //unsigned long  matchSetValue;
   Timer(unsigned long ulBase, unsigned long ulTimer, unsigned long ulInversion); // Constructor
   unsigned long getTimerIntervalReload(void); //getter method 
   unsigned long getMatchSetValue(void); //getter method 

   void configure(unsigned long ulConfig); //Configure timer
   void prescaleSet(); // configure clock scale
   void controlLevel(); //inverting the timer output if required
   void setPeriod(float fPeriod);  // Set timer period in mili-second
   void setDuty(float fDuty);     // Set timer duty in mili-second
   void matchSet(unsigned long matchSetValue); // Match value set to reflect duty cycle
   void enable(void); // Enable Timer 
   void pinMuxAndPeripheralClockConfigure(unsigned long ucPRCM_Timer, unsigned char ucPin, unsigned char ucPinMode); // Configure pin mux
   void updateDutyCycle(unsigned long dutyCycle);  // Update duty cycle
   // Interrupt
   void setInterrupt(unsigned long ulEvent, unsigned long ulControlEvent, void (*intCallBack)());
   void setCallBack(unsigned long ulBase, unsigned long ulTimer, unsigned long ulIntFlags, void (*intCallBack)());
   unsigned long intStatus(unsigned long ulBase);
   void intClear(unsigned long ulBase, unsigned long ulflag);
   void intEnable(unsigned long ulBase, unsigned long ulflag);
   unsigned long readRegister(unsigned long ulBase, unsigned char shift);
   // software reset
   void softReset(unsigned long long  ullTicks);

 };
