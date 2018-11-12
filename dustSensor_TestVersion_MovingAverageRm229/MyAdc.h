/*
 * adc.h
 * A library for adc 1.0
 *
 */
 


#ifndef MyAdc_H
#define MyAdc_H

// Driverlib includes
#ifndef _ENERGIA_H
#include "Energia.h"
#endif
#include "driverlib\utils.h"
#include "inc\hw_memmap.h"
#include "inc\hw_common_reg.h"
#include "inc\hw_types.h"
#include "inc\hw_adc.h"
#include "inc\hw_ints.h"
#include "inc\hw_gprcm.h"
#include "driverlib\rom.h"
#include "driverlib\rom_map.h"
#include "driverlib\interrupt.h"
#include "driverlib\prcm.h"
#include "driverlib\pin.h"
#include "driverlib\adc.h"



class cc3200Adc{
	public: 
		cc3200Adc(uint8_t adcChannel = ADC_CH_3); // default adc channel : ADC3

		void           begin();
                uint16_t       getRawAdcValue(void);
		double         getAdcValue();
                unsigned long   getTimeStamp(void);

	private:
  		uint8_t _adcChannel;
                uint8_t _adcInputPin;
              };

#endif
