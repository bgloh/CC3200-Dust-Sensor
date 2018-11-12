/*
 * MyAdc.cpp
 * A library for ADC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "MyAdc.h"

cc3200Adc::cc3200Adc(uint8_t adcChannel){
	_adcChannel = adcChannel;
        if (adcChannel == ADC_CH_3)
        _adcInputPin = PIN_60;

	
}

void cc3200Adc::begin(){
	 // Configure Pin mux
  PinTypeADC(_adcInputPin,PIN_MODE_255);
  // Configure ADC timer which is used to timestamp the ADC data samples
  ADCTimerConfig(ADC_BASE,131072);
  //Enable ADC timer which is used to timestamp the ADC data samples ; 2^17 = 131072
  ADCTimerEnable(ADC_BASE);
  // Enable ADC module
  ADCEnable(ADC_BASE);
  // Enable ADC channel
 ADCChannelEnable(ADC_BASE, _adcChannel);
 
}

uint16_t cc3200Adc::getRawAdcValue(void){

 uint16_t sample, adcCount;
 
 // wait until ad conversion is complete
   while(!ADCFIFOLvlGet(ADC_BASE, _adcChannel));
   // read adc result
   sample = ADCFIFORead(ADC_BASE, _adcChannel);
   // // adc result is in 14 bits. shift 2 bits 
   adcCount = ((sample >> 2 ) & 0x0FFF);
   return adcCount;
 
}
unsigned long cc3200Adc::getTimeStamp(void){
  unsigned long timeStamp;
  timeStamp = ADCTimerValueGet(ADC_BASE);
  return timeStamp;

}

double cc3200Adc::getAdcValue(){

 return (double)((getRawAdcValue()*1.47)/4096.0);
 
}

