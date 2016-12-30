// DUST SENSOR
// REVISION HISTORY BY DR. B LOH.
// 12.30.2016 : CLOCK-MODULE IS ADDED

// Moving Average of ADC voltage

// PWM + GPIO_PIN Interrupt
#include "pwm.h"
#include "MyAdc.h"
#include <Wire.h>
#include "HDC1000.h"
#include <WiFi.h>
#include <stdarg.h>

// Realtime Clock Update
#include "RTC_Library.h"
#include "NTP_WiFi.h"
// Define variables and constants
DateTime myRTC;
time_t myEpochRTC;
tm myTimeNTP, myTimeRTC;
uint32_t counter = 0;
bool flagRTC = true; // First time update of RTC

// Prototypes
uint32_t sendNTPpacket(IPAddress& address);
unsigned int nextDataSendMinuteOneShot;

// Timer
#define TIMER_BASE TIMERA2_BASE
#define TIMER TIMER_B

// CREAT PWM FROM TIMERA2 TIMER_B
Timer timerA2PWM(TIMER_BASE,TIMER,INVERSION_YES);
cc3200Adc adcCh3(ADC_CH_3);
HDC1000 tempHumiditySensor;
//HDC1000 mySensor(0x41, 2) <-- DRDYn enabled and connected to Arduino pin 2 (allows for faster measurements).
// Initialize the Wifi client library
WiFiClient client;



// global variable
#define NoOfData 25
#define NoOfAverage 10
#define PWMPeriod 10.0    // milisecond
#define PWMDuty 0.32      // milisecond
#define MOVING_AVEGRAGE_1  20            // moving average of 20 samples
#define MOVING_AVEGRAGE_Number  60
#define NoOfDataToEncode  1
#define DC_FAN  5


double P1Threshold = 0.52 ;   // P1 threshold
double  P2Threshold = 0.55 ;   // P2 threshold
double adcVoltAverage;
unsigned long adcTimeStamp,adcTS1,adcTS2, timeDifference; // adc timestamp
unsigned long p1Count, p2Count, p1CountAverage, p2CountAverage;
boolean flagFor1sec = false;  // 1 sec flag
unsigned long tick1Second, tick30Second, tick1Minute, tick1Hour;
double *p1CountArray, *p2CountArray;
double adcVoltMovingAverage1Array[MOVING_AVEGRAGE_1];
unsigned long t1,t2,t3;
double Temperature,Humidity,AdcVoltage;
uint16_t HDC1000Configuration;              //HDC1000 current configuration
unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 1 * 60 * 1000L; // delay between updates, in milliseconds
const unsigned long sleepInterval = 15 * 1000L;   // sleep 15 sec until postingInterval
// int *humidityArray , *temperatureArray, *adcVoltageArray;      // dynamic memory for temperature & humidity measurements

// moving average 
double movngAverageStorage[MOVING_AVEGRAGE_Number];
double movingAveragedADCVoltage;
// Encode data array
int humidityArray[NoOfDataToEncode];
int temperatureArray[NoOfDataToEncode]; 
int adcVoltageArray[NoOfDataToEncode];
int movingAverageAdcVoltageArray[NoOfDataToEncode];

// Network SSID , password
//char* networkID = "SungwonGawon2_5G";
//char* networkID = "ByoungLoh" ;
char* networkID = "NetweeN";
char* password = "car391133";

void setup()
{
// dynamicMemoryAllocation();
 intialization(); 
 digitalWrite(DC_FAN,1); // turn on fan
 
 // Network Clock initialization
  // Real-Time Clock Update
  myRTC.begin();
  myRTC.setTimeZone(tz_KOR);
  myRTC = RealTimeClockUpdate(myRTC);
  unsigned long int timeInMin; 
   timeInMin= getCurrentTime(myRTC);
   Serial.print("time:");Serial.println(timeInMin);
   nextDataSendMinuteOneShot = timeInMin +1;
 //delay(10000); // wait for sensor to be ready 
 }



void loop()
{
  
  // data send flag is updated every 2 minutes
  static bool dataSendFlag;
  char updateIntervalInMinutes = 2; // unit : minutes
  dataSendFlag = dataSendFlagHandler(nextDataSendMinuteOneShot, updateIntervalInMinutes, myRTC);
  
  // clock update (myRTC : RTC instance)
  unsigned int updateTime = 9;  // clock is updated via web 9th minutes of every hour
  myRTC = updateRealTimeClock(myRTC,updateTime);
  
  // Hibernate
  #define HIBERNATE_TIME ((32768)*(5)*(60))      // 4 min ; time based on 32.768 kHz clock
 // #define RESET_INTERVAL_HOUR         6      // software reset every 6 hour
 static  char **dataToPostToGoogle;              // double pointer to encoded data string
 
 // display adc results every 1 second
   if(flagFor1sec) 
   {
      flagFor1sec = false;   
  //   displayResults();
    }
  
  // TURN-ON FAN 10 SEC BEFORE TRANSMISSION  
  if (millis() - lastConnectionTime > postingInterval - 10000L) 
  {
  //  digitalWrite(DC_FAN,1); // turn on fan
    
  }
  // if postingInverval seconds have passed since your last connection
  // then connect again and send data:
  if (dataSendFlag)
     {  
       
       //     char **dataToPostToGoogle;
       dataToPostToGoogle = postDataEncoding(temperatureArray,humidityArray,adcVoltageArray,movingAverageAdcVoltageArray);
       httpRequest(dataToPostToGoogle);
     //  Serial.println("Hibernating ...");delay(100);
   //    timerA2PWM.softReset(HIBERNATE_TIME);
        
     }
     // software reset
    // if (tick1Minute > 1)
     //  timerA2PWM.softReset(HIBERNATE_TIME);
}

/***************************************************************************/
/*********************** SUB-ROUTINES **************************************/
/**************************************************************************/
/*
void dynamicMemoryAllocation(){
  // allocate dynamic memory for calculating moving average
//  adcVoltMovingAverage1Array = (double*)calloc(MOVING_AVEGRAGE_1,sizeof(double));
//  p1CountArray = (double*)calloc(MOVING_AVEGRAGE_30,sizeof(double));
//  p2CountArray = (double*)calloc(MOVING_AVEGRAGE_30,sizeof(double));
  temperatureArray = (int*)calloc(NoOfDataToEncode, sizeof(int));
  humidityArray = (int*)calloc(NoOfDataToEncode, sizeof(int));
  adcVoltageArray = (int*)calloc(NoOfDataToEncode, sizeof(int));
}*/

void intialization() {
  setupPWM_Timer();
  Serial.begin(9600);
  adcCh3.begin();
  tempHumiditySensor.begin();
  pinMode(DC_FAN,OUTPUT);  // FAN ON-OFF CONTRON
//  digitalWrite(DC_FAN,1); // turn on fan
  connectToWifi(networkID,password); // setup wifi
}
 
void storeDataForTransmission(int *array, int data)
{
  // shift all data in the array to the right     
  for ( char k=0; k <NoOfDataToEncode-1; k++)
      *(array+k) = *(array+k+1);
  // store a new data to array    
      *(array + NoOfDataToEncode-1) = data;
}


// Encode int only for posting to Google sheet
static char** postDataEncoding(int* temperature, int* humidity, int *adcVoltage , int *movAveragedAdcVoltage) 
{
  #define BUFFER_SIZE  120  
 static char *bufferEncoding[NoOfDataToEncode]; 
 static unsigned long data_index;
 char batteryStatus = (HDC1000Configuration & 0x0800) >> 11 ;
 for(char k=0;k<NoOfDataToEncode; k++)
  {
    bufferEncoding[k]     = (char*)calloc(BUFFER_SIZE, sizeof(char));
    sprintf( bufferEncoding[k], "id=%d&time=2016&temperature=%d&humidity=%d&PM2_5=%d&MovingAverage=%d&BatteryStatus=%d",data_index, *(temperature+k),*(humidity+k),*(adcVoltage +k),*(movAveragedAdcVoltage+k),batteryStatus);
  }
  data_index++;
  return  bufferEncoding;
}


void displayResults() {
  Serial.print("count: ");                  Serial.print(tick1Second);
  Serial.print("\t adc volt(v): ");          Serial.print(adcVoltAverage);
  Serial.print("\t TIME(min):  ");           Serial.print(tick1Minute);
  Serial.print("\t t3(isr loop time, ms) :  ");           Serial.print(t3);
 // Serial.print("\t p1Count:  ");      Serial.print(p1Count);
 // Serial.print("\t p2Count:  ");      Serial.print(p2Count);
//  Serial.print("\t p1Avg:  ");              Serial.print(p1CountAverage);
//  Serial.print("\t p2Avg:  ");              Serial.print(p2CountAverage);
  Serial.print("\t Temperature: ");         Serial.print(Temperature); 
  Serial.print("\t Humidity: ");            Serial.print(Humidity);Serial.println("%");
}

void setupPWM_Timer()
{
  timerA2PWM.pinMuxAndPeripheralClockConfigure(PRCM_TIMERA2, PIN_64, PIN_MODE_3);
  timerA2PWM.configure( TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM ); // half width, PWM
  timerA2PWM.setInterrupt(TIMER_CAPB_EVENT, TIMER_EVENT_NEG_EDGE, pwmISR);
  timerA2PWM.setPeriod(PWMPeriod);   //    period in milisecond
  timerA2PWM.setDuty(PWMDuty);     //     duty in milisecond
  timerA2PWM.enable();         //     enable timer
}

// isr on pwm negative-edge
void pwmISR()
{
  static int isr_cnt,k, data;
  unsigned long ulstatus;
  double adcVoltage;  // adc voltage
  double adcVoltArray[NoOfData];
  
  t1 = millis();  //pwmISR loop-time check
  adcTS1 = adcCh3.getTimeStamp(); // get adc start time
 
  for (char i=0; i<NoOfData; i++)
  {
    adcVoltArray[i]  =  adcCh3.getAdcValue();
  }  
  // ADC time check
  adcTS2 = adcCh3.getTimeStamp();  // final time for mulitiple times of ADC
  timeDifference = adcTS2-adcTS1;    // elapsed time
  
  // clear interrupt flag
  ulstatus = timerA2PWM.intStatus(TIMER_BASE);
  timerA2PWM.intClear(TIMER_BASE, ulstatus);
  
// calcuate moving-avegrage of volt_array[20]
 adcVoltAverage =  movingAverage(adcVoltMovingAverage1Array, adcVoltArray[20], MOVING_AVEGRAGE_1);

  // P1 and P2 count
/*  if (adcVoltAverage > P1Threshold)
      p1Count++;
  if (adcVoltAverage > P2Threshold)
      p2Count++; */
  
  // 1 second flag
  isr_cnt ++; 
if(isr_cnt == (int)(1000 / (PWMPeriod))) {
  flagFor1sec = true;
  isr_cnt = 0;
 // p1CountAverage = (int)movingAverage(p1CountArray, (double)p1Count, MOVING_AVEGRAGE_30);
 // p2CountAverage = (int)movingAverage(p1CountArray, (double)p2Count, MOVING_AVEGRAGE_30);
  
  // get data
  Temperature = tempHumiditySensor.getTemp(); // get temperature
  Humidity = tempHumiditySensor.getHumi();  // get humidity
  AdcVoltage = adcVoltAverage ; // get ADC voltage
  HDC1000Configuration = tempHumiditySensor.readConfig();
  movingAveragedADCVoltage= movingAverage( movngAverageStorage, AdcVoltage,MOVING_AVEGRAGE_Number);
 // movingAveragedADCVoltage= (int) movingAverage( movngAverageStorage, 100.0 ,20);
  // store data for transmission ( temp. and humidity is multiplied by 10)
  storeDataForTransmission(temperatureArray,(int)(Temperature*10));
  storeDataForTransmission(humidityArray,(int)(Humidity*10));
  storeDataForTransmission(adcVoltageArray,(int)(AdcVoltage*1000)); // mili-volt
  storeDataForTransmission(movingAverageAdcVoltageArray,(int)(movingAveragedADCVoltage*1000)); // mili-volt
  
//  p1Count=0;                                              // reset p1 count
//  p2Count=0;
  tick1Second++;                                      // increment 1 sec tick
  (tick1Second%60) ?  :tick1Minute++ ;   // increment 1 min tick
   (tick1Minute%60) ?   : tick1Hour++ ;   // increment 1 min tick
  }
// isr loop time check
  t2 = millis();
  t3=t2-t1;
}

void pinISR()
{
//  static char data = 0;
//  int val;
//  data ^=1;  // toggle output
//  digitalWrite(8,data); // pulse output
//  val = analogRead(A3); // adc  
// if (isr_cnt == 500)
//  digitalWrite(5,0); 
}

// moving average of 10 samples
// moving average with pointer
double movingAverage(double* arrayPtr, double stream, const int movingAverageNum)
{
  static double voltSum;
  double voltMovingAvg;
  char k,m;

  for (  k=0; k <movingAverageNum-1; k++)
  *(arrayPtr+k) = *(arrayPtr+k+1);
  *(arrayPtr + movingAverageNum-1) = stream;

  // calculate moving average
  for ( m=0; m<movingAverageNum; m++)
  voltSum += *(arrayPtr + m);
  voltMovingAvg = voltSum /movingAverageNum ;
  voltSum =0;  // reset dummy2*/

  return voltMovingAvg;
}

/************ WIFI CONNECTION ***************/
// this method makes a HTTP connection to the server:
void httpRequest(char **dataStringArray) {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  // server address:
char server[] = "script.google.com";
String data; // data to post

  // if there's a successful connection:
  for (char k=0; k<NoOfDataToEncode; k++)
   {
      client.stop();
    if (client.sslConnect(server, 443)) {
    Serial.print("POSTING DATA TO GOOGLE SHEET...");
    
      data = dataStringArray[k];
      if (k < NoOfDataToEncode)  // test for 
      {
        // send the HTTP POST request: (Google sheet ==> CC3200 Energia Google Post with Dweet For Subway PM2_5 Test)
       // client.println("POST /macros/s/AKfycbxV4wmDa5fe9zPsLnWL0dKoNcFJLo2te07DBw-9beMFMHypi3M/exec HTTP/1.1");
        client.println("POST /macros/s/AKfycbxQmn0Ly3tcE9sCied0Rh_XV6-k9reEMkW0lCkCvUai5J3hgFI/exec HTTP/1.1");
        
        client.println("Host: script.google.com");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(data.length());
        client.println();  // required but i don't know exactly why?
        client.print(data);
        client.println();
        client.println("Connection: close");
          }
   
      Serial.print("posted data: ");Serial.println(data);
      delay(50);  // required but i don't know exactly why?
      // FREE DYNAMIC MEMORY  
      free(dataStringArray[k]);
    }
    else 
    {
      // if you couldn't make a connection:
      Serial.println("connection failed");
    }
    delay(800); // delay for posting data to GOOGLE (required !!)
}
    
    // TURN OFF FAN
    //digitalWrite(DC_FAN,0);
    
    // GO INTO LOW POWER MODE
  //  sleep(postingInterval - sleepInterval);
    
    // note the time that the connection was made:
    lastConnectionTime = millis();
}


void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToWifi(char* ssid, char *password)
{
  // your network name also called SSID
//char *ssid = "NetweeN";
// your network password
//char *password = "car391133";
 // attempt to connect to Wifi network:
 const int delayTime_ms = 500;
 const char connectionTryMaxCount = 5; // max. connectionn try before give-up
 static int connectionTrialCount;
    Serial.print("Attempting to connect to Network named: ");
    // print the network name (SSID);
    Serial.println(ssid); 
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin(ssid, password);
    while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
      Serial.print("re-connection tried :");
      Serial.println(connectionTrialCount);
      delay(delayTime_ms);
      connectionTrialCount++;
      if(connectionTrialCount > connectionTryMaxCount)
      {
        timerA2PWM.softReset(10000L); // reboot system to re-connect
        connectionTrialCount = 0;
      }
      
    }
  
    Serial.println("\nYou're connected to the network");
    Serial.println("Waiting for an ip address");
  
    while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
      Serial.print(".");
      delay(300);
      }

    Serial.println("\nIP Address obtained");
    // We are connected and have an IP address.
    // Print the WiFi status.
    printWifiStatus();
}

//*************  Clock Update Sub-routines *****************//
DateTime RealTimeClockUpdate(DateTime myRTC)
{
  
  time_t myEpochNTP, myEpochRTC;
  tm myTimeRTC;
  char loop_cnt; 
  unsigned int hour, minute, timeInMinutes; 
  bool flagRTCU = true;
  while (flagRTCU) // real time clock update via web
  {
  bool flagNTP = getTimeNTP(myEpochNTP);
  
  Serial.print("NTP ? ");Serial.println(flagNTP, DEC);
  Serial.println(myEpochNTP);
  if (!flagNTP)
    {
        flagRTCU = false;
        myRTC.setTime(myEpochNTP);
        myEpochRTC = myRTC.getTime();
         Serial.println(myEpochRTC);
        Serial.println("*** CC3200 NTC updated: Greenwich Time");
        convertEpoch2Structure(myEpochRTC, myTimeRTC);
        Serial.println(convertDateTime2String(myTimeRTC));
        Serial.print("hour : min ==> ");
        Serial.print(myTimeRTC.tm_hour);
        Serial.print(":");
        Serial.println(myTimeRTC.tm_min);
    }
   else
   {
     // wait for 5 secondes before attempting a retry
     for (int8_t i = 5; i > 0; i--)
     {
        Serial.print(".");
        delay(1000);
      }
      loop_cnt++; // increase loop counter if it exceeds a certain number cancel update
      Serial.print(" loop counter:");
      Serial.println(loop_cnt);
   }
  }
  
   return myRTC;
}

unsigned long int getCurrentTime(DateTime myRTC)
{
  time_t myEpochRTC;
  tm myTimeRTC; 
  unsigned long int currentTime;
  
  myEpochRTC = myRTC.getTime();
   convertEpoch2Structure(myEpochRTC, myTimeRTC);
    //   Serial.println(convertDateTime2String(myTimeRTC));
    //    Serial.print("hour : min ==> ");
    //    Serial.print(myTimeRTC.tm_hour);
    //    Serial.print(":");
    //    Serial.println(myTimeRTC.tm_min);
        
   // currentTimeInSecond = myTimeRTC.tm_hour * 3600 + myTimeRTC.tm_min * 60 + myTimeRTC.tm_sec;
   // Serial.println(currentTimeInSecond);
   currentTime = myTimeRTC.tm_min;
   
    return currentTime;
} 

// raise data send flag when time equals data-send-time which increments at a certain time-interval
bool dataSendFlagHandler(unsigned int nextDataSendTimeOneShot, char incrementTime, DateTime myRTC )
{
  unsigned int currentMinute, currentTime;
  static unsigned int dataSendTime;
  static unsigned long loopCnt;
  bool dataSendFlag;
  
  loopCnt++;
  // get current time
  currentMinute = getCurrentTime(myRTC);
  currentTime = currentMinute;
  
  // excute only once for first update
  if(loopCnt == 1) 
      dataSendTime = nextDataSendTimeOneShot; // first data send ; one minute after the setup()
   
   // when current time equals dataSendTime dataSendFlag is raised    
  if (currentTime == dataSendTime)
  {
    // set updateFlag
    dataSendFlag = true;
    
    // increment the send-time to next data send time
    dataSendTime = dataSendTime + incrementTime;
    
    // reset dataSendTime when it gets greater than 60
      if(dataSendTime >= 60)
         dataSendTime -= 60;
        return dataSendFlag;
  }
  else 
  {
    dataSendFlag = false; // reset data flag for next check for data send
    return dataSendFlag;
  }
}

// Clock update

DateTime updateRealTimeClock(DateTime myRTC, unsigned int updateTime)
{
  // check current time
  // if current time passes preset update time, then updates. 
  // If update is successful, set updatedFlag to true
  // update frequency is once a day
  
  static bool updatedFlag = false;
  unsigned int currentTime, currentMinute;
  
  // check current time
  currentMinute = getCurrentTime(myRTC);
  currentTime = currentMinute;
        
  if (currentTime > updateTime)
      {
        if (updatedFlag == false)
          {
            //update !!
            myRTC = RealTimeClockUpdate(myRTC);
            updatedFlag = true;
            Serial.println("Clock is updated via Web server !");
         //   Serial.println(updatedFlag);
            return myRTC;
          }
        else
         {
           // Do nothing.
         }
      }
   else
   {
     // reset updatedFlag
     updatedFlag = false;
   }   
   // Serial.println(updatedFlag);        
}

// *********** END OF CLOCK-UPDATE SUB-ROUTINES ************************* //


/*****************************************************************/
/******************  DEPRECATED **********************************/
/****************************************************************/

double movingAverage10(double stream)
{
  const int arrayIndex = 10;
  static unsigned long cnt, i;
  static double dummy[arrayIndex];
  static double dummy2;
  double volt_avg;
  cnt++;
  for ( char k=0; k <arrayIndex-1; k++)
  dummy[k] = dummy[k+1];
  dummy[arrayIndex-1] = stream; 
  
  // calculate average
  for (char m=0; m<arrayIndex; m++)
  dummy2 += dummy[m];
  volt_avg = dummy2 /arrayIndex ;
  dummy2 =0;  // reset dummy2
  return volt_avg;
}

// moving average of 30 seconds
unsigned long movingAverageOfP1CountFor30Sec(unsigned long stream)
{
  const int arrayIndex = 30;
  static unsigned long cnt, i;
  static unsigned long dummy[arrayIndex];
  static unsigned long dummy2;
  unsigned long pCount_avg;
  cnt++;
  for ( char k=0; k <arrayIndex-1; k++)
  dummy[k] = dummy[k+1];
  dummy[arrayIndex-1] = stream; 
  
  // calculate average
  for (char m=0; m<arrayIndex; m++)
  dummy2 += dummy[m];
  pCount_avg = dummy2 /arrayIndex ;
  dummy2 =0;  // reset dummy2
  return pCount_avg;
}

unsigned long movingAverageOfP2CountFor30Sec(unsigned long stream)
{
  const int arrayIndex = 30;
  static unsigned long cnt;
  static unsigned long dummy[arrayIndex];
  static unsigned long dummy2;
  unsigned long pCount_avg;
  cnt++;
  for ( char k=0; k <arrayIndex-1; k++)
  dummy[k] = dummy[k+1];
  dummy[arrayIndex-1] = stream; 
  
  // calculate average
  for (char m=0; m<arrayIndex; m++)
  dummy2 += dummy[m];
  pCount_avg = dummy2 /arrayIndex ;
  dummy2 =0;  // reset dummy2
  return pCount_avg;
}


