
/**********************************************
  Project: Watering System SMD V1.0       31-07-2017
  JEC
***********************************************
***********************************************
 
 Comments:
  - ESP-01 must be previously configured:
      * AT+CWMODE_DEF=3                         // SoftAP+Station mode 
      * AT+CWJAP_DEF= "<ssid>","<password>"     // Connect to wifi access point
      * AT+UART_DEF=9600,8,1,0,0                // Set UART speed to 9600
      
************************************************/

// INCLUDES
#include <I2CSoilMoistureSensor.h>
#include <TinyWireM.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>


// ADJUSTMENT VARIABLES
unsigned int nLightMax         = 40000;           // Max light to water
unsigned int nTempMin          = 4;               // Min. temp to water (ºC)
unsigned int nMoistureMin      = 490;             // Min. moist to water
unsigned int nMoistureMax      = 540;             // Max. moist to water
unsigned int nWateringTime     = 20000;           // Watering time (ms) 
unsigned int nMaxWateringTimes = 5;               // Max. watering times




// DEFINES
#define EV_ON       PORTA |=  (1 << PORTA5)
#define EV_OFF      PORTA &= ~(1 << PORTA5)

#define SENSOR_ON   PORTB |=  (1 << PORTB0)
#define SENSOR_OFF  PORTB &= ~(1 << PORTB0)

#define ESP_ON      PORTA |=  (1 << PORTA7)
#define ESP_OFF     PORTA &= ~(1 << PORTA7)
#define RX          PORTA3
#define TX          PORTA2
#define BAUDRATE    9600

//#define IP          "184.106.153.149"           // thingspeak.com




// OBJECTS
I2CSoilMoistureSensor sensor(0x20);
SoftwareSerial esp(RX, TX);
//SoftwareSerial debug(-1, PORTA4);

// VARIABLES
int          nState = 0;
const char*  GET = "GET /update?key=XXXXXXXXXXXXXX&field1=%u&field2=%d&field3=%d&field4=%d";
char         cmd[80];
int          nChars = 0;
uint16_t     nLight;                      // Measured light value
int          nTemp;                       // Measured temp value
unsigned int nMoisture;                   // Measured moist value
unsigned int nShortSleep = 300;           // Short sleep lenght in seconds 300s=5min
unsigned int nLongSleep = 3600;           // Long sleep lenght in seconds 3600s=1h
bool         bOldState = 0;               // Previous valve state
unsigned int nWateringTimes = 0;          // Watering times





void setup() {

  configurePins();
  setupPowerSaving();
  esp.begin(BAUDRATE);        // Init serial comm.
  TinyWireM.begin();          // Init i2c comm.

}


void loop() {

  switch (nState) {

          case 0:   // Switch on sensor

                SENSOR_ON;
                delay(1500);
                nState = 10;
                break;
                

          case 10:  // Measure light, temperature and moisture
                
                nLight = sensor.getLight(true);
                nTemp = sensor.getTemperature()/10;
                nMoisture = sensor.getCapacitance();
                SENSOR_OFF;                             // Switch off sensor
       
          
                // If "dark" and tempeture is higher that min then water
                if ((nLight > nLightMax) && nTemp > nTempMin) {
                  nState = 20;
                }
          
                // No watering then go to sleep
                else {
                  
                  nWateringTimes = 0;
                  bOldState = false;
                  nState = 200;
                  
                }

       
                break;

           case 20: // Check moist
        
        
        
              if ((nMoisture < nMoistureMin) || (nMoisture < nMoistureMax) && bOldState) {
        
                nWateringTimes++;
        
                if (nWateringTimes > nMaxWateringTimes) {
                  nWateringTimes = 0;
                  bOldState = false;
                  nState = 200;
                }
                else {
                  bOldState = true;
                  nState = 30;
                }
              }
        
              if ((nMoisture > nMoistureMax) || (nMoisture > nMoistureMin) && !bOldState) {
        
                nWateringTimes = 0;
                bOldState = false;
                nState = 200;
              }


                break;    



          case 30:  // Water


                EV_ON;                    // Switch on solenoid valve
                delay(nWateringTime);     // Watering................
                EV_OFF;                   // Switch off solenoid valve

                nState = 100;
                break;

          case 100:   // Short sleep

                uploadData();                           // Send data to thinkspeak.com
                sleep(nShortSleep/8);                       // Sleep "nShortSleep" seconds
                nState = 0;
                break;
      
          case 200:   // Long sleep
      
                uploadData();                           // Send data to thinkspeak.com
                sleep(nLongSleep/8);                        // Sleep "nLongSleep" seconds
                nState = 0;
                break;
      
      
          default: nState = 0;




  }

  
}

/*
bool espCommand ( char* input, char* output, uint8_t length, unsigned int timeout, unsigned int maxAttempts) {

                unsigned int attempts = 0;
  
                 while (attempts < maxAttempts) {
                  
                      clearBuffer();
                      esp.println(input);
            
                      if (waitForString(output,length,timeout)) {
                          return true;
                      }
                      else {
                          attempts++;
                          //debug.println();
                          //debug.println(attempts,DEC);
                      }
                                       
                }

                return false;
}






bool waitForString(char* input, uint8_t length, unsigned int timeout) {

  unsigned long end_time = millis() + timeout;
  int current_byte = 0;
  uint8_t index = 0;

  while (end_time >= millis()) {

    if (esp.available()) {

      //Read one byte from serial port
      current_byte = esp.read();
      //debug.print(char(current_byte));

      if (current_byte != -1) {
        //Search one character at a time
        if (current_byte == input[index]) {
          index++;

          //Found the string
          if (index == length) {
            return true;
          }
          //Restart position of character to look for
        } else {
          index = 0;
        }
      }
    }
  }
  //Timed out

  return false;
}



//Remove all bytes from the buffer
void clearBuffer() {

  while (esp.available())
    esp.read();
}

*/


//Count bytes in a char string
uint8_t countChars(char* inputChar) {

    uint8_t n = 0;
    
    while (inputChar[n]!= 0) {
      //debug.print(inputChar[n]);
      n++;
    }
  
  //debug.println();
  return n+2;
}


void configurePins() {
  
  DDRB |= (1 << DDB0);      // B0 -> Output (SENSOR ON)
  DDRA |= (1 << DDA7);      // A7 -> Output (PWR ESP)
  DDRA |= (1 << DDA5);      // A5 -> Output (E.VALVE)

  
}




void uploadData() {


      // Switch on ESP and wait while connects to wifi network
      ESP_ON;
      delay(10000);

      // Connect to thinkspeak.com
      esp.println(F("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80"));
      delay(1000);

      // Write command in "cmd"
      sprintf(cmd,GET,nLight,nTemp,nMoisture, bOldState);
      nChars = countChars(cmd);

      // Send command
      esp.print(F("AT+CIPSEND="));
      esp.print(nChars);
      esp.println();
      delay(1000);
      esp.println(cmd);
      delay(1500);
      ESP_OFF; 
  
}

void setupWatchdog() {
  cli();                                            // Disable global interrupts
  // Set watchdog timer in interrupt mode
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);     // set WDIE, and 8 seconds delay
  sei();                                            // Enable global interrupts
}


void sleep(int times) {

  setupWatchdog();                                  // Configure and activate Watchdog

  for (int i = 0; i < times; i++) {

    cli();                                          // Disable interruptions just in case
    sleep_enable();                                 // Enable sleep


    //    sleep_bod_disable();                       // Deshabilitar BOD durante sleep
    //    MCUCR |= _BV(BODS) | _BV(BODSE);           //disable brownout detection during sleep
    //    MCUCR &=~ _BV(BODSE);

    sei();                                          // Enable interruptions
    sleep_cpu();                                    // Go to sleep
    sleep_disable();                                // When cpu is awake, disable sleep

  }

  wdt_disable();                                    // Disable watchdog in order to not have more WDT interruptions
}


void setupPowerSaving() {

  ADCSRA &= ~(1 << ADEN);                           // Disable ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // Configure sleep mode
  
}

ISR(WDT_vect) {

  // Nothing to do, just wake up

}


