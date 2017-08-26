
/**********************************************
  Project: Watering System SMD V1.0       31-07-2017
  JEC
***********************************************/


// INCLUDES
#include <I2CSoilMoistureSensor.h>
//#include <TinyWireM.h>
#include <SoftwareSerial.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/wdt.h>
//#include <avr/interrupt.h>



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
#define WIFIAP      "MOVISTAR_0200"  
#define PASS        "lmwKu4pTwtru7n9lpwpp"
#define MAX_ATTEMPTS 5
#define IP          "184.106.153.149"           // thingspeak.com




// OBJECTS
I2CSoilMoistureSensor sensor(0x20);
SoftwareSerial esp(RX, TX);
SoftwareSerial debug(-1, PORTA4);

// VARIABLES
int nState = 0;
const char* GET = "GET /update?key=PXHL4S4I4WITLDS3&field1=%d";
char cmd[64];
int n = 0;
int nChars = 0;


//MESSAGES
const PROGMEM char msgInput[] = "AT";
const PROGMEM char msgOutput[] = "OK";

void setup() {
 
  DDRB |= (1 << DDB0);      // B0 -> Output
  DDRA |= (1 << DDA7);      // A7 -> Output


  esp.begin(BAUDRATE);     // Init serial comm.
  debug.begin(BAUDRATE);        // Init debug wire
  //TinyWireM.begin();          // Init i2c comm.

 

}



void loop() {


  switch (nState) {

          case 0:

                debug.print(F("case ")); debug.println(nState,DEC);
                
                ESP_ON;
                delay(10000);
                nState = 20;
                break;
                
/*
          case 10:

                debug.print("case "); debug.println(nState,DEC);

                if (espCommand ("AT", "OK", 2, 2000, 5)) {
                    debug.println();
                    debug.println("Init OK");
                    nState = 20;
                    break;
                }
                else {
                    debug.println();
                    debug.println("Init NOK");
                    nState = 100;
                    break;
                }
  */                              

          case 20:

                debug.print(F("case ")); debug.println(nState,DEC);
                
                esp.println(F("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80"));
                
                delay(1000);
                nState = 30;
                break;

          case 30:
                debug.print(F("case ")); debug.println(nState,DEC);

                sprintf(cmd,GET,n);
                //sprintf(cmd,"\r\n");
                //cmd = GET;
                //cmd += n;
                //cmd += "\r\n";
                
                debug.println(cmd);
                debug.flush();

                nChars = countChars(cmd);
                debug.println(nChars);
                
                esp.print("AT+CIPSEND=");
                esp.print(nChars);
                esp.println();
                delay(10000);
                nState=40;
                break;

          case 40:
                debug.print(F("case ")); debug.println(nState,DEC);
                esp.println(cmd); 
                n++;
                nState = 100;
                break; 
                
          
     


          

          case 100:
                debug.println(F("________________________________________"));
                delay(20000);
                nState = 20;
                break;

  }




}


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
                          debug.println();
                          debug.println(attempts,DEC);
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
      debug.print(char(current_byte));

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


//Count bytes in a char string
uint8_t countChars(char* inputChar) {

    uint8_t n = 0;
    
    while (inputChar[n]!= 0) {
      debug.print(inputChar[n]);
      n++;
    }
  
  debug.println();
  return n+2;
}


