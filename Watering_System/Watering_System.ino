
/**********************************************
  Project: Watering System SMD V1.0       31-07-2017
  JEC
***********************************************/


// INCLUDES
#include <I2CSoilMoistureSensor.h>
#include <TinyWireM.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>



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


// OBJECTS
I2CSoilMoistureSensor sensor(0x20);
SoftwareSerial mySerial(RX, TX);
SoftwareSerial debug(-1, PORTA4);


//MESSAGES
const char msgInit[] = "OK";

void setup() {
 
  DDRB |= (1 << DDB0);      // B0 -> Output
  DDRA |= (1 << DDA7);      // A7 -> Output


  mySerial.begin(BAUDRATE);     // Init serial comm.
  debug.begin(BAUDRATE);        // Init debug wire
  //TinyWireM.begin();          // Init i2c comm.


  //ESP8266 ON;
  ESP_ON;
  delay(500);


}



void loop() {

  clearBuffer();
  mySerial.println("AT");
  bool initOK = waitForString("OK",2,2000);

  debug.println();
  if (initOK) debug.println("initOK");
  else debug.println("initNoOK");


  delay(5000);



}



bool waitForString(char* input, uint8_t length, unsigned int timeout) {

  unsigned long end_time = millis() + timeout;
  int current_byte = 0;
  uint8_t index = 0;

  while (end_time >= millis()) {

    if (mySerial.available()) {

      //Read one byte from serial port
      current_byte = mySerial.read();
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

  while (mySerial.available())
    mySerial.read();
}

