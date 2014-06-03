#include <cstdarg.h>


#define PCBVERSION1.0

#ifdef PCBVERSION1.0
#define STAT1 A2
#define MCUONLED A3
#define STAT2 A6
#define ERRORLED A7

#elif defined PCBVERION2.0
#define STAT1 A2
#define MCUONLED 8
#define STAT2 A3
#define ERRORLED A7

#endif


//void statusLight(boolean s1, boolean s2, boolean err);
void statusLight(boolean s1, boolean s2, boolean err) {
  digitalWrite(STAT1, s1);
  digitalWrite(STAT2, s2);
  digitalWrite(ERRORLED, err);
}

void statusLightIntOne(boolean s1, boolean err, boolean mcu) {
  digitalWrite(STAT1, s1);
  digitalWrite(ERRORLED, err);
  digitalWrite(MCUONLED, mcu);
}

void statusLight(boolean s1, boolean s2, boolean err, boolean mcu) {
  digitalWrite(STAT1, s1);
  digitalWrite(STAT2, s2);
  digitalWrite(ERRORLED, err);
  digitalWrite(MCUONLED, mcu);
}

void setStatusLight(boolean s1enable, boolean s1, boolean s2enable, boolean s2, boolean errenable, boolean err) {
  if(s1enable) digitalWrite(STAT1, s1);
  if(s2enable) digitalWrite(STAT2, s2);
  if(errenable) digitalWrite(ERRORLED, err);
}


/*void xbeeWrite(int xPin, boolean value) {
  xbee.print("+++ATMY");
}
*/


void setPins2() {
  pinMode(2, INPUT_PULLUP); //for interrupt 0 (this is the mode button)
  pinMode(3, INPUT_PULLUP); //for interrupt 1 (this is the XBee interrupt) #PWM
//pinMode(4, OUTPUT);    //XBee (MCU to) TX is on pin 7 **SoftwareSerial
  pinMode(5, OUTPUT);    //XBee Wireless Interupt LED #PWM
  pinMode(6, OUTPUT);    //Buzzer #PWM
//pinMode(7, OUTPUT);    //XBee (MCU to) RX is on pin 6 #PWM **SoftwareSerial
  pinMode(8, OUTPUT);    //MCU-ON LED
  pinMode(9, OUTPUT);    //MCU Status 1 LED
  pinMode(10, OUTPUT);   //Laser EN #PWM
  pinMode(11, OUTPUT);   //LED for mode button (BUTTONLED) #PWM
  pinMode(12, INPUT);    //Charge Indicator
  pinMode(13, OUTPUT);   //Builtin LED/STAT2
  pinMode(A0, OUTPUT);   //Error LED
  pinMode(A1, OUTPUT);   //XBee Sleep Pin
//pinMode(A2, OUTPUT);   //LCD RX/TX is on pin 4 **SendOnlySoftwareSerial
  pinMode(A3, OUTPUT);   //I2C SDA/SCL Relay (RELAY)
//pinMode(A4, INPUT);    //I2C SDA #PWM-S **Wire
//pinMode(A5, INPUT);    //I2C SCL #PWM-S **Wire
  pinMode(A6, INPUT);    //Humidity sensor
  pinMode(A7, INPUT);    //Microphone
}

void setPins1() {
pinMode(2, INPUT_PULLUP); //for interrupt 0 (this is the mode button)
  pinMode(3, INPUT_PULLUP); //for interrupt 1 (this is the XBee interrupt) #PWM
//pinMode(4, OUTPUT);    //LCD RX/TX is on pin 4 **SendOnlySoftwareSerial
  pinMode(5, OUTPUT);    //XBee Wireless Interupt LED #PWM
//pinMode(6, OUTPUT);    //XBee (MCU to) RX is on pin 6 #PWM **SoftwareSerial
//pinMode(7, OUTPUT);    //XBee (MCU to) TX is on pin 7 **SoftwareSerial
  pinMode(8, OUTPUT);    //LiPo Regulator Power-Save
  pinMode(9, OUTPUT);    //Buzzer #PWM
  pinMode(10, OUTPUT);   //Laser EN #PWM
  pinMode(11, OUTPUT);   //LED for mode button (BUTTONLED) #PWM
  pinMode(12, OUTPUT);   //I2C SDA/SCL Relay (RELAY)
  pinMode(13, OUTPUT);   //Builtin LED
  pinMode(A0, INPUT);    //Humidity sensor
  pinMode(A1, OUTPUT);   //XBee Sleep Pin
  pinMode(A2, OUTPUT);   //MCU Status 1 LED
  pinMode(A3, OUTPUT);   //MCU On LED
//pinMode(A4, INPUT);    //I2C SDA #PWM-S **Wire
//pinMode(A5, INPUT);    //I2C SCL #PWM-S **Wire
  pinMode(A6, OUTPUT);   //MCU Status 2 LED
  pinMode(A7, OUTPUT);   //Error LED
}

/*
#include <iostream>
#include <stdarg.h>
using namespace std;

void statusLight (char byter, bool sw, ...);
void statusLight(char byter, bool sw, ...) {

  va_list swargs;
  va_start(swargs, sw);

  if (byter == '0') {cout << "m"}
  int led = 0;
    if (led == 1) cout << "m1";
    if (led == 2) cout << "m2";
    if (led == 3) cout << "m3";
    cout << "f";
  count++;
  va_arg(swargs, bool);
  va_end(swargs);
}

*/
