// Jim: The Self-Aware SenseBot//
// Written in Wiring, C, and C++ //

// ***** Coded by Jake Tesler ***** //

/* *************************************************************************
 *    ********* READ THESE INSTRUCTIONS! *********
 *  This project uses a custom enclosure, custom PCB (public design coming soon), various sensors and componentry, including an
 *  Arduino Pro Mini with an Atmel ATmega328p processor, a Digi International/MaxStream XBee 1mW 802.15.4 Series 1 radio module (with trace antenna), a STMicro LSM303D Accelerometer/Magnetometer and Compass,
 *  a Freescale MPL3115A2 Temperature and Barometric Pressure sensor, a Honeywell HIH4030 Humidity sensor, a TexasAOS TSL2561 Light and Illumination sensor, a Maxim Integrated DS1307 RTC for Date/Time storage and management (daughtercard), 
 *  a Maxim MAX17043G+U LiPo Monitoring module, a dual TI TPS61200/Microchip MCP73831T for LiPo regulation and power distribution, a Pololu D24V6F3 and a S10V4F5 for voltage regulation, 
 *  a PIC16F88-enabled character 20x4 Serial LCD panel, and a SparkFun TTL 654.5nm Class 3A Laser (detachable). 
 *  
 *  SDA/SCL control uses a G5V-2 Relay
 *  Modes are as follows: Off - Accelerometer/Compass - Altitude(Atmosphere, Temp, Altitude, Humididy, Pressure) - Light(lux, laser) - Warning
 *
 *  Debug Mode can be set in "debug.h".
 *
 *  LED Indicator Lights are used as follows: 
 *  SYS-ON: Indicates that the entire SenseBot System is receiving power. (SOLDER PAD MUST BE ENABLED)
 *  MCU-ON: This LED will be illuminated if the MCU is powered and is running. This LED's purpose is to echo the MCU's built-in onboard LED. 
 *  STAT1:  Generic indicator light. Used with LED Status Indicator States.
 *  STAT2:  Laser-enabled light. If the laser is on, this will be illuminated. [Also used as a Status Indicator Light, or SIL].
 *  ERROR:  This LED will illuminate if an error within the program has occurred, or possibly if some other hardware error has occurred. [SIL]
 *  LED Status Indicator Lights [SIL]:                               //Hint: [ON] (OFF) *IGNORE* {Mode}
 *   (STAT1) [STAT2] [ERROR] {*}: LSM303D initialization failed.
 *   (STAT1) (STAT2) [ERROR] {*}: Interrupt 0 execution error
 *   [STAT1] (STAT2) [ERROR] {0}: Battery error
 *   [STAT1] (STAT2) [ERROR] {*}: Out-of-range Sensor Error
 *   [STAT1] *STAT2* [ERROR] {3}: Interrupt 1 execution error
 *  
 * *************************************************************************
 */

#include "Arduino.h"
#include "Wire.h"
#include <SoftwareSerial.h>
#include <SendOnlySoftwareSerial.h>
#include "lcdCommands.h"
#include "animation.h"
#include "cycle.h"
#include "pluscode.h"
#include "debug.h"
#include "MAX1704.h"
#include <LSM303.h>
//#include <Adafruit_Sensor.h>
//#include <TSL2561.h>
#include <SFE_TSL2561.h>
#include <MPL3115A2.h>
#include <HIH4030.h>
#include <Time.h> //custom-adapted library
#include <DS1307RTC.h>


#define PCBVERSION1.0
static double pcbVersion = 1.0;

//Clarify definitions for pin reference
#ifdef PCBVERSION1.0
#define XBEELED 5
#define PWSAVE 8
#define BUZZER 9
#define LASEREN 10
#define BUTTONLED 11
#define RELAY 12
#define ONETHREE 13
#define HIH4030_PIN A0
#define XBEESLEEPD A1
#define STAT1 A2
#define MCUONLED A3
#define STAT2 A6
#define ERRORLED A7

#elif defined PCBVERSION2.0
#define XBEELED 5
#define PWSAVE 8
#define BUZZER 9
#define LASEREN 10
#define BUTTONLED 11
#define RELAY 12
#define ONETHREE 13
#define HIH4030_PIN A0
#define XBEESLEEPD A1
#define STAT1 A2
#define MCUONLED A3
#define STAT2 A6
#define ERRORLED A7

#endif


/*
 * Set the analog reference voltage.  Unless you have set it explicity using
 * analogReference(), this will be the same voltage as your Arduino, either 3.3 or 5.0
 */
#define ARDUINO_VCC 5.0
#define HIH4030_SUPPLY_VOLTAGE  5.0 //voltage supplied
#define H_SLOPE 0.03068 //from datasheet Table 2: Example Data Printout
#define H_OFFSET 0.958  //from datasheet Table 2: Example Data Printout

//I2C Address Definitions
#define MPL3115A2_ADDR 0x60
#define TSL2561_ADDR 0x39
#define LSM303_ADDR_B 0b0011101
#define LSM303_ADDR 0x1D
#define DS1307_ADDR 0x68
#define MAX1704_ADDR 0x6D

//Mode Information
volatile signed int mode = -1; //initial mode
volatile signed int prevMode = -1; //mode == 0 initially (really) and this will 
//##confirm the check for mode accuracy...[(4)-0-1-2-3] cycle
volatile boolean initSet = 0; //initial settings for each mode bit
volatile signed int warningShown = -1;

volatile signed int laser_pwr = 0;
volatile boolean xbeeSleep = 1; //start XBee Sleeping
boolean xbeeButtonLED = LOW;
const int ledLuxLevel = 200; //mode/XBee button led brightness

//String curTimeHMS;
//String curTimeMDY;
boolean timeOn; //are we enabling time for this boot?

//Debug On/Off (remember to switch below)

 
   boolean tslGain = 0;  // Gain setting, 0 = X1, 1 = X16;
    unsigned char tslTime = 2;
    unsigned int msInt;  // Integration ("shutter") time in milliseconds
    //static int tslSwitchCounter = 0;

//Animation tracker
//signed int animStep = -17; //tracker for confusedAnimation()
//static signed int initAnimStep = -17;


const int globalDelay = 250; //global event delay in ms (milliseconds)


MAX1704 fuelGauge;  //I2C - initialize fuel gauge
MPL3115A2 altbar;   //I2C - initialize barometer/altimeter/temperature
LSM303 lsm;         //I2C - initialize compass/accelerometer
SFE_TSL2561 tsl;
//TSL2561 tsl = TSL2561(TSL2561_ADDR_FLOAT, 12345); //I2C - initialize light sensor
HIH4030 calHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC);              //Analog - initialize humidity sensor (calibrated)
//HIH4030 uncalHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC);          //Analog - initialize humidity sensor

SoftwareSerial xbee(7,6); //7 iRX, 6 iTX* //initialize XBee
//SendOnlySoftwareSerial myLCD(4); //4 TX* ##CUSTOM //redefined in lcdCommands.h
//SoftwareSerial myLCD(5,4); //5 RX, 4 TX //redefined in lcdCommands.h ##not used

void setup()
{
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

  //digitalWrite(2, HIGH); //button power (un-float)
  analogWrite(BUTTONLED, ledLuxLevel); //turn on LED initially
  digitalWrite(PWSAVE, HIGH); //power-save mode (HIGH=off, LOW=on)
  digitalWrite(ONETHREE, HIGH); //activate builtin LED
  digitalWrite(MCUONLED, HIGH); //activate 'MCU On' LED
  digitalWrite(RELAY, HIGH); //start SDA/SCL batt monitor relay, leave on permanently
  digitalWrite(LASEREN, HIGH); //disable laser
  digitalWrite(XBEELED, ledLuxLevel); //XBee Button LED


  myLCD.begin(9600);                                                                        //initializes LCD at 9600 baud
  xbee.begin(115200);                                                                       //start XBee

  if(debug_setup) Serial.begin(9600);
  if(debug_setup) Serial.println("boot");
  xbee.println("boot");
  digitalWrite(MCUONLED, LOW);
  Wire.begin(); //delay(5);                                                                 //start I2C
  fuelGauge.reset(); fuelGauge.quickStart(); if(debug_setup) Serial.println("fuel.done");         //initialize LiPo capacity sensor
  altbar.begin();                            if(debug_setup) Serial.println("AltSnsr.done");      //initialize MPL3115A2
  lsm.init(); lsm.enableDefault();           if(debug_setup) Serial.println("ACM.done");          //initialize LSM303D
  calHumid.calibrate(H_SLOPE, H_OFFSET);     if(debug_setup) Serial.println("Humid.done");        //initialize/calibrate HIH4030
  //tsl.begin(); configureLightSensor();       if(debug_setup) Serial.println("LuxSnsr.done");      //initialize TSL2561
  tsl.begin(); tsl.setPowerUp(); tsl.setTiming(tslGain,tslTime,msInt);
  setSyncProvider(RTC.get);               /* if(debug_setup) Serial.println("Get RTC"); */        //the function to get the time from the RTC
  if (timeStatus() != timeSet) { timeOn = 0; if(debug_setup) Serial.println("No RTC"); }          //initialize or permanently kill RTC           
  else                         { timeOn = 1; if(debug_setup) Serial.println("RTC set"); }         //we are 1-offing this

  /*if (! RTC.isrunning()) {
   Serial.println("RTC is NOT running!");
   // following line sets the RTC to the date & time this sketch was compiled
   //RTC.adjust(DateTime(__DATE__, __TIME__));
   }*/

 if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    xbee.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));
    statusLight(1,0,1);
    delay(10000);
  }
  


  //delay(500);
  /*digitalWrite(MCUONLED, LOW);
  delay(500);
  digitalWrite(MCUONLED, HIGH);
  delay(500);*/
  digitalWrite(MCUONLED, HIGH);
  digitalWrite(STAT1, LOW);
  /*
  LCDbrightness(255);
   LCDturnDisplayOn(); //elsewhere
   LCDclearScreen(); //clears LCD
   LCDsetPosition(1,1); //splash
   myLCD.print(" The COM-BAT System ");
   LCDsetPosition(2,1);
   myLCD.print("Built by Jake Tesler");
   LCDsetPosition(4,1);
   myLCD.print("Loading...");
   */



  LCDturnDisplayOn(); //elsewhere
  LCDclearScreen(); //clears LCD
  LCDsetPosition(1,1); //splash
  myLCD.print(F("Jim: SenseBot, A.I.")); 
  xbee.println(F("Jim: SenseBot, A.I."));
  LCDsetPosition(2,1);
  myLCD.print(F("Built by Jake Tesler")); 
  xbee.println(F("Built by Jake Tesler"));
  LCDsetPosition(4,1);
  myLCD.print(F("Loading"));
  /*for(int itime = 0; itime <= 1; itime++)
   { 
   LCDsetPosition(4,8);
   myLCD.print("   ");
   int curDot = 0;
   for(int pixeldot = 1; pixeldot < 4; pixeldot++);
   {
   LCDsetPosition(4, curDot+8);
   myLCD.print(".");
   curDot++;
   delay(300);
   }
   
   delay(300);
   }*/



  static int dotdelay = 100;
  LCDsetPosition(4,8); 
  myLCD.print("."); 
  delay(dotdelay);
  LCDsetPosition(4,9); 
  myLCD.print("."); 
  delay(dotdelay);
  LCDsetPosition(4,10); 
  myLCD.print(".");
  delay(750);
  /*LCDsetPosition(4,8); myLCD.print("   "); delay(dotdelay);
   LCDsetPosition(4,8); myLCD.print("."); delay(dotdelay);
   LCDsetPosition(4,9); myLCD.print("."); delay(dotdelay);
   LCDsetPosition(4,10); myLCD.print("."); delay(500);
   //cyclone(3, 1000, 4, 12); //delay(500);
   
   
   //A funky loading animation
   LCDsetPosition(4,17); myLCD.write(0x4F); myLCD.print("_o"); delay(167);
   LCDsetPosition(3,17); myLCD.print("_ _"); delay(333);
   LCDsetPosition(3,19); myLCD.print("-"); delay(167);
   LCDsetPosition(4,20); myLCD.print("?"); delay(333);
   LCDsetPosition(3,19); myLCD.print("_"); delay(167);
   LCDsetPosition(3,19); myLCD.print("-"); delay(333);*/
  delay(100);
  LCDclearScreen();
  ///    #### We can re-enable these two code blocks for final release, unless space is an issue

  xbee.println(F("XBee Sleep..."));
  if(debug_setup) Serial.println(F("XBee Sleep..."));
  digitalWrite(XBEESLEEPD, xbeeSleep); //start XBee sleep
  xbeeButtonLED = HIGH;
  analogWrite(XBEELED, ledLuxLevel); //turn on XBee Button LED
  xbee.println(F("done. If you're reading this, you're not on wireless (or something broke)."));
  if(debug_setup) Serial.println(F("done. If you're reading this, you're not on wireless (or something broke)."));


  //warningShown = 0;  //initialize warning system
  //attachInterrupt(0, interrupt0, LOW); //digital pin 2 //should be last in setup
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  delay(250);
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  if(debug_setup) Serial.println(F("AboutToSetupInterrupt"));
  attachInterrupt(0, interrupt0, FALLING); //digital pin 2 //should be last in setup
  attachInterrupt(1, interrupt1, FALLING); //digital pin 3 //should be last in setup // laser
  //attachInterrupt(0, interrupt0, CHANGE); //digital pin 2
  if(debug_setup) Serial.println("Exit Setup");
}


void loop() {

  //##DEBUG
  if(debug_loop) {
    Serial.print("MODE ");
    Serial.println(mode);
    //Serial.println(fuelGauge.stateOfCharge());
    Serial.print("INITSET ");
    Serial.println(initSet);
    Serial.print("Free Mem:" );
    Serial.print(freeRam()); 
    Serial.println(" of 2048"); 
    Serial.println();
  }
  xbee.print(F("Free Mem: ")); 
  xbee.print(freeRam()); 
  xbee.println(F(" bytes of 2048"));

  if (mode == -1)
  {
    buzz(BUZZER,400,200); 
    delay(200); //buzz on pin 9 at 400hz for 200ms
    //  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
    mode = 0;
    if(debug_loop) Serial.println("ModeSetTo0");
  }

  if (initSet == 0 && mode != 3) { 
    laser_pwr = 0; 
    digitalWrite(LASEREN, HIGH); 
    digitalWrite(STAT2, LOW); 
  } //disable laser

  //set title bar...mode titles cannot be longer than 14char
  /*if (laser_pwr > 0) { LCDsetPosition(1,13); myLCD.print("L"); }
   else               { LCDsetPosition(1,13); myLCD.print(" "); }*/


  //GLOBAL TIME
  if (timeOn) {
    time_t t = processSyncMessage();
    if (t != 0) { 
      RTC.set(t); 
      setTime(t); 
    } // set the RTC and the system time to the received value
    //if(debug_loop) { Serial.print(curTimeMDY); Serial.print(" "); Serial.println(curTimeHMS); }
    if(debug_loop) { 
      Serial.print(genCurTimeMDY()); 
      Serial.print(" "); 
      Serial.println(genCurTimeHMS()); 
    }
    //xbee.print(genCurTimeMDY()); xbee.print(" "); xbee.println(genCurTimeHMS()); 
    xbee.println(genCurTimeMDY()+ " " + genCurTimeHMS()); 
  }


  //GLOBAL BATTERY STAT
  float battLevel = fuelGauge.stateOfCharge();
  //Serial.print("global batt "); Serial.println(battLevel);
  LCDsetPosition(1,15); 
  if (battLevel < 100 && battLevel >= 10)
  {
    myLCD.print("B"); 
    xbee.print("B");//batt logo;
    myLCD.print(battLevel,1); 
    xbee.print(battLevel,1); //[,1]=.1 (include decimal)
    LCDsetPosition(1,20); 
    myLCD.print("%"); 
    xbee.println("%");
  }
  else if (battLevel < 10 && battLevel > -0.01)
  {
    myLCD.print("B0"); //batt logo;
    myLCD.print(battLevel,1); 
    xbee.print(battLevel,1); //16+1 for B"zero" //[,1]=.1 (include decimal)
    LCDsetPosition(1,20); 
    myLCD.print("%"); 
    xbee.println("%");
  }
  else if (battLevel >= 100 && 
    battLevel < 255)    { 
    myLCD.print("B FULL"); 
    xbee.println("Battery Full"); 
  } //batt logo;
  else if (battLevel >= 255)   { 
    myLCD.print("PW_ERR"); 
    xbee.println(F("Power Error")); 
    digitalWrite(ERRORLED, HIGH); 
  }
  else if (battLevel <= -0.01) { 
    myLCD.print("PWE_LW"); 
    xbee.println(F("Battery Low Error")); 
    digitalWrite(ERRORLED, HIGH); 
  }
  else                         { 
    myLCD.print("B_SNSR"); 
    xbee.println(F("Battery Sensor Error")); 
    digitalWrite(ERRORLED, HIGH); 
  }



  if (mode == 1) { 
    mode1(); 
  } //accelerometer/compass
  if (mode == 2) { 
    mode2(); 
  } //atmospheric mode2
  if (mode == 0) { 
    mode0(); 
  } //off-recharge0
  if (mode == 3) { 
    mode3(); 
  } //lux/laser
  if (mode == 4) { 
    mode4(); 
  } //initial science warning


  //delay(500); //delay until next global event  
  delay(globalDelay);
} //END void loop()




void mode0() //off-recharge
{
  if(debug_mode0) Serial.println("InsideMode0");
  if (mode == 0 || mode == -1) //correction for always displayed
  {
    if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility   
      if (prevMode == 0 || prevMode == 1 || prevMode == 2) //(prevMode != 3 || prevMode != 4) //if abnormal behavior
      {
        mode = 3; //go back
        initSet = 0;
      }
      else { 
        prevMode = 0; 
      } //else set to current mode
      LCDclearScreen();
      //delay(100);
      LCDsetPosition(1,1);
      myLCD.print("OFF/CHG");
      if(debug_mode0) Serial.println("OFF/CHG");
      xbee.println("Off / Charge Mode");
      //animStep = initAnimStep;


      //delay(50);
      analogWrite(BUTTONLED, 0);
      delay(250);
      interrupts();
      analogWrite(BUTTONLED, ledLuxLevel);
      //interrupts();

      if((millis()/1000) > 15) buzz(BUZZER,400,200); //buzz on pin 9 at 400hz for 200ms 
      /*if alive for more than 10 sec, buzz for 200ms.*/
    }

    /*
    if (battLevel >= 254) //battery disconnected error
     {
     LCDsetPosition(2,1); myLCD.print("BATTERY ERROR:");
     LCDsetPosition(3,1); myLCD.print("Upper Out of Range");
     //Serial.println("Battery Error");
     }
     else if (battLevel < 101 && battLevel >= 0) //normal battery
     { }//confusedAnimation(animStep); animStep++;  //do normal stuff, but animations have been discontinued
     else if (battLevel < 0) //unknown sub-zero data reported
     {
     LCDsetPosition(2,1); myLCD.print("BATTERY ERROR:");
     LCDsetPosition(3,1); myLCD.print("Lower Out of Range");
     if(debug_mode0) Serial.println("Battery Error");
     }*/
    float battLevel = fuelGauge.stateOfCharge();
    if (battLevel >= 254 || battLevel < 0) { 
      statusLight(1,0,1); 
    }

    //display time
    if(timeOn) {
      String curTimeHMS = genCurTimeHMS();
      String curTimeMDY = genCurTimeMDY();
      LCDsetPosition(3,1); 
      myLCD.print(curTimeMDY + " " + curTimeHMS);
    }

    //3rd line Temperature
    LCDsetPosition(3,10);
    myLCD.print(altbar.readTempF(), 1); //prints the temperature to LCD with one decimal place
    myLCD.write(0b11011111);
    myLCD.print("F");

    //Uptime reporting
    LCDsetPosition(4, 1); 
    myLCD.print("Up: ");
    if(millis()/1000 >= 60) {
      myLCD.print((millis()/1000)/60,1); 
      myLCD.print("m"); 
    } //mins
    myLCD.print(((millis()/1000) % 60),1);
    myLCD.print("s"); //secs

  }
  /*else
   {
   interrupt0(); //crash protection of some sort?
   }*/
}


void mode1() //Accelerometer/Compass
{
  if(debug_mode1)Serial.println("InsideMode1");
  if (initSet != 1)
  {
    initSet = 1; //should be first because of interrupt possibility
    if(debug_mode1)Serial.println("InitSET To 1");
    if (prevMode == 1 || prevMode == 2 || prevMode == 3) //(prevMode != 0 || prevMode != 4)
    {
      mode = 0;
      initSet = 0;
      if(debug_mode1)Serial.println("SwitchingTo0, Prev Was 1,2,3");

      //This block is to generate an error if the sensor isn't present, as we know this stalls the MCU.
      statusLight(0,1,1); 
      lsm.heading(); 
      digitalWrite(ERRORLED, LOW); 
      digitalWrite(STAT2, LOW);
    }

    else {  ////if (prevMode == 0)
      prevMode = 1; 
      LCDclearScreen(); 
      LCDsetPosition(1,1); 
      myLCD.print("Where Am I?   ");
      if(debug_mode1)Serial.println("Where Am I?");
      LCDsetPosition(2,1); 
      myLCD.print("Heading:");
    } 

  }

  float curHeading = lsm.heading();
  /*LCDsetPosition(2,1);
   myLCD.print("Heading: "); myLCD.print(curHeading);*/  //This would save space
  LCDsetPosition(2,9); 
  myLCD.print(curHeading);
  if(debug_mode1) { 
    Serial.print("Heading: "); 
    Serial.println(curHeading); 
  }

  lsm.read();

  LCDsetPosition(3,1);
  myLCD.print("A X: ");
  myLCD.print(lsm.a.x);
  myLCD.print("Y: ");
  myLCD.print(lsm.a.y);
  myLCD.print("Z: ");
  myLCD.print(lsm.a.z);
  LCDsetPosition(4,1);
  myLCD.print("M X: ");
  myLCD.print(lsm.m.x);
  myLCD.print("Y: ");
  myLCD.print(lsm.m.y);
  myLCD.print("Z: ");
  myLCD.print(lsm.m.z);

  analogWrite(BUTTONLED, 0);
  delay(200);
  interrupts();
  analogWrite(BUTTONLED, ledLuxLevel);
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms



}

void mode2() //Altitude
{
  if(debug_mode2)Serial.println("InsideMode2");
  if (initSet != 1)
  {
    initSet = 1; //should be first because of interrupt possibility
    //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0
    if (prevMode != 1)
    {
      mode = 1;
      initSet = 0;
    }
    else { 
      prevMode = 2; 
    }

    LCDclearScreen();
    delay(50);
    LCDclearScreen();
    delay(100);

    LCDsetPosition(4,1); 
    myLCD.print("LOADING...");

    //EEPROM.write(1,2);


    analogWrite(BUTTONLED, 0);
    delay(200);
    interrupts();
    analogWrite(BUTTONLED, ledLuxLevel);
    buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms


    LCDclearScreen();
    delay(50);
    //LCDclearScreenFull();


    LCDsetPosition(1,1); 
    myLCD.print("Atmospheric");
    LCDsetPosition(2,1); 
    myLCD.print("Temp:");
    LCDsetPosition(3,1); 
    myLCD.print("Pressure:");
    LCDsetPosition(4,1); 
    myLCD.print("Altitude:");


    //interrupts();
    //delay(100);
  }


  float curTemp = altbar.readTempF();
  float curTempC = altbar.readTemp();
  float curHumidRHT = calHumid.getTrueRH(curTempC); //calculate true %RH based from Temperature
  //float curHumidRH = calHumid.getSensorRH(); //calculate relative %RH

  if (curTemp > -200)
  {
    LCDsetPosition(2,7);
    myLCD.print(curTemp, 1); //prints the temperature to LCD with one decimal place
    myLCD.write(0b11011111);
    myLCD.print("F | H");
    //humidity
    myLCD.print(curHumidRHT, 1); //prints the humidity with one decimal place
    myLCD.write(0b00100101); //'%'
  }
  else { 
    myLCD.print("Temp Error"); 
  }

  if(debug_mode2) {
    Serial.print(curTemp);
    Serial.println("F | ");
    Serial.print(curHumidRHT);
    Serial.println("% RH");
  }

  LCDsetPosition(3,11);
  float curBarPres = altbar.readPressure();
  if (curBarPres < 115000) { 
    myLCD.print(curBarPres, 0); 
    myLCD.print(" Pa"); 
  }
  else { 
    myLCD.print(">1k kPa"); 
  }
  if(debug_mode2) { 
    Serial.print(curBarPres); 
    Serial.println(" Pa"); 
  }



  LCDsetPosition(4,11);
  float curBarAlt = altbar.readAltitudeFt();
  if (curBarAlt < 10000 && curBarAlt > -2000) { 
    myLCD.print((curBarAlt), 1); 
    myLCD.print(" Ft."); 
  }//(" Feet ");
  else { 
    myLCD.print(">10k Feet"); 
  }


  //Serial.println(curTemp);
  //Serial.println(curBarAlt);
}


void mode3() //
{
  if(debug_mode3)Serial.println(F("InsideMode3"));
  if (initSet != 1) {
    initSet = 1; //should be first because of interrupt possibility
    if (prevMode != 2)
    {
      mode = 2;
      initSet = 0;
    }
    else { 
      prevMode = 3; 
    }

    LCDclearScreen(); 
    delay(100);

    LCDsetPosition(4,1); 
    myLCD.print("LOADING...");

    //EEPROM.write(1,2);


    analogWrite(BUTTONLED, 0);
    delay(200);
    interrupts();
    analogWrite(BUTTONLED, ledLuxLevel);
    buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms


    LCDclearScreen();
    delay(50);
    //LCDclearScreenFull();


    //LCDsetPosition(1,1); 
    LCDsetPosition(1,1); 
    myLCD.print("Light Sensors");
    LCDsetPosition(2,1); 
    myLCD.print("Lux: ");
    //LCDsetPosition(3,1);
    LCDsetPosition(4,1); 
    myLCD.print("Laser: Press Button");

    //interrupts();
    //delay(100);
  }
  
  double currentLuxLevel;
  static int tslSwitchCounter;
  unsigned int data0, data1;
  if (tsl.getData(data0,data1)) { //if we have connectivity
    
    if (tslSwitchCounter > 20)
    {
      msInt=402; tslTime=2; //0 = 13.7ms, 1 = 101ms, 2 = 402ms, 3 = manual
      tsl.setTiming(tslGain,tslTime,msInt);
      tslSwitchCounter = 0;
    }
    else if (tslSwitchCounter > 0) { tslSwitchCounter++; }
    
    boolean good = tsl.getLux(tslGain,msInt,data0,data1,currentLuxLevel); //do we have a good value [this also generates lux value]
    if (data0 == 65535 || data1 == 65535)
    {
      msInt=101; tslTime=1; //0 = 13.7ms, 1 = 101ms, 2 = 402ms, 3 = manual
      tsl.setTiming(tslGain,tslTime,msInt);
      Serial.println("switcing");
      LCDsetPosition(3,5); 
      myLCD.print("Switching int ms"); 
      delay(500);
      tslSwitchCounter = 1;
    }
    
    LCDsetPosition(3,5); 
    myLCD.print(currentLuxLevel); 
    if(tslSwitchCounter > 0) { myLCD.print("sw"); }
    myLCD.print(" | d0: ");
    myLCD.print(data0);
    myLCD.print(" d1: ");
    myLCD.print(data1);
  }
  else
  {
    statusLight(1,0,1); 
    printTSLError(tsl.getError());
  }

  //ledLuxLevelFromSensor = map(currentLuxLevel, lightSensor.min_value, lightSensor.max_value, 0, 255);
  //int ledLuxLevelFromSensor = map(currentLuxLevel, lightSensor.min_value, lightSensor.max_value, 50, 255);
  /*if (currentLuxLevel < 20) */ analogWrite(BUTTONLED, map(currentLuxLevel, 0, 400, 50, 255)); //set the brightness of the LED to ~room brightness
  //else                      analogWrite(BUTTONLED, map(currentLuxLevel, 0, 2000, 50, 255)); //set the brightness of the LED to ~room brightness
  

  //Blink XBee Button LED
  if (xbeeButtonLED) { 
    digitalWrite(XBEELED, LOW); 
  }        //turn off XBee Button LED
  else               { 
    analogWrite(XBEELED, ledLuxLevel); 
  } //turn on XBBL to light level
  xbeeButtonLED = !xbeeButtonLED;

  //Laser strobe
  if (laser_pwr == 4)
  {
    digitalWrite(LASEREN, LOW); 
    digitalWrite(STAT2, HIGH); //enable laser
    laser_pwr = 3;
  }
  else if (laser_pwr == 3)
  {
    digitalWrite(LASEREN, HIGH); 
    digitalWrite(STAT2, LOW); //disable laser
    laser_pwr = 4;
  }

}

void mode4() //warning
{
  if(debug_mode4) Serial.println("InsideMode4");
  if (prevMode != 0) //if abnormal behavior
  {
    mode = 0; //go back
    initSet = 0;
  }
  if (prevMode == 0) // else
  {
    //prevmode = 4; //else set to current mode //RECTIFIED, "MODE 4" ISN'T A MODE!!
    warningShown = 1; //to activate next step when interrupt is called
    if(debug_mode4) Serial.println(F("WarningShownEquals1"));
    //interrupts();
    //initSet = 0;
    unsigned long previousWarnMillis = millis();
    //previousWarnMillis = millis();
    boolean curWarnScreen = true; //T = 1; F = 2
    if(debug_mode4) Serial.println("WarnScreen1");
    interrupts();
    while(warningShown == 1)
    {
      //interrupts(); //moved up
      if (curWarnScreen)
      {
        LCDsetPosition(1,1); 
        myLCD.print(F("WARNING: Do not use this robot for scien-tific purposes, as"));
        LCDsetPosition(4,1); 
        myLCD.print(F("Press to continue..."));
        //delay(100); 
      }
      else //if (!(curWarnScreen)) //
      {
        LCDsetPosition(1,1); 
        myLCD.print(F("it may sometimes    "));
        LCDsetPosition(2,1); 
        myLCD.print(F("yield inaccurate or imprecise data.     "));
        //delay(100); 
      }

      if (millis() - previousWarnMillis > 2500)
      {
        if(debug_mode4) Serial.println("WarnScreenChange");
        curWarnScreen = !(curWarnScreen);
        previousWarnMillis = millis();
      }
    }

    /*
    while(warningShown == 1)
     {
     interrupts();
     LCDsetPosition(1,1);
     myLCD.print("WARNING: Do not use this device for scie-ntific purposes, as");
     LCDsetPosition(4,1);
     myLCD.print("Press to continue...");
     delay(2500);
     
     LCDsetPosition(1,1);
     myLCD.print("it may sometimes    ");
     LCDsetPosition(2,1);
     myLCD.print("yield inaccurate or imprecise data.     ");
     delay(2500);
     }
     */
  }
}

volatile unsigned long last_interrupt_time = 0;
//volatile unsigned long interrupt_time = 0;
//volatile unsigned long interrupt_time = millis();
void interrupt0()
{
  //debounce protection

  //interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (millis() - last_interrupt_time > 200) 
  {
    noInterrupts();
    if(mode != -1) { 
      buzz(BUZZER,500,200); 
    } //buzz on pin 9 at 500hz for 200ms
    Serial.println("InsideInterrupt");
    digitalWrite(MCUONLED, LOW); 
    statusLight(0,0,1);
    //digitalWrite(BUTTONLED, LOW);
    analogWrite(BUTTONLED, ledLuxLevel);

    if (warningShown == 0 || warningShown == -1) //if just pressed for very first time
    {
      mode = 4; //send to warning screen when loop() will call mode4();
      Serial.println("switchToWarn");
    }
    else if (warningShown == 1) //if TOC just confirmed/accepted [from warning screen]
    {
      mode = 1;
      warningShown = 2;
      prevMode = 0;
      initSet = 0;
      //prevmode = 4; ##previous mode would be 0, since technically "mode 4" isn't really a mode
      Serial.println("switchWarnShown1");
    }
    else if(mode == 1 && warningShown == 2) { 
      mode = 2; 
      if(debug_int0)Serial.println("switchto2"); 
    }
    else if(mode == 0 && warningShown == 2) { 
      mode = 1; 
      if(debug_int0)Serial.println("switchto1"); 
    }
    else if(mode == 2 && warningShown == 2) { 
      mode = 3; 
      if(debug_int0)Serial.println("switchto3"); 
    }
    else if(mode == 3 && warningShown == 2) { 
      mode = 0; 
      if(debug_int0)Serial.println("switchto0"); 
    }
    else                                    { 
      mode = 0; 
      if(debug_int0)Serial.println("switchfrom_elseto0"); 
    } 

    initSet = 0;
    //digitalWrite(BUTTONLED, HIGH);
    //Serial.print("Interrupt Set Mode: "); //  Serial.println(mode);
    analogWrite(BUTTONLED, ledLuxLevel);
    digitalWrite(MCUONLED, HIGH);  
    digitalWrite(ERRORLED, LOW);

  }/*
  else //THIS CANNOT EXIST UNLESS ACTUALLY PLUGGED INTO USB, CAUSES BUGS OTHERWISE
   {
   Serial.println("buttonPressRegistered");
   }
   Serial.println(last_interrupt_time);*/
  last_interrupt_time = millis();
}



//volatile unsigned long last_interrupt1_time = 0;
//volatile unsigned long interrupt1_time = millis();
void interrupt1() {
  //interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  //if (interrupt_time - last_interrupt_time > 200)
  if (millis() - last_interrupt_time > 200) 
  {
    noInterrupts();
    if(mode != -1) { 
      buzz(BUZZER,500,200); 
    } //buzz on pin 9 at 500hz for 200ms

    digitalWrite(MCUONLED, HIGH); 
    digitalWrite(STAT1, HIGH); 
    digitalWrite(ERRORLED, HIGH);
    analogWrite(XBEELED, ledLuxLevel); //XBee Button LED
    //digitalWrite(BUTTONLED, LOW);
    analogWrite(BUTTONLED, ledLuxLevel);


    if (mode == 3) {
      if (laser_pwr == 0)        { digitalWrite(LASEREN, LOW); laser_pwr = 255; } //enable laser
      else if (laser_pwr == 255) { analogWrite(LASEREN, 127); laser_pwr = 127; }
      else if (laser_pwr == 127) { analogWrite(LASEREN, 77); laser_pwr = 77; }
      else if (laser_pwr == 77)  { analogWrite(LASEREN, 40); laser_pwr = 40; }
      else if (laser_pwr == 40)  { digitalWrite(LASEREN, LOW); laser_pwr = 4; } //enable laser
      else if (laser_pwr == 4 || //4 means strobe on, 3 strobe off, for this
               laser_pwr == 3)   { digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); laser_pwr = 0; } //disable laser
      else
      {
        digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); laser_pwr = 0; //disable laser
        xbee.println(F("Laser Var Error"));
      }
      if (laser_pwr > 0 && laser_pwr < 10) digitalWrite(STAT2, HIGH); //effectively, if laser is on (but we don't deal with strobing here) 
    }

    else { //mode != 3
      /////#####Insert XBee Sleep stuff here
      //maybe something about triggering and switching LED status
      /*if (xbeeSleep) { digitalWrite(XBEESLEEPD, LOW); }  //stop XBee sleep
      else           { digitalWrite(XBEESLEEPD, HIGH); } //start XBee sleep
      xbeeSleep = !xbeeSleep;*/
      xbeeSleep = !xbeeSleep;
      digitalWrite(XBEESLEEPD, xbeeSleep); //toggle XBee Sleep
      
    }
    //analogWrite(BUTTONLED, ledLuxLevel);
    digitalWrite(MCUONLED, LOW); 
    digitalWrite(STAT1, LOW); 
    digitalWrite(ERRORLED, LOW);
  }
  last_interrupt_time = millis();
}


void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000/frequency/2; // calculate the delay value between transitions
  //## 1 second's worth of microseconds, divided by the frequency, then split in half since
  //## there are two phases to each cycle
  long numCycles = frequency * length/ 1000; // calculate the number of cycles for proper timing
  //## multiply frequency, which is really cycles per second, by the number of seconds to 
  //## get the total number of cycles to produce
  for (long i=0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}


String genCurTimeHMS() {
  String genTimeHMS;
  genTimeHMS += hourFormat12();
  genTimeHMS += ":";
  genTimeHMS += minute();
  genTimeHMS += ":";
  genTimeHMS += second();
  genTimeHMS += " ";
  genTimeHMS += meridian();
  return genTimeHMS;
}
String genCurTimeMDY() {
  String genTimeMDY;
  genTimeMDY += month();
  genTimeMDY += "/";
  genTimeMDY += day();
  genTimeMDY += "/";
  genTimeMDY += year();
  return genTimeMDY;
}



/*void displayLightSensorDetails(void)
 {
 sensor_t sensor;
 tsl.getSensor(&sensor);
 Serial.println("------------------------------------");
 Serial.print  ("Sensor:       "); Serial.println(lightSensor.name);
 Serial.print  ("Driver Ver:   "); Serial.println(lightSensor.version);
 Serial.print  ("Unique ID:    "); Serial.println(lightSensor.sensor_id);
 Serial.print  ("Max Value:    "); Serial.print(lightSensor.max_value); Serial.println(" lux");
 Serial.print  ("Min Value:    "); Serial.print(lightSensor.min_value); Serial.println(" lux");
 Serial.print  ("Resolution:   "); Serial.print(lightSensor.resolution); Serial.println(" lux");  
 Serial.println("------------------------------------");
 Serial.println("");
 delay(500);
 }
 */
void configureLightSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
//tsl.enableAutoGain(true);          /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
//tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  /*Serial.println("------------------------------------");
   Serial.print  ("Gain:         "); Serial.println("Auto");
   Serial.print  ("Timing:       "); Serial.println("13 ms");
   Serial.println("------------------------------------");*/
}



int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


// variables created by the build process when compiling the sketch
extern int __bss_end;
extern void *__brkval;

// function to return the amount of free RAM
/*int memoryFree()
 {
 int freeValue;
 
 if ((int)__brkval == 0)
 freeValue = ((int)&freeValue) - ((int)&__bss_end);
 else
 freeValue = ((int)&freeValue) - ((int)__brkval);
 
 return freeValue;
 }
 */

//RTC TIME SET CODE
//code to process time sync messages from the serial port
#define TIME_HEADER  "T"   //Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; //Jan 1 2013 
  if(Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if( pctime < DEFAULT_TIME) { pctime = 0L; } //check the value is a valid time (greater than Jan 1 2013)
  }                                             //return 0 to indicate that the time is not valid
  return pctime;
}


void printTSLError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  if(debug_mode3) {
    xbee.print("I2C error: ");
    xbee.print(error,DEC);
    xbee.print(", ");
    
    switch(error)
    {
      case 0:
        xbee.println("success");
        break;
      case 1:
        xbee.println("data too long for transmit buffer");
        break;
      case 2:
        xbee.println("received NACK on address (disconnected?)");
        break;
      case 3:
        xbee.println("received NACK on data");
        break;
      case 4:
        xbee.println("other error");
        break;
      default:
        xbee.println("unknown error");
    }
  }
}


/* SWITCH HOLD [DEBOUNCE?] CODE?????
 int varSWITCH_PIN = 0;  //variable to store switch presses
 
 if (digitalRead(SWITCH_PIN) == LOW)
 {
 time = millis();
 delay(200); //debounce
 
 // check if the switch is pressed for longer than 1 second.
 if(digitalRead(SWITCH_PIN) == LOW && time - millis() >1000) 
 
 {
 varSWITCH_PIN++;  //add 1 Step to next Mode in setup
 if(varSWITCH_PIN==8){varSWITCH_PIN=0;}       //switch back to 0 after the required modes
 
 // if it is a short press <1000 
 } else
 do something  
 
 }
 */


