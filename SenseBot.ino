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
 *
 *  Modes are as follows: Off - Accelerometer/Compass - Altitude(Atmosphere, Temp, Altitude, Humididy, Pressure) - Light(lux, laser) - Warning
 *
 *  LED Indicator Lights are used as follows: 
 *  
 *  SYS-ON: Indicates that the entire SenseBot System is receiving power. (SOLDER PAD MUST BE ENABLED)
 *  MCU-ON: This LED will be illuminated if the MCU is powered and is running within the loop cycle of the program (beyond setup()).
 *  STAT1:  Generic indicator light, echoes Pin 13. Should be on constantly (except for blinking) if no issues are present.
 *  STAT2:  Laser-enabled light. If the laser is on, this will be illuminated.
 *  ERROR:  This LED will illuminate if an error within the program has occurred, or possibly if some other hardware error has occurred. 
 *          Likely causes of ERROR will be a failed interrupt execution or a battery state failure. 
 *  OTHER STATES: [ERROR] + [STAT2]: LSM303D initialization failed.
 *  //Hint: [ON] / (OFF)
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
#include "MAX1704.h"
#include <LSM303.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <MPL3115A2.h>
#include <HIH4030.h>
#include <Time.h> //custom-adapted library
#include <DS1307RTC.h>


#define PCBVERION1.0
static double pcbVersion = 1.0;

//Clarify definitions for pin reference
#ifdef PCBVERION1.0
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

#elif defined PCBVERION2.0
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

//Mode Information
volatile signed int mode = -1; //initial mode
volatile signed int prevMode = -1; //mode == 0 initially (really) and this will 
                  //##confirm the check for mode accuracy...[(4)-0-1-2-3] cycle
volatile boolean initSet = 0; //initial settings for each mode bit
volatile signed int warningShown = -1;

volatile signed int laser_pwr = 0;
volatile boolean xbeeSleep = 0;
boolean xbeeButtonLED = LOW;
static int ledLuxLevel = 200; //mode/xbee button led brightness

String curTimeHMS;
String curTimeMDY;
boolean timeOn; //are we enabling time for this boot?

//Debug On/Off (remember to switch below)
                                                        static boolean debug = false; //#######SET DEBUG (Note: this disables XBee/Laser Interrupt)


//Animation tracker
//signed int animStep = -17; //tracker for confusedAnimation()
//static signed int initAnimStep = -17;


static int globalDelay = 250; //global event delay in ms (milliseconds)


MAX1704 fuelGauge; //I2C - initialize fuel gauge
MPL3115A2 altbar; //I2C - initialize barometer/altimeter/temperature
LSM303 lsm; //I2C - initialize compass/accelerometer
//HIH4030 uncalHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC); //I2C - initialize humidity sensor
HIH4030 calHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC); //I2C - initialize humidity sensor (calibrated)
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); //I2C - initialize light sensor

SoftwareSerial xbee(7,6); //7 iRX, 6 iTX* //initialize XBee
//SendOnlySoftwareSerial myLCD(4); //4 TX* ##CUSTOM //redefined in lcdCommands.h
//SoftwareSerial myLCD(5,4); //5 RX, 4 TX //redefined in lcdCommands.h ##not used

void setup()
{
  pinMode(2, INPUT_PULLUP); //for interrupt 0 (this is the mode button)
  pinMode(3, INPUT_PULLUP); //for interrupt 1 (this is the xbee interrupt) #PWM
//pinMode(4, OUTPUT);  //LCD RX/TX is on pin 4 **SendOnlySoftwareSerial
  pinMode(5, OUTPUT);  //Xbee Wireless Interupt LED #PWM
//pinMode(6, OUTPUT);  //Xbee (MCU to) RX is on pin 6 #PWM **SoftwareSerial
//pinMode(7, OUTPUT);  //Xbee (MCU to) TX is on pin 7 **SoftwareSerial
  pinMode(8, OUTPUT);  //LiPo Regulator Power-Save
  pinMode(9, OUTPUT);  //Buzzer #PWM
  pinMode(10, OUTPUT); //Laser EN #PWM
  pinMode(/*11*/BUTTONLED, OUTPUT); //LED for mode button (pin 11) #PWM
  pinMode(/*12*/RELAY, OUTPUT);     //SDA/SCL Relay (pin 12)
  pinMode(13, OUTPUT); //Builtin LED
  pinMode(A0, INPUT);  //Humidity sensor
  pinMode(A1, OUTPUT); //Xbee Sleep Pin
  pinMode(A2, OUTPUT); //MCU Status 1 LED
  pinMode(A3, OUTPUT); //MCU On LED
//pinMode(A4, INPUT);  //I2C SDA #PWM-S **Wire
//pinMode(A5, INPUT);  //I2C SCL #PWM-S **Wire
  pinMode(A6, OUTPUT); //MCU Status 2 LED
  pinMode(A7, OUTPUT); //Error LED
  
//digitalWrite(2, HIGH); //button power (un-float)
  analogWrite(BUTTONLED, ledLuxLevel); //turn on LED initially
  digitalWrite(PWSAVE, HIGH); //power-save mode (HIGH=off, LOW=on)
  digitalWrite(ONETHREE, HIGH); digitalWrite(STAT1, HIGH); //activate builtin LED
  digitalWrite(MCUONLED, HIGH); //activate 'MCU On' LED
  digitalWrite(RELAY, HIGH); //start SDA/SCL batt monitor relay, leave on permanently
  digitalWrite(LASEREN, HIGH); //disable laser
  digitalWrite(XBEELED, ledLuxLevel); //XBee Button LED
  
  
  myLCD.begin(9600); //initializes LCD at 9600 baud
  xbee.begin(115200); //start Xbee
                                                        
  if(debug) Serial.begin(9600); //necessary? //#######SET DEBUG
  if(debug) Serial.println("boot");
  xbee.println("boot");
  digitalWrite(ONETHREE, LOW); digitalWrite(STAT1, HIGH);
  Wire.begin(); //delay(5);                                                             //start I2C
  fuelGauge.reset(); fuelGauge.quickStart(); if(debug) Serial.println("fuel.done");     //initialize LiPo sensor
  altbar.begin();                            if(debug) Serial.println("AltSnsr.done");  //initialize MPL3115A2
  lsm.init(); lsm.enableDefault();           if(debug) Serial.println("ACM.done");      //initialize LSM303D
  calHumid.calibrate(H_SLOPE, H_OFFSET);     if(debug) Serial.println("Humid.done");    //initialize/calibrate HIH4030
  tsl.begin(); configureLightSensor();       if(debug) Serial.println("LuxSnsr.done");  //initialize TSL2561
  setSyncProvider(RTC.get);                                                             //the function to get the time from the RTC
  if (timeStatus() != timeSet) { timeOn = 0; if(debug) {Serial.println("No RTC"); } }   //initialize or kill RTC
  else                         { timeOn = 1; if(debug) {Serial.println("RTC set"); } }  //we are 1-offing this
  
   /*if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    //RTC.adjust(DateTime(__DATE__, __TIME__));
  }*/
  
  
  //delay(500);
  digitalWrite(ONETHREE, LOW); digitalWrite(STAT1, LOW);
  delay(500);
  digitalWrite(ONETHREE, HIGH); digitalWrite(STAT1, HIGH);
  delay(500);
  digitalWrite(STAT1, LOW);
  /*
  LCDbrightness(255);
  LCDturnDisplayOn(); //elsewhere
  LCDclearDisplay(); //clears LCD
  LCDsetPosition(1,1); //splash
  myLCD.print(" The COM-BAT System ");
  LCDsetPosition(2,1);
  myLCD.print("Built by Jake Tesler");
  LCDsetPosition(4,1);
  myLCD.print("Loading...");
  */
  
  
  
  LCDturnDisplayOn(); //elsewhere
  LCDclearDisplay(); //clears LCD
  LCDsetPosition(1,1); //splash
  myLCD.print(F("Jim: SenseBot, A.I.")); xbee.println(F("Jim: SenseBot, A.I."));
  LCDsetPosition(2,1);
  myLCD.print(F("Built by Jake Tesler")); xbee.println(F("Built by Jake Tesler"));
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
  LCDsetPosition(4,8); myLCD.print("."); delay(dotdelay);
  LCDsetPosition(4,9); myLCD.print("."); delay(dotdelay);
  LCDsetPosition(4,10); myLCD.print(".");
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
  LCDclearDisplay();
   ///    #### We can re-enable these two code blocks for final release, unless space is an issue
  
  xbee.println(F("Xbee Sleep..."));
  if(debug) Serial.println(F("Xbee Sleep..."));
  digitalWrite(XBEESLEEPD, HIGH); //start xbee sleep
  xbeeSleep = 1;
  xbeeButtonLED = HIGH;
  analogWrite(XBEELED, ledLuxLevel); //turn on XBee Button LED
  xbee.println(F("done. If you're reading this, you're not on wireless (or something broke)."));
  if(debug) Serial.println(F("done. If you're reading this, you're not on wireless (or something broke)."));
  
  
  //warningShown = 0;  //initialize warning system
  //attachInterrupt(0, interrupt0, LOW); //digital pin 2 //should be last in setup
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  delay(250);
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  if(debug) Serial.println(F("AboutToSetupInterrupt"));
  attachInterrupt(0, interrupt0, FALLING); //digital pin 2 //should be last in setup
  if(!debug)
  attachInterrupt(1, interrupt1, FALLING); //digital pin 3 //should be last in setup // laser
  //attachInterrupt(0, interrupt0, CHANGE); //digital pin 2
  if(debug) Serial.println("Exit Setup");
}


void loop() {

//##DEBUG
  if(debug) {
    Serial.print("MODE ");
    Serial.println(mode);
    //Serial.println(fuelGauge.stateOfCharge());
    Serial.print("INITSET ");
    Serial.println(initSet);
    Serial.print("Free Mem:" );
    Serial.print(freeRam()); Serial.println(" of 2048"); Serial.println();
  }
  xbee.print(F("Free Mem:" )); xbee.print(freeRam()); xbee.println(F(" bytes of 2048"));
  
  if (mode == -1)
  {
    buzz(BUZZER,400,200); delay(200); //buzz on pin 9 at 400hz for 200ms
//  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
    mode = 0;
    if(debug) Serial.println("ModeSetTo0");
  }

  if (initSet == 0 && mode != 3) { laser_pwr = 0; digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); } //disable laser
  
  //set title bar...mode titles cannot be longer than 14char
  /*if (laser_pwr > 0) { LCDsetPosition(1,13); myLCD.print("L"); }
  else               { LCDsetPosition(1,13); myLCD.print(" "); }*/


  //GLOBAL TIME
    if (timeOn) {
      time_t t = processSyncMessage();
      if (t != 0) { RTC.set(t); setTime(t); } // set the RTC and the system time to the received value
      genCurTimeHMS();
      genCurTimeMDY();
      if(debug) { Serial.print(curTimeMDY); Serial.print(" "); Serial.println(curTimeHMS); }
      xbee.print(curTimeMDY); xbee.print(" "); xbee.println(curTimeHMS);
    }
  //GLOBAL BATTERY STAT
  float battLevel = fuelGauge.stateOfCharge();
  //Serial.print("global batt "); Serial.println(battLevel);
  if (battLevel < 100 && battLevel >= 10)
  {
    LCDsetPosition(1,15); myLCD.print("B"); //batt logo;
    LCDsetPosition(1,16); myLCD.print(battLevel,1); xbee.println(battLevel,1); //[,1]=.1 (include decimal)
    LCDsetPosition(1,20); myLCD.print("%");
  }
  else if (battLevel < 10 && battLevel > -0.01)
  {
    LCDsetPosition(1,15); myLCD.print("B0"); //batt logo;
    LCDsetPosition(1,17); myLCD.print(battLevel,1); xbee.print(battLevel,1); //16+1 for B"zero" //[,1]=.1 (include decimal)
    LCDsetPosition(1,20); myLCD.print("%"); xbee.println("%");
  }
  else if (battLevel >= 100 && 
           battLevel < 255)    { LCDsetPosition(1,15); myLCD.print("B FULL"); xbee.println("Battery Full"); } //batt logo;
  else if (battLevel >= 255)   { LCDsetPosition(1,15); myLCD.print("PW_ERR"); xbee.println("Power Error"); digitalWrite(ERRORLED, HIGH); }
  else if (battLevel <= -0.01) { LCDsetPosition(1,15); myLCD.print("PWE_LW"); xbee.println(F("Battery Low Error")); digitalWrite(ERRORLED, HIGH); }
  else                         { LCDsetPosition(1,15); myLCD.print("B_SNSR"); xbee.println(F("Battery Sensor Error")); digitalWrite(ERRORLED, HIGH); }
  
  
  if (mode == 1) { mode1(); } //accelerometer/compass
  if (mode == 2) { mode2(); } //atmospheric mode2
  if (mode == 0) { mode0(); } //off-recharge0
  if (mode == 3) { mode3(); } //lux/laser
  if (mode == 4) { mode4(); } //initial science warning
  

  //delay(500); //delay until next global event  
  delay(globalDelay);
} //END void loop()




void mode0() //off-recharge
{
  if(debug) Serial.println("InsideMode0");
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
      else { prevMode = 0; } //else set to current mode
    
      LCDclearDisplay();
                                //delay(100);
      LCDsetPosition(1,1);
      myLCD.print("OFF/CHG");
      if(debug) Serial.println("OFF/CHG");
      animStep = initAnimStep;
  
  
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
    LCDsetPosition(3, 8);
    myLCD.print(battLevel0, 1);
    myLCD.print("%");
    */
    
    float battLevel0 = fuelGauge.stateOfCharge();
    
    if (battLevel0 >= 254) //battery disconnected error
    {
      LCDsetPosition(2,1); myLCD.print("BATTERY ERROR:");
      LCDsetPosition(3,1); myLCD.print("Upper Out of Range");
      //Serial.println("Battery Error");
    }
    else if (battLevel0 < 101 && battLevel0 >= 0) //normal battery
    { /*confusedAnimation(animStep); animStep++;*/ } //do normal stuff, but animations have been discontinued
    else if (battLevel0 < 0) //unknown sub-zero data reported
    {
      LCDsetPosition(2,1); myLCD.print("BATTERY ERROR:");
      LCDsetPosition(3,1); myLCD.print("Lower Out of Range");
      if(debug) Serial.println("Battery Error");
    }
    
    
    LCDsetPosition(4, 13); //display relay time remaining
    
    
    //Uptime reporting
    LCDsetPosition(4, 1); myLCD.print("Up: ");
    if(millis()/1000 >= 60) {myLCD.print((millis()/1000)/60,1); myLCD.print("m"); } //mins
    myLCD.print(((millis()/1000) % 60),1);
    myLCD.print("s"); //secs
    
    
    
                                                                                        //##########REIMPLEMENT TEMPERATURE HERE   
                                                                                        //##########REIMPLEMENT TEMPERATURE HERE
                                                                                        //##########REIMPLEMENT TEMPERATURE HERE
    //float curTemp = (((bar.readTemperature()) * 1.8) + 32);
    /*float curTemp = bar.readTemperature();
    curTemp = ((curTemp * 1.8) + 32);
    LCDsetPosition(3,10);
    if (curTemp > -200)
    {
      LCDsetPosition(2,15);
      myLCD.print(curTemp, 1); //prints the temperature to LCD with one decimal place
      myLCD.write(0b11011111);
      myLCD.print("F");
    }
    else
    {
      myLCD.print("Temp Error");
    }*/
    
  }
  /*else
  {
    interrupt0(); //crash protection of some sort?
  }*/  
}


void mode1() //Accelerometer/Compass
{
  if(debug)Serial.println("InsideMode1");
  if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility
      if(debug)Serial.println("InitSET To 1");
      if (prevMode == 1 || prevMode == 2 || prevMode == 3) //(prevMode != 0 || prevMode != 4)
      {
        mode = 0;
        initSet = 0;
        if(debug)Serial.println("SwitchingTo0, Prev Was 1,2,3");
        
        //This block is to generate an error if the sensor isn't present, as we know this stalls the MCU.
        digitalWrite(ERRORLED, HIGH); digitalWrite(STAT2, HIGH); lsm.heading(); digitalWrite(ERRORLED, LOW); digitalWrite(STAT2, LOW);
      }
      else { prevMode = 1; } ////if (prevMode == 0)
    }    
      LCDclearDisplay(); 
      delay(75);
      //LCDsetPosition(3,1); myLCD.print("                    ");
      //LCDsetPosition(1,1); myLCD.print("                    ");
      LCDsetPosition(1,1); myLCD.print("Where Am I?   ");
      if(debug)Serial.println("Where Am I?");
    
      float curHeading = lsm.heading();
      LCDsetPosition(2,1);
      myLCD.print("Heading: "); myLCD.print(curHeading);
      if(debug) { Serial.print("Heading: "); Serial.println(curHeading); }
      
      lsm.read();
      
      LCDsetPosition(3,1);
      myLCD.print("A ");
      myLCD.print("X: ");
      myLCD.print(lsm.a.x);
      myLCD.print("Y: ");
      myLCD.print(lsm.a.y);
      myLCD.print("Z: ");
      myLCD.print(lsm.a.z);
      LCDsetPosition(4,1);
      myLCD.print("M ");
      myLCD.print("X: ");
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
   
      //interrupts(); //re-enable after interrupt0() interrupts shutdown

  
}

void mode2() //Altitude
{
  if(debug)Serial.println("InsideMode2");
  if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility
      //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0
      if (prevMode != 1)
      {
        mode = 1;
        initSet = 0;
      }
      else { prevMode = 2; }
      
      LCDclearDisplay();
      delay(50);
      LCDclearScreen();
      delay(100);
      
      LCDsetPosition(4,1); myLCD.print("LOADING...");
      
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
      myLCD.print("              ");
      LCDsetPosition(1,1); myLCD.print("Atmospheric   ");
      LCDsetPosition(2,1); myLCD.print("Temp: ");
      LCDsetPosition(3,1); myLCD.print("Pressure: ");
      LCDsetPosition(4,1); myLCD.print("          ");
      LCDsetPosition(4,1); myLCD.print("Altitude:  ");
     
      
      //interrupts();
      //delay(100);
    }
    
    
  //float curTemp = (((bar.readTemperature()) * 1.8) + 32);
  float curTemp = altbar.readTempF();
  float curTempC = altbar.readTemp();
  float curHumidRHT = calHumid.getTrueRH(curTempC); //calculate true %RH based from Temperature
  //float curUncalHumidRHT = uncalHumid.getTrueRH(curTempC); //calculate true %RH based from Temperature
  float curHumidRH = calHumid.getSensorRH(); //calculate relative %RH
  
  if(debug) {
    Serial.print(curTemp);
    Serial.println("F");
    Serial.print(curHumidRHT);
    Serial.println(" %RH");
  }
  
  LCDsetPosition(2,7);
  if (curTemp > -200)
  {
    LCDsetPosition(2,7);
    myLCD.print(curTemp, 1); //prints the temperature to LCD with one decimal place
    myLCD.write(0b11011111);
    myLCD.print("F");
    //humidity
    myLCD.print(" H");
    myLCD.print(curHumidRHT, 1); //prints the humidity with one decimal place
    myLCD.write(0b00100101); //'%'
  }
  else { myLCD.print("Temp Error"); }


  
  float curBarPres = altbar.readPressure();
  LCDsetPosition(3,11);
  if (curBarPres < 115000) { myLCD.print(curBarPres, 0); myLCD.print(" Pa "); }
  else { myLCD.print(">1k kPa [E"); }
  if(debug) { Serial.print(curBarPres); Serial.println(" Pa"); }
  
  LCDsetPosition(4,11);
  float curBarAlt = altbar.readAltitudeFt();
  if (curBarAlt < 10000 && curBarAlt > -2000) { myLCD.print((curBarAlt), 1); myLCD.print(" Ft."); }//(" Feet ");
  else { myLCD.print(">10k Feet"); }
  
  
  //Serial.println(curTemp);
  //Serial.println(curBarAlt);
}


void mode3() //
{
  if(debug)Serial.println(F("InsideMode3"));
  if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility
      if (prevMode != 2)
      {
        mode = 2;
        initSet = 0;
      }
      else { prevMode = 3; }
      
      LCDclearDisplay(); delay(50);
      LCDclearScreen(); delay(100);
      
      LCDsetPosition(4,1); myLCD.print("LOADING...");
      
      //EEPROM.write(1,2);
  
      
      analogWrite(BUTTONLED, 0);
      delay(200);
      interrupts();
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
      
      
      LCDclearScreen();
      delay(50);
      //LCDclearScreenFull();
      
      
      LCDsetPosition(1,1); myLCD.print("              ");
      LCDsetPosition(1,1); myLCD.print("Light Sensors");
      LCDsetPosition(2,1); myLCD.print("Lux");
      LCDsetPosition(3,1); myLCD.print("              ");
      LCDsetPosition(4,1); myLCD.print("Laser: Press Button");
      
      //interrupts();
      //delay(100);
    }
     
       sensor_t lightSensor;
       tsl.getSensor(&lightSensor);
  if(debug) { Serial.print  ("Max Value:    "); Serial.print(lightSensor.max_value); Serial.println(" lux"); //test whether this changes, if not we may be able to make it only inquire once in setup
              Serial.print  ("Min Value:    "); Serial.print(lightSensor.min_value); Serial.println(" lux"); }
     
     
     sensors_event_t event;
     tsl.getEvent(&event);
     int currentLuxLevel = event.light;
  //if (event.light) { Serial.print(event.light); Serial.println(" lux"); }
  if (event.light && debug) { Serial.print(event.light); Serial.println(" lux"); }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    if(debug) Serial.println("Sensor overload");
  }
  
     
     //ledLuxLevelFromSensor = map(currentLuxLevel, lightSensor.min_value, lightSensor.max_value, 0, 255);
     //int ledLuxLevelFromSensor = map(currentLuxLevel, lightSensor.min_value, lightSensor.max_value, 50, 255);
     analogWrite(BUTTONLED, map(currentLuxLevel, lightSensor.min_value, lightSensor.max_value, 50, 255)); //set the brightness of the LED to ~room brightness
     
     //Blink XBee Button LED
     if (xbeeButtonLED) { digitalWrite(XBEELED, LOW); }        //turn off XBee Button LED
     else               { analogWrite(XBEELED, ledLuxLevel); } //turn on XBBL to light level
     xbeeButtonLED = !xbeeButtonLED;
     
     //Laser strobe
     if (laser_pwr == 4)
     {
       digitalWrite(LASEREN, LOW); digitalWrite(STAT2, HIGH); //enable laser
       laser_pwr = 3;
     }
     else if (laser_pwr == 3)
     {
       digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); //disable laser
       laser_pwr = 4;
     }
     
}

void mode4() //warning
{
  if(debug) Serial.println("InsideMode4");
  if (prevMode != 0) //if abnormal behavior
  {
    mode = 0; //go back
    initSet = 0;
  }
  if (prevMode == 0) // else
  {
    //prevmode = 4; //else set to current mode //RECTIFIED, "MODE 4" ISN'T A MODE!!
    warningShown = 1; //to activate next step when interrupt is called
    if(debug) Serial.println(F("WarningShownEquals1"));
    //interrupts();
    //initSet = 0;
    unsigned long previousWarnMillis = millis();
    //previousWarnMillis = millis();
    boolean curWarnScreen = true; //T = 1; F = 2
    if(debug) Serial.println("WarnScreen1");
    interrupts();
    while(warningShown == 1)
    {
      //interrupts(); //moved up
      if (curWarnScreen)
      {
        LCDsetPosition(1,1); myLCD.print(F("WARNING: Do not use this robot for scien-tific purposes, as"));
        LCDsetPosition(4,1); myLCD.print(F("Press to continue..."));
        //delay(100); 
      }
      else //if (!(curWarnScreen)) //
      {
        LCDsetPosition(1,1); myLCD.print(F("it may sometimes    "));
        LCDsetPosition(2,1); myLCD.print(F("yield inaccurate or imprecise data.     "));
        //delay(100); 
      }
      
      if (millis() - previousWarnMillis > 2500)
      {
        if(debug) Serial.println("WarnScreenChange");
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
    if(mode != -1) { buzz(BUZZER,500,200); } //buzz on pin 9 at 500hz for 200ms
    Serial.println("InsideInterrupt");
    digitalWrite(ONETHREE, LOW); digitalWrite(STAT1, LOW); digitalWrite(ERRORLED, HIGH);
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
  else if(mode == 1 && warningShown == 2) { mode = 2; if(debug)Serial.println("switchto2"); }
  else if(mode == 0 && warningShown == 2) { mode = 1; if(debug)Serial.println("switchto1"); }
  else if(mode == 2 && warningShown == 2) { mode = 3; if(debug)Serial.println("switchto3"); }
  else if(mode == 3 && warningShown == 2) { mode = 0; if(debug)Serial.println("switchto0"); }
  else                                    { mode = 0; if(debug)Serial.println("switchfrom_elseto0"); } 
  
  initSet = 0;
  //digitalWrite(BUTTONLED, HIGH);
  //Serial.print("Interrupt Set Mode: "); //  Serial.println(mode);
  analogWrite(BUTTONLED, ledLuxLevel);
  digitalWrite(ONETHREE, HIGH); digitalWrite(STAT1, HIGH); digitalWrite(ERRORLED, LOW);
  
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
    if(mode != -1) { buzz(BUZZER,500,200); } //buzz on pin 9 at 500hz for 200ms
    
    digitalWrite(ONETHREE, HIGH); digitalWrite(STAT1, LOW); digitalWrite(ERRORLED, HIGH);
    analogWrite(XBEELED, ledLuxLevel); //XBee Button LED
    //digitalWrite(BUTTONLED, LOW);
    analogWrite(BUTTONLED, ledLuxLevel);
    
    
    if (mode == 3) {
      if (laser_pwr == 0)
      {
        digitalWrite(LASEREN, LOW); digitalWrite(STAT2, HIGH); //enable laser
        laser_pwr = 255;
       
      }
      else if (laser_pwr == 255)
      {
        analogWrite(LASEREN, 127); digitalWrite(STAT2, HIGH); 
        laser_pwr = 127;
      }
      else if (laser_pwr == 127)
      {
        analogWrite(LASEREN, 77); digitalWrite(STAT2, HIGH); 
        laser_pwr = 77;
      }
       else if (laser_pwr == 77)
      {
        analogWrite(LASEREN, 40); digitalWrite(STAT2, HIGH); 
        laser_pwr = 40;
      }
      else if (laser_pwr == 40)
      {
        digitalWrite(LASEREN, LOW); digitalWrite(STAT2, HIGH); //enable laser
        laser_pwr = 4;
      }
      else if (laser_pwr == 4 || laser_pwr == 3) //4 means strobe on, 3 strobe off, for this
      {
        digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); //disable laser
        laser_pwr = 0;
      }
      else
      {
        digitalWrite(LASEREN, HIGH); digitalWrite(STAT2, LOW); //disable laser
        laser_pwr = 0;
        xbee.println(F("Laser Var Error"));
      }
    }

    
    
    else {
     /////#####Insert XBee Sleep stuff here
    //maybe something about triggering and switching LED status
      if (xbeeSleep) { digitalWrite(XBEESLEEPD, HIGH); }//start xbee sleep
      else { digitalWrite(XBEESLEEPD, LOW); } //stop xbee sleep
      xbeeSleep = !xbeeSleep;
    }
    
    //analogWrite(BUTTONLED, ledLuxLevel);
    digitalWrite(ONETHREE, HIGH); digitalWrite(STAT1, HIGH);
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
  tsl.enableAutoGain(true);          /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
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

void genCurTimeHMS() {
  curTimeHMS = "";
  curTimeHMS += hourFormat12();
  curTimeHMS += ":";
  curTimeHMS += minute();
  curTimeHMS += ":";
  curTimeHMS += second();
  curTimeHMS += " ";
  curTimeHMS += meridian();
}

void genCurTimeMDY() {
  curTimeMDY = "";
  curTimeMDY += month();
  curTimeMDY += "/";
  curTimeMDY += day();
  curTimeMDY += "/";
  curTimeMDY += year();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
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

