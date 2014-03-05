// Jim: The Self-Aware SenseBot//
// Written in Wiring, C, and C++ //

// ***** Coded by Jake Tesler ***** //

/* *************************************************************************
 *    ********* READ THESE INSTRUCTIONS! *********
 *  This project uses a custom PCB design (public design coming soon), custom enclosure, and various sensors and componentry, including the following: 
 *  Atmel ATmega328P-AU processor, Digi International/MaxStream XBee 1mW 802.15.4 Series 1 radio module (with trace antenna), STMicro LSM303D Accelerometer/Magnetometer and Compass,
 *  Freescale MPL3115A2 Temperature and Barometric Pressure sensor, Honeywell HIH4030 Humidity sensor, TexasAOS TSL2561 Light and Illumination (IR and Visible) sensor, 
 *  Maxim Integrated DS1307 RTC for Date/Time storage and management (daughtercard for v1), 
 *  Maxim MAX17043G+U LiPo Monitoring module, dual TI TPS61200/Microchip MCP73831T package for LiPo regulation and power distribution, 
 *  Pololu D24V6F3 and S10V4F5 for voltage regulation, Hitachi HD44780 PIC16F88-enabled character 20x4 Serial LCD panel, and SparkFun Red TTL 654.5nm Class 3A Laser (detachable). 
 *  The I2C LiPo SDA/SCL control uses a Omron G5V-2 Relay (DPDT, although DPST will work), and sound is produced by a Multicomp MCKPT-G1210 Piezo Buzzer. 
 *  
 *  This combined hardware produces a device with 7DOF (degrees of freedom), integrated temperature and (limited) weather monitoring, 
 *  area condition sensing and environmental surroundings awareness. 
 *  
 *  Information:
 *  Modes are as follows: Off - Accelerometer/Compass - Altitude(Atmosphere, Temp, Altitude, Humididy, Pressure) - Light(lux, laser) - Warning
 *  Debug Mode can be set in "debug.h".
 *
 *  LED Indicator Lights are used as follows: 
 *  SYS-ON: Indicates that the entire SenseBot System is receiving power. (SOLDER PAD MUST BE ENABLED)
 *  MCU-ON: This LED will be illuminated if the MCU is powered and is running. This LED's purpose is to echo the MCU's built-in onboard LED. 
 *  STAT1:  Generic indicator light. Used with LED Status Indicator States.
 *  STAT2:  Laser-enabled light. If the laser is on, this will be illuminated. [Also used as a Status Indicator Light, or SIL].
 *  ERROR:  This LED will illuminate if an error within the program has occurred, or possibly if some other hardware error has occurred. [SIL]
 *  LED Status Indicator Lights [SIL]:                               //Hint: [ON] (OFF) *IGNORE* {Mode}
 *   (STAT1) [STAT2] [ERROR] {*}: LSM303D initialization failed
 *   (STAT1) (STAT2) [ERROR] {*}: Interrupt 0 execution error
 *   [STAT1] (STAT2) [ERROR] {0}: Battery fault error
 *   [STAT1] (STAT2) [ERROR] {*}: Out-of-range Sensor error
 *   [STAT1] *STAT2* [ERROR] {3}: Interrupt 1 execution error
 *  
 *  TO-DO: Implement XBee transmission by rewriting lines rather than re-printing prefixes
 * 
 * "THE BEER-WARE LICENSE"
 * As long as you retain this notice you can do whatever you want with this stuff. 
 * If we meet some day, and you think this stuff is worth it, you can buy me a beer.
 *
 * *************************************************************************
 */
const String sbVersion = "v0.8.3r1";
#define PCBVERSION1.0
//#define PCBVERSION2.0
const double pcbVersion = 1.0;
//const double pcbVersion = 2.0;



#include "Arduino.h"
#include "Wire.h"
#include <SoftwareSerial.h>
#include <SendOnlySoftwareSerial.h>
#include <EEPROM.h>
#include "lcdCommands.h"
#include "animation.h"
#include "pluscode.h"
#include "debug.h"
#include "eeprom_set.h"
#include "MAX1704.h"
#include <LSM303.h>
#include <SFE_TSL2561.h>
#include <MPL3115A2.h>
#include <HIH4030.h>
#include <Time.h> //customized Arduino library
#include <DS1307RTC.h>
#include <avr/pgmspace.h>



//Clarify definitions for pin reference
#ifdef PCBVERSION1.0
#define LCDPIN 4
#define XBEELED 5
#define XBEETX 6
#define XBEERX 7
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
#define MIC 0

#elif defined PCBVERSION2.0
#define LCDPIN A2
#define XBEELED 5
#define XBEETX 4
#define XBEERX 7
#define BUZZER 6  
#define LASEREN 10
#define BUTTONLED 11
#define RELAY A3
#define ONETHREE 13
#define HIH4030_PIN A6
#define XBEESLEEPD A1
#define STAT1 9
#define MCUONLED 8
#define STAT2 12
#define ERRORLED A0
#define MIC A7

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
volatile signed char mode = -1; //initial mode
volatile signed char prevMode = -1; //mode == 0 initially (really) and this will 
//##confirm the check for mode accuracy...[(4)-0-1-2-3] cycle
volatile boolean initSet = 0; //initial settings for each mode bit
volatile signed char warningShown = -1;

volatile unsigned long last_interrupt_time = 0; //int0
//volatile unsigned long interrupt_time = millis(); //int0
//volatile unsigned long last_interrupt1_time = 0; //int1
//volatile unsigned long interrupt1_time = millis(); //int1

//laser
volatile uint8_t laser_pwr = 0;

//xbee
volatile boolean xbeeSleep = 0; //start XBee not Sleeping
boolean xbeeButtonLED = LOW;

//RTC
//String curTimeHMS;
//String curTimeMDY;
boolean timeOn; //are we enabling time for this boot?

//light sensor
const boolean tslGain = 0;  // Gain setting, 0 = X1, 1 = X16; //this is const only because we don't change it below...but it is change-able
uint8_t tslTime = 2; //integration time constant //0 = 13.7ms, 1 = 101ms, 2 = 402ms, 3 = manual, make sure to set both vars
unsigned int msInt;  // Integration ("shutter") time in milliseconds
//static int tslSwitchCounter = 0;

//how many cycles until the screen rewrites content
uint8_t redrawCounter;
const uint8_t redrawMax = 10;

volatile boolean xbeePrintMode = 1; //this is to see if we have printed status info to the XBee for mode switching
volatile boolean xbeePrintInfo = 0;

//Laser strobe delay
//balance these two
const uint8_t strobeKnockVal = 4; //how much to change the value by per delay [recommended: 1-10]
const uint8_t strobeStopVal = 5; //delay to wait for (ms) [recommended: 3-12]
//static boolean laserStrobeDir = 1; //1 tells to rise (0->255), 0 tells to fall (255->0)

//Animation tracker
//int8_t animStep = -17; //tracker for confusedAnimation()
//const int8_t initAnimStep = -17;

//Options
const unsigned int globalDelay = 250; //global event delay in ms (milliseconds) {suggested values: 250, 500}
  #define globalDelayEEP 1
const uint8_t ledLuxLevel = 200; //mode/XBee button LED brightness  (x out of 255)
  #define ledLuxLevelEEP 2
const uint8_t switchCount = 30; //the number of intervals that elapse before the light sensor switches reverts sensitivity
  #define switchCountEEP 3
const boolean powerSaveEnabled = false; //enable or disable power-save functions for the regulator ###experimental
  #define powerSaveEnabledEEP 4
const boolean eeprom_config = false; //configure EEPROM settings?

//initialize
MAX1704 fuelGauge;  //I2C - initialize fuel gauge
MPL3115A2 altbar;   //I2C - initialize barometer/altimeter/temperature
LSM303 lsm;         //I2C - initialize compass/accelerometer
SFE_TSL2561 tsl;    //I2C - initialize light sensor
HIH4030 calHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC);        //Analog - initialize humidity sensor (calibrated)
//HIH4030 uncalHumid(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC);    //Analog - initialize humidity sensor
SoftwareSerial xbee(XBEERX,XBEETX); //7 iRX, 6 or 4 iTX* //initialize XBee
//SendOnlySoftwareSerial myLCD(LCDPIN); //4 or A2 TX* ##CUSTOM //redefined in lcdCommands.h

void setup()
{ 
  if (pcbVersion==1.0) {
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
  else if (pcbVersion==2.0){setPins2();}
  
  analogWrite(BUTTONLED, ledLuxLevel);     //turn on LED initially
  if(pcbVersion==1.0) digitalWrite(PWSAVE, !powerSaveEnabled); //power-save mode (HIGH=off, LOW=on)
  digitalWrite(ONETHREE, HIGH);            //activate builtin LED
  digitalWrite(MCUONLED, LOW);             //we will activate 'MCU On' LED after the init setup
  digitalWrite(RELAY, HIGH);               //start SDA/SCL batt monitor relay, leave on permanently
  digitalWrite(LASEREN, HIGH);             //disable laser
  //digitalWrite(XBEELED, ledLuxLevel);      //XBee Button LED [inserted below]


  if(debug_serial) Serial.begin(9600);
  if(debug_setup) Serial.println("boot");
  if(debug_setup) printGlobals();
  
  if(eeprom_config) {eeprom_set(); while(1);}
  
  myLCD.begin(9600);                          //initialize LCD at 9600 baud
  xbee.begin(9600); delay(50); xbee.println(""); //xbee.println("boot");  //initialize XBee at 9600 baud
  Wire.begin(); //delay(5);                   //initialize I2C
  fuelGauge.reset(); fuelGauge.quickStart();                            if(debug_setup) Serial.println("fuel.done");         //initialize LiPo capacity sensor
  altbar.begin(); altbarStarter();                                      if(debug_setup) Serial.println("AltSnsr.done");      //initialize MPL3115A2
  lsm.init(); lsm.enableDefault();                                      if(debug_setup) Serial.println("ACM.done");          //initialize LSM303D
  calHumid.calibrate(H_SLOPE, H_OFFSET);                                if(debug_setup) Serial.println("Humid.done");        //initialize/calibrate HIH4030
  tsl.begin(); tsl.setPowerUp(); tsl.setTiming(tslGain,tslTime,msInt);  if(debug_setup) Serial.println("LuxSnsr.done");      //initialize TSL2561
  //setSyncProvider(RTC.get);                                             if(debug_setup) Serial.print("Get RTC...");          //the function to get the time from the RTC
  //if (timeStatus() != timeSet) { timeOn = 0;                            if(debug_setup) Serial.println("No RTC"); }          //initialize or kill RTC           
  //else                         { timeOn = 1;                            if(debug_setup) Serial.println("RTC set"); }         //we are 1-offing this
  if(debug_setup && !tsl.begin()) { //There was a problem detecting the TSL2561 ... check the connections
    xbee.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));
    statusLight(1,0,1); delay(10000); }
  
  //xbee.println("XBee Sleep..."); if(debug_setup) Serial.println(F("XBee Sleep..."));
  digitalWrite(XBEESLEEPD, xbeeSleep); //start or stop XBee sleep
  analogWrite(XBEELED, ledLuxLevel); xbeeButtonLED = HIGH; //turn on XBee Button LED
  

  //delay(500);
/*digitalWrite(MCUONLED, LOW); delay(500);
  digitalWrite(MCUONLED, HIGH); delay(500);*/
  digitalWrite(MCUONLED, HIGH);
  digitalWrite(STAT1, LOW);

  //LCDbrightness(255); //increases binary size a lot
  //LCDturnDisplayOn(); 
  LCDclearScreen(); //clears LCD
  LCDsetPosition(1,1); myLCD.print(F("  Jim The SenseBot"));  xbee.println(F("Jim The SenseBot")); //splash
  LCDsetPosition(2,1); myLCD.print(F("Built by Jake Tesler")); xbee.println(F("Built by Jake Tesler"));
  //xbee.println("[To sleep, press red button on the back.]"); 
  LCDsetPosition(3,1); myLCD.print(sbVersion); LCDsetPosition(3,11); myLCD.print(F("Build: ")); myLCD.print(pcbVersion);
  LCDsetPosition(4,1); myLCD.print(F("Loading")); xbee.print(F("Loading"));
  for (uint8_t itime = 0; itime <= 2; itime++)
  {
    LCDsetPosition(4,8); myLCD.print("   "); delay(250);
    LCDsetPosition(4,8);
    xbee.print(".");
    for(uint8_t pixelDot = 1; pixelDot < 4; pixelDot++) { myLCD.print("."); delay(250); }
  }
  xbee.println("Done!"); //xbee.println("");
  //cyclone(3, 1000, 4, 12);


/*
    //A funky loading animation
   LCDsetPosition(4,17); myLCD.write(0x4F); myLCD.print("_o"); delay(167);
   LCDsetPosition(3,17); myLCD.print("_ _"); delay(333);
   LCDsetPosition(3,19); myLCD.print("-"); delay(167);
   LCDsetPosition(4,20); myLCD.print("?"); delay(333);
   LCDsetPosition(3,19); myLCD.print("_"); delay(167);
   LCDsetPosition(3,19); myLCD.print("-"); delay(333);
*/
   
  //altbar.readTempF(); //for initial boot temperature eval
  altbar.readTemp();
  altbar.setModeBarometer(); altbar.readPressure(); altbar.readTemp();
  //delay(1500);
  //delay(100);
  LCDclearScreen();
  ///    #### We can re-enable these two code blocks for final release, unless space is an issue


  //warningShown = 0;  //initialize warning system
  buzz(BUZZER,400,200); delay(250); //buzz on pin 9 at 400hz for 200ms
  buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  if(debug_setup) Serial.println(F("AboutToSetupInterrupt"));
  attachInterrupt(0, interrupt0, FALLING); //digital pin 2 //should be last in setup
  attachInterrupt(1, interrupt1, FALLING); //digital pin 3 //should be last in setup //xbee/laser
//attachInterrupt(0, interrupt0, CHANGE);  //digital pin 2
//attachInterrupt(0, interrupt0, LOW);     //digital pin 2 //should be last in setup
  if(debug_setup) Serial.println("Exit Setup");
}


void loop() {
  //##DEBUG
  if(debug_loop) {
    Serial.print("MODE "); Serial.println(mode);
    //Serial.println(fuelGauge.stateOfCharge());
    Serial.print("INITSET "); Serial.println(initSet);
    Serial.print("Free Mem:" ); Serial.print(freeRam()); Serial.println(" of 2048"); 
    Serial.println();
  }
  
  
  //redraw protocol control
  if(redrawCounter > redrawMax) redrawCounter=0;
  else redrawCounter++;
  
  float battLevel = fuelGauge.stateOfCharge();
  
  if (initSet == 0) //if we just switched modes, execute these "housekeeping" commands
  {
    if (prevMode == 3 && mode != 3) { laser_pwr = 0; /*laserStrobeDir = 1;*/ digitalWrite(LASEREN, HIGH); 
                                    /*digitalWrite(STAT2, LOW);*/ digitalWrite(ONETHREE, LOW); } //disable laser
    if (pcbVersion==1.0 && powerSaveEnabled && mode != 0) digitalWrite(PWSAVE, HIGH);
    if (mode == -1) { buzz(BUZZER,400,200); delay(200); //buzz(BUZZER,400,200); //buzz on pin 9 at 400hz for 200ms
                      mode = 0; if(debug_loop) Serial.println("ModeSetTo0"); }
    digitalWrite(MCUONLED, HIGH);
    redrawCounter = 0;
    if(mode == 0) { //we can disable this if we want to add RTC for every mode
      if (!timeOn) {
        setSyncProvider(RTC.get);                  if(debug_setup) Serial.print("Get RTC...");   //the function to get the time from the RTC
        if (timeStatus() != timeSet) { timeOn = 0; if(debug_setup) Serial.println("No RTC"); }   //initialize or permanently kill RTC           
        else                         { timeOn = 1; if(debug_setup) Serial.println("RTC set"); }
      } 
      if(timeOn) { //display time
        time_t t = processSyncMessage();
        if (t != 0) { RTC.set(t); setTime(t); } // set the RTC and the system time to the received value
      }
    }
    
  }
  else { digitalWrite(MCUONLED, LOW); }

  //stuff for xbee initialization, so it only happens once
    if(xbeePrintInfo) {
      xbeePrintInfo = 0;
      if(timeOn) { //display time
        String genCurTimeMDYStr = genCurTimeMDY();
        String genCurTimeHMSStr = genCurTimeHMS();
        xbee.println(genCurTimeMDYStr+ " " + genCurTimeHMSStr);
      }
      if (debug_verbose) { xbee.print(F("Free Mem: ")); xbee.print(freeRam()); xbee.println(F("/2048")); } //memory
      if (battLevel < 255 && battLevel > -0.01) { xbee.print("Battery: "); xbee.print(battLevel,1); xbee.println("%"); } //battery
      else if (battLevel >= 255)                { xbee.println(F("Batt Sensor Error")); }
      else if (battLevel <= -0.01)              { xbee.println(F("Batt Low Err")); }
      xbee.println("");
    }

  //set title bar...mode titles cannot be longer than 14char
  /*if (laser_pwr > 0) { LCDsetPosition(1,13); myLCD.print("L"); }
   else               { LCDsetPosition(1,13); myLCD.print(" "); }*/
  //##laser power outside of mode3 has since been disabled in favor of XBee sleep control


  //GLOBAL BATTERY STAT
  if(debug_loop){ Serial.print("global batt "); Serial.println(battLevel); }
  LCDsetPosition(1,15); 
  if (battLevel < 100 && battLevel > -0.01) //if normal
  {
    myLCD.print("B"); //batt logo;
    if(battLevel < 10) myLCD.print("0"); //prefix zero for battLevel double digits
    myLCD.print(battLevel,1); //[,1]=.1 (include decimal)
    /*LCDsetPosition(1,20);*/ myLCD.print("%");
  }
  else if (battLevel >= 100 && 
           battLevel < 255)    { myLCD.print("B FULL"); }
  else if (battLevel >= 255)   { myLCD.print("PW_ERR"); digitalWrite(ERRORLED, HIGH); }
  else if (battLevel <= -0.01) { myLCD.print("PW_LOW"); digitalWrite(ERRORLED, HIGH); }
  else                         { myLCD.print("B_SNSR"); digitalWrite(ERRORLED, HIGH); }

  switch(mode)
    {
      case 1: mode1(); break;
      case 2: mode2(); break;
      case 0: mode0(); break;
      case 3: mode3(); break;
      case 4: mode4(); break;
      //default: mode0();
    }
 
  delay(globalDelay); //delay until next global event  
  //delay(EEPROM.read(globalDelayEEP));
} //END void loop()




void mode0() //off-recharge
{
  if(debug_mode0) Serial.println("InsideMode0");
  if (mode <= 0) //if (mode == 0 || mode == -1) //correction for always displayed
  {
    if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility   
      if (prevMode <=2 && prevMode >= 0) { mode = 3; initSet = 0; } //(prevMode != 3 || prevMode != 4) //if abnormal behavior, go back
      //if (prevMode == 0 || prevMode == 1 || prevMode == 2) { mode = 3; initSet = 0; } //(prevMode != 3 || prevMode != 4) //if abnormal behavior, go back
      else { prevMode = 0; } //else set to current mode
      
      if(powerSaveEnabled) digitalWrite(PWSAVE, LOW); //turn on power save mode //###experimental
      
      LCDclearScreen(); //delay(50);
      LCDsetPosition(1,1); myLCD.print("OFF/CHG"); if(debug_mode0) Serial.println("OFF/CHG");
      if(xbeePrintMode) {
        xbee.println("");
        xbee.println("Off / Charge Mode"); 
        xbeePrintMode = 0; 
        xbeePrintInfo = 1;
      } 
      //animStep = initAnimStep;


      //delay(50);
      analogWrite(BUTTONLED, 0); delay(250); interrupts(); 
      analogWrite(BUTTONLED, ledLuxLevel);
      //analogWrite(BUTTONLED, EEPROM.read(ledLuxLevelEEP));

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
    if (battLevel >= 254 || battLevel < 0) statusLight(1,0,1);

    //display time
    if (timeOn) {
      //time_t t = processSyncMessage();
      //if (t != 0) { RTC.set(t); setTime(t); } // set the RTC and the system time to the received value
      String genCurTimeMDYStr = genCurTimeMDY();
      String genCurTimeHMSStr = genCurTimeHMS();
      if(debug_loop) { Serial.print(genCurTimeMDYStr); Serial.print(" "); Serial.println(genCurTimeHMSStr); }
      //xbee.println(genCurTimeMDYStr+ " " + genCurTimeHMSStr);
      LCDsetPosition(2,1); myLCD.print(genCurTimeMDYStr + " " + genCurTimeHMSStr);
    }
    

    //3rd line Light
    unsigned int data0, data1; 
    if (tsl.getData(data0, data1)) { //if we have connectivity
      double currentLuxLevel;
      boolean good = tsl.getLux(tslGain,msInt,data0,data1,currentLuxLevel); //do we have a good (in-bounds) value [this line also generates lux value]
      if(good) { LCDsetPosition(3,1); myLCD.print("Lux: "); myLCD.print(currentLuxLevel); }
    }
    else if(debug_mode3) { printTSLError(tsl.getError()); }
    
    //3rd line Temperature
    if (altbar.readTempF() > -1765) {
      LCDsetPosition(3,15);
      myLCD.print(altbar.readTempF(), 1); //prints the temperature to LCD with one decimal place
      myLCD.write(0b11011111);
      myLCD.print("F");
    }

    //Uptime reporting
    LCDsetPosition(4,1); myLCD.print("Up: ");
    long millis1k = millis()/1000;
    if(millis1k >= 60) { myLCD.print((millis1k)/60,1); myLCD.print("m"); } //mins
    myLCD.print(((millis1k) % 60),1); myLCD.print("s"); //secs
    if(((millis1k) % 60) < 10) myLCD.print(" ");
    
    //internal temperature (v1)
    if(pcbVersion==1.0)      { LCDsetPosition(4,13); myLCD.print("I:"); myLCD.print(getInternalTemp('F'), 1); myLCD.write(0b11011111); myLCD.print("F"); }
    else if(pcbVersion==2.0) { LCDsetPosition(4,13); myLCD.print(analogRead(MIC), 1); myLCD.print("dB"); }
    
  }
  //else interrupt0(); //crash protection of some sort?
}


void mode1() //Accelerometer/Compass
{
  if(debug_mode1)Serial.println("InsideMode1");
  if (initSet != 1)
  {
    initSet = 1; //should be first because of interrupt possibility
    if(debug_mode1)Serial.println("InitSET To 1");
    if (prevMode <=3 && prevMode >= 1)
    //if (prevMode == 1 || prevMode == 2 || prevMode == 3) //(prevMode != 0 || prevMode != 4)
    {
      mode = 0; initSet = 0;
      if(debug_mode1)Serial.println("SwitchingTo0, Prev Was 1,2,3");
    }
    else {  ////if (prevMode == 0)
      LCDclearScreen(); delay(50);
      LCDsetPosition(4,1); myLCD.print("LOADING...");
      statusLight(0,1,1); lsm.heading(); statusLight(0,0,0); //This will generate an error if the sensor isn't present (auto stalls the MCU).
      LCDclearScreen();
      prevMode = 1; 
      LCDsetPosition(1,1); myLCD.print("Where Am I?"); if(debug_mode1)Serial.println("Where Am I?");
      if(xbeePrintMode) {
        xbee.println("Where Am I?"); 
        xbeePrintMode = 0; 
        //xbeePrintInfo = 1;
      }
      LCDsetPosition(2,1); myLCD.print("Heading:");
    /*LCDsetPosition(3,1);  myLCD.print("A X: ");
      LCDsetPosition(3,8);  myLCD.print("Y: ");
      LCDsetPosition(3,15); myLCD.print("Z: ");
      LCDsetPosition(4,1);  myLCD.print("M X: ");
      LCDsetPosition(4,8);  myLCD.print("Y: ");
      LCDsetPosition(4,15); myLCD.print("Z: ");*/
    } 
    analogWrite(BUTTONLED, 0);
    //delay(50);
    interrupts();
    analogWrite(BUTTONLED, ledLuxLevel);
    //analogWrite(BUTTONLED, EEPROM.read(ledLuxLevelEEP));
    buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms
  }

  //redraw protocol
  if(redrawCounter >= redrawMax)
  {
    LCDsetPosition(2,1); myLCD.print("Heading:");
  }
  
  float curHeading = lsm.heading();
  /*LCDsetPosition(2,1);
   myLCD.print("Heading: "); myLCD.print(curHeading);*/  //This would save space
  LCDsetPosition(2,1); myLCD.print("Heading: ");/*LCDsetPosition(2,9);*/ if(curHeading) {myLCD.print(curHeading); myLCD.write(0b11011111);}
  if(debug_mode1) { Serial.print("Heading: "); Serial.println(curHeading); }

  lsm.read();
  
  /*LCDsetPosition(3,1);
  String LineA = "AX:"; 
  LineA += (lsm.a.x)/10;
  LineA += " Y:";
  LineA += (lsm.a.y)/10;
  LineA += " Z:";
  LineA += (lsm.a.z)/10;
  myLCD.print(LineA);
  LCDsetPosition(4,1); 
  String LineM = "MX:";
  LineM += (lsm.m.x)/10;
  LineM += " Y:";
  LineM += (lsm.m.y)/10;
  LineM += " Z:";
  LineM += (lsm.m.z)/10;
  myLCD.print(LineM);*/
  
  
  LCDsetPosition(3,1);
  myLCD.print("AX:"); myLCD.print((lsm.a.x)/10);
  myLCD.print(" Y:");   myLCD.print((lsm.a.y)/10);
  myLCD.print(" Z:");   myLCD.print((lsm.a.z)/10);
  LCDsetPosition(4,1);  
  myLCD.print("MX:"); myLCD.print((lsm.m.x)/10);
  myLCD.print(" Y:");   myLCD.print((lsm.m.y)/10);
  myLCD.print(" Z:");   myLCD.print((lsm.m.z)/10);
}

void mode2() //Altitude
{
  if(debug_mode2) Serial.println("InsideMode2");
  if (initSet != 1)
  {
    initSet = 1; //should be first because of interrupt possibility
    //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0
    if (prevMode != 1) { mode = 1; initSet = 0; }
    else                 prevMode = 2;

    LCDclearScreen(); delay(50);
    //LCDsetPosition(4,1); myLCD.print("LOADING...");

    digitalWrite(BUTTONLED, LOW);
    //delay(50);
    interrupts();
    analogWrite(BUTTONLED, ledLuxLevel);
    buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms


    //LCDclearScreen(); delay(50);

    LCDsetPosition(1,1); myLCD.print("Atmospheric");
    if(xbeePrintMode) {
        xbee.println("Atmospheric"); 
        xbeePrintMode = 0; 
        //xbeePrintInfo = 1;
      } 
    LCDsetPosition(2,1); myLCD.print("Temp:");
    LCDsetPosition(3,1); myLCD.print("Pressure:");
    LCDclearLine(3,17);
    LCDsetPosition(4,1); myLCD.print("Altitude:");


    //interrupts();
    //delay(100);
  }

  //redraw protocol
  if(redrawCounter >= redrawMax)
  {
    LCDsetPosition(1,1); myLCD.print("Atmospheric");
    LCDsetPosition(2,1); myLCD.print("Temp:");
    LCDsetPosition(3,1); myLCD.print("Pressure:");
    LCDclearLine(4,20);
    LCDsetPosition(4,1); myLCD.print("Altitude:");
  }


  float curTempF = altbar.readTempF();
  float curTempC = altbar.readTemp();
  float curHumidRHT = calHumid.getTrueRH(curTempC); //calculate true %RH based from Temperature
  //float curHumidRH = calHumid.getSensorRH(); //calculate relative %RH

  if (curTempF > -200)
  {
    LCDsetPosition(2,7);
    myLCD.print(curTempF, 1); //prints the temperature to LCD with one decimal place
    myLCD.write(0b11011111); myLCD.print("F |  H");
    //humidity
    if((int)curHumidRHT < 10) { myLCD.print("0"); }
    myLCD.print(curHumidRHT, 1); //prints the humidity with one decimal place
    myLCD.write(0b00100101); //'%'
  }
  else { myLCD.print("Temp Error"); }

  if(debug_mode2) { Serial.print(curTempF); Serial.println("F | ");
                    Serial.print(curHumidRHT); Serial.println("% RH"); }

  LCDsetPosition(3,11);
  altbar.setModeBarometer();
  float curBarPres = altbar.readPressure();
  if (curBarPres < 115000) { myLCD.print(curBarPres, 0); myLCD.print(" Pa  "); }
  else                     { myLCD.print(">1k kPa"); }
  if(debug_mode2) { Serial.print(curBarPres); Serial.println(" Pa"); }



  LCDsetPosition(4,11);
  altbar.setModeAltimeter();
  float curBarAlt = altbar.readAltitudeFt();
  if (curBarAlt < 12000 && curBarAlt > -2000) { myLCD.print((curBarAlt), 1); myLCD.print(" Ft."); } //(" Feet ");
  else                                        { myLCD.print(">10k Ft"); }


  //Serial.println(curTempF);
  //Serial.println(curBarAlt);
}


void mode3() //
{
  if(debug_mode3)Serial.println(F("InsideMode3"));
  if (initSet != 1) {
    initSet = 1; //should be first because of interrupt possibility
    if (prevMode != 2) { mode = 2; initSet = 0; }
    else               { prevMode = 3;  }
    
    LCDclearScreen(); delay(50); 
    //LCDsetPosition(4,1); myLCD.print("LOADING...");

    //EEPROM.write(1,2);


    digitalWrite(BUTTONLED, LOW);
    //delay(50);
    interrupts();
    analogWrite(BUTTONLED, ledLuxLevel);
    buzz(BUZZER,400,200);  //buzz on pin 9 at 400hz for 200ms


    //LCDclearScreen(); delay(50);


    LCDsetPosition(1,1);  myLCD.print(F("Light Sensors"));
    if(xbeePrintMode) {
        xbee.println("Light Sensors"); 
        xbeePrintMode = 0; 
        //xbeePrintInfo = 1;
      } 
    LCDsetPosition(2,1);  myLCD.print(F("Lux:"));
    LCDsetPosition(3,1);  myLCD.print(F("Vis:"));
    LCDsetPosition(3,15); myLCD.print(F("Laser:"));
    /*LCDsetPosition(4,1); */ myLCD.print(F(" IR:"));
    
    //interrupts();
    //delay(100);
  }
  
  
    //redraw protocol
    if(redrawCounter == redrawMax)
    {
      //LCDsetPosition(2,1); myLCD.print("Lux:");
      LCDsetPosition(3,1); myLCD.print(F("Vis:"));
      LCDsetPosition(3,15); myLCD.print(F("Laser: IR:")); //concatenated 
    }
  
  
  double currentLuxLevel;
  static uint8_t tslSwitchCounter;
  unsigned int data0, data1;
  
  
  if (tsl.getData(data0,data1)) { //if we have connectivity
    if (tslSwitchCounter > switchCount) //if we need to switch back from integration
    {
      msInt=402; tslTime=2; //0 = 13.7ms, 1 = 101ms, 2 = 402ms, 3 = manual, make sure to set both vars
      tsl.setTiming(tslGain,tslTime,msInt); delay(50);
      LCDclearLine(2);
      tslSwitchCounter = 0;
    }
    else if (tslSwitchCounter > 0) { //if we are still switched
      LCDsetPosition(2,14); myLCD.print(" LOW ");
      if (switchCount-tslSwitchCounter+1 < 10) { myLCD.print(" "); }
      myLCD.print(switchCount-tslSwitchCounter+1); 
      tslSwitchCounter++; 
    }
    
    //LCDsetPosition(2,1); myLCD.print("Lux: ");
    
    boolean good = tsl.getLux(tslGain,msInt,data0,data1,currentLuxLevel); //do we have a good (in-bounds) value [this line also generates lux value]
    if (data0 == 65535 || data1 == 65535) //if we need to switch integration because of upper value
    {
      //if (tslSwitchCounter <= 1) {
        msInt=101; tslTime=1; //0 = 13.7ms, 1 = 101ms, 2 = 402ms, 3 = manual
        tsl.setTiming(tslGain,tslTime,msInt);
        if(debug_verbose) xbee.println(F("Sw Integration"));
        LCDsetPosition(2,5); myLCD.print(F("Sw Integration")); 
      //}
      delay(1250);
      LCDclearLine(2,5); delay(50);
      //LCDsetPosition(2,1); myLCD.print("Lux: "); //moved below
      tslSwitchCounter = 1;
    }
    if (good) {
      LCDsetPosition(2,6); myLCD.print(currentLuxLevel); 
        if( tslSwitchCounter > 0 && currentLuxLevel > 1000) {myLCD.print("  ");} else myLCD.print("   ");
      LCDsetPosition(3,8); myLCD.print("   ");
      LCDsetPosition(3,6); myLCD.print(data0);
      LCDsetPosition(4,8); myLCD.print("   ");
      LCDsetPosition(4,6); myLCD.print(data1);
    }
    else if (tslSwitchCounter != 1) { LCDsetPosition(2,5); myLCD.print(F("Bad Value")); } //bug fix for persistance of the string on screen
    
    LCDsetPosition(2,1); myLCD.print("Lux: ");
  }
  else { statusLight(1,0,1); printTSLError(tsl.getError()); delay(10000);}
  
  
  
  //###recheck with hardware to find optimal "brightness zone"
  //shift 0-400lux to 50-255brightness
  //if (currentLuxLevel < 400) analogWrite(BUTTONLED, map(currentLuxLevel, 0, 400, 50, 255)); //set the brightness of the LED to ~room brightness
  //else                       analogWrite(BUTTONLED, map(currentLuxLevel, 0, 2000, 50, 255)); //set the brightness of the LED to ~room brightness
  
  if (currentLuxLevel < 1) analogWrite(BUTTONLED, map_double(currentLuxLevel, 0, 1, 20, 255)); //set the brightness of the LED to ~room brightness
  else if (currentLuxLevel < 400) analogWrite(BUTTONLED, map(currentLuxLevel, 0, 400, 20, 255)); //set the brightness of the LED to ~room brightness
  //else if (currentLuxLevel < 2000) analogWrite(BUTTONLED, map(currentLuxLevel, 0, 2000, 50, 255)); //set the brightness of the LED to ~room brightness
  else analogWrite(BUTTONLED, ledLuxLevel); //set the brightness of the LED to ~room brightness
  
  
  static uint8_t xbeeLEDCount = 0;
  //Blink XBee Button LED
  if (xbeeLEDCount > 3) { //4 beats or greater
    if (xbeeButtonLED) digitalWrite(XBEELED, LOW);        //turn off XBee Button LED
    else               analogWrite(XBEELED, ledLuxLevel); //turn on XBBL to light level
    xbeeLEDCount = 0;
    xbeeButtonLED = !xbeeButtonLED;
  }
  else xbeeLEDCount++;
  
  LCDsetPosition(4,14); 
  if(laser_pwr > 8) { 
    if (laser_pwr < 100) myLCD.print(" "); 
      if (laser_pwr < 10) myLCD.print(" ");
    myLCD.print(laser_pwr); myLCD.print(F("/255")); 
  }
  else if (laser_pwr == 0) { myLCD.print("    OFF"); }
  else if (laser_pwr == 5) { myLCD.print(F("BREATHE")); }
  else if (laser_pwr == 4) { myLCD.print(F(" STROBE")); }
  //else if (laser_pwr == 6) { myLCD.print(F("   FADE")); }
  //else myLCD.print(F("  ERROR"));

  //Laser strobe
  if (laser_pwr == 4)      { digitalWrite(LASEREN, LOW); /*digitalWrite(STAT2, HIGH);*/ digitalWrite(ONETHREE, HIGH); laser_pwr = 3; } //enable laser
  else if (laser_pwr == 3) { digitalWrite(LASEREN, HIGH); /*digitalWrite(STAT2, LOW);*/ digitalWrite(ONETHREE, LOW); laser_pwr = 4; } //disable laser
  
  if (laser_pwr == 5) { laserBreathe(); }
  
  /*if (laser_pwr == 6) {
    for (int16_t val = 255; val > 0; val-=strobeKnockVal)  {analogWrite(LASEREN, val); delay(strobeStopVal);} delay(200); //brighten laser
    for (int16_t val = 0; val <= 255; val+=strobeKnockVal) {analogWrite(LASEREN, val); delay(strobeStopVal);} delay(100); //dim laser
  }*/
  
  
  /*if (laser_pwr == 4) {
    if (laserStrobeDir) { for (int val = 255; val > 0; val-=strobeKnockVal)  {analogWrite(LASEREN, val); delay(strobeStopVal);} delay(200); }//brighten laser
    else                { for (int val = 0; val <= 255; val+=strobeKnockVal) {analogWrite(LASEREN, val); delay(strobeStopVal);} delay(100); } //dim laser
    laserStrobeDir = !laserStrobeDir;
  }*/
//for reference, in the two 'for' statements above, 'val' must be of an 16-bit signed int type (rather than 8-bit)
//because the Knock value will prevent the condition from ever becoming true (the value just wraps back and locks into an infinite loop)


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
    
    static boolean curWarnScreen = true; //T = 1; F = 2
    
    if(debug_mode4) Serial.println("WarnScreen1");
    interrupts();
    while(warningShown == 1)
    {
      LCDsetPosition(1,1); 
      if (curWarnScreen) { myLCD.print(F("WARNING: Do not use this robot for scien-tific purposes, as Press to continue...")); }
      else               { myLCD.print(F("it may sometimes    yield inaccurate or imprecise data.     ")); }
      
      if (millis() - previousWarnMillis > 2500)
      {
        if(debug_mode4) Serial.println("WarnScreenChange");
        curWarnScreen = !(curWarnScreen);
        previousWarnMillis = millis();
      }
    }
    
  }
}


void interrupt0()
{
  //debounce protection
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (millis() - last_interrupt_time > 200) 
  {
    noInterrupts();
    if(mode != -1) { buzz(BUZZER,500,200); } //buzz on pin 9 at 500hz for 200ms //INTERRUPT INDICATOR
    Serial.println("InsideInterrupt");
    digitalWrite(MCUONLED, LOW); 
    statusLight(0,0,1); //(s1)(s2)[err]
    //digitalWrite(BUTTONLED, LOW);
  
  switch(warningShown)
  {
    case -1: mode = 4; if(debug_int0)Serial.println("switchToWarn"); break; //send to warning screen when loop() will call mode4();
    case 0:  mode = 4; if(debug_int0)Serial.println("switchToWarn"); break; //send to warning screen when loop() will call mode4();
    case 1:  mode = 1; warningShown = 2; prevMode = 0; initSet = 0; if(debug_int0)Serial.println("switchWarnShown1"); break; //prevmode = 4; ##prevMode would be 0, since "mode 4" isn't really a mode 
    case 2:
      switch(mode)
      {
        case 1: { mode = 2; if(debug_int0)Serial.println("switchto2"); break;}
        case 0: { mode = 1; if(debug_int0)Serial.println("switchto1"); break;}
        case 2: { mode = 3; if(debug_int0)Serial.println("switchto3"); break;}
        case 3: { mode = 0; if(debug_int0)Serial.println("switchto0"); break;}
        default: mode = 0; if(debug_int0)Serial.println("switchfrom_elseto0");
      }
      break;
      default: statusLight(0,0,1); if(debug_int0) {Serial.println("warning err"); xbee.println("err: true-ing out"); while(1);}
  }
    
    initSet = 0;
    xbeePrintMode = 1;
    //digitalWrite(BUTTONLED, HIGH);
    //Serial.print("Interrupt Set Mode: "); //  Serial.println(mode);
    //analogWrite(BUTTONLED, ledLuxLevel); 
    digitalWrite(MCUONLED, HIGH); digitalWrite(ERRORLED, LOW);

  }
/*else //THIS CANNOT EXIST UNLESS ACTUALLY PLUGGED INTO USB, CAUSES BUGS OTHERWISE
   {
   if(debug_int0)Serial.println("buttonPressRegistered");
   }
   if(debug_int0)Serial.println(last_interrupt_time);*/
  last_interrupt_time = millis();
}


void interrupt1() {
  //debounce protection
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (millis() - last_interrupt_time > 200) 
  {
    noInterrupts();
    if(mode != -1) { buzz(BUZZER,575,200); } //buzz on pin 9 at 575hz for 200ms //INTERRUPT INDICATOR

    digitalWrite(STAT1, HIGH); digitalWrite(ERRORLED, HIGH); digitalWrite(MCUONLED, LOW); 
    //statusLightIntOne(1,1,1);
    //analogWrite(XBEELED, ledLuxLevel); //XBee Button LED
    //analogWrite(BUTTONLED, ledLuxLevel); OR //digitalWrite(BUTTONLED, LOW); //why did we have these...leftovers from CB?

    if (mode == 3) {
      //uint8_t setLaserPower; //for eeprom?
      
      switch(laser_pwr)
      {
        case 0:  digitalWrite(LASEREN, LOW);     laser_pwr = 255; break; //enable laser, now at full brightness
        case 255: analogWrite(LASEREN, 255-127); laser_pwr = 127; break; //now at 127 brightness
        case 127: analogWrite(LASEREN, 255-45);  laser_pwr = 45;  break; //now at 45 brightness
        case 45: digitalWrite(LASEREN, LOW);     laser_pwr = 4;   break; //now strobing
        case 5:  digitalWrite(LASEREN, HIGH);    laser_pwr = 0;          //now off  ////now fading [WAIT REQUIRED -> FOR]
                 LCDsetPosition(4,14); myLCD.print(F("   WAIT")); break;
        //case 6:  digitalWrite(LASEREN, HIGH);    laser_pwr = 0;          //turn off laser, now off [WAIT REQUIRED -> FOR]
                 //LCDsetPosition(4,14); myLCD.print(F("   WAIT")); break; 
        case 4:  digitalWrite(LASEREN, HIGH);    laser_pwr = 5;   break; //now breathing
        case 3:  digitalWrite(LASEREN, HIGH);    laser_pwr = 5;   break; //now breathing 
               //case 4 and 3 cannot be combined with an || statement; it won't work.
        default:
          digitalWrite(LASEREN, HIGH); /*digitalWrite(STAT2, LOW);*/ digitalWrite(ONETHREE, LOW); 
          laser_pwr = 0; //disable laser due to var error //THIS OCCURS AFTER BREATHE ALWAYS
          if(debug_verbose) xbee.println(F("Laser Var Error"));
      }
      
      if (laser_pwr > 5) { /*digitalWrite(STAT2, HIGH);*/ digitalWrite(ONETHREE, HIGH); } //effectively, if laser is on (but we don't deal with strobing here) 
      
      if(pcbVersion==2.0){if(laser_pwr>0){digitalWrite(STAT2, HIGH); } else digitalWrite(STAT2, LOW);} //effectively, if laser is on (but we don't deal with strobing here) 
    }

    else { //mode != 3
      /////#####Insert XBee Sleep stuff here
      //maybe something about triggering and switching LED status
      /*if (xbeeSleep) { digitalWrite(XBEESLEEPD, LOW); }  //stop XBee sleep
      else           { digitalWrite(XBEESLEEPD, HIGH); } //start XBee sleep
      xbeeSleep = !xbeeSleep;*/
      digitalWrite(XBEESLEEPD, !xbeeSleep); //toggle XBee Sleep
      xbeeSleep = !xbeeSleep;
      
    }
    //analogWrite(BUTTONLED, ledLuxLevel);
    digitalWrite(STAT1, LOW); digitalWrite(ERRORLED, LOW); digitalWrite(MCUONLED, HIGH); 
    //statusLightIntOne(0,0,0);
  }
  last_interrupt_time = millis();
}


void buzz(uint8_t targetPin, uint16_t frequency, uint16_t length) {
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
  if(minute() < 10) {genTimeHMS+="0";}
  genTimeHMS += minute();
  genTimeHMS += ":";
  if(second() < 10) {genTimeHMS+="0";}
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
  if( ( (hourFormat12() > 9) && (month() > 9 || day() > 9) ) ||
        (month() > 9 && day() > 9) )
    {genTimeMDY += (year()-2000);} //for LCD spacing issues
  else {genTimeMDY += (year());}
  return genTimeMDY;
}

void altbarStarter() {
  altbar.setModeAltimeter(); // Measure altitude above sea level in meters
  altbar.setOversampleRate(7); // Set Oversample to the recommended 128
  altbar.enableEventFlags(); // Enable all three pressure and temp event flags
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
int memoryFree() {
 int freeValue;
 if ((int)__brkval == 0) freeValue = ((int)&freeValue) - ((int)&__bss_end);
 else                    freeValue = ((int)&freeValue) - ((int)__brkval);
 
 return freeValue;
}
 

//RTC TIME SET CODE
//code to process time sync messages from the serial port
#define TIME_HEADER  "T"   //Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; //Jan 1 2013 
  if(Serial.find(TIME_HEADER))
  {
    pctime = Serial.parseInt(); return pctime;
    if( pctime < DEFAULT_TIME) { pctime = 0L; } //check the value is a valid time (greater than Jan 1 2013)
  }                                             //return 0 to indicate that the time is not valid
  return pctime;
}


void printTSLError(byte error) // If there's an TSL I2C error, this function will print out an explanation.
{
  if(debug_mode3 || debug_mode0) { 
    xbee.print("I2C error: "); xbee.print(error,DEC); xbee.print(", ");
    switch(error)
    {
      case 0: xbee.println("success"); break;
      case 1: xbee.println("data too long for transmit buffer"); break;
      case 2: xbee.println("received NACK on address (disconnected?)"); break;
      case 3: xbee.println("received NACK on data"); break;
      case 4: xbee.println("other error"); break;
      default: xbee.println("unknown error");
    }
  }
}


void printGlobals() {
  if(debug_setup) Serial.println("Global Variables:");
  if(debug_setup) Serial.print("Global Delay: ");     if(debug_setup) Serial.print(globalDelay);
  if(debug_setup) Serial.print(" | LED Lux Level: "); if(debug_setup) Serial.print(ledLuxLevel);
  if(debug_setup) Serial.print(" | Switch Count: ");  if(debug_setup) Serial.print(switchCount);
  if(debug_setup) Serial.print(" | Power Save: ");    if(debug_setup) Serial.println(powerSaveEnabled);
}

//const uint8_t pulseValue[] PROGMEM = 
const uint8_t pulseValue[] = {1, 1, 2, 3, 5, 8, 11, 15, 20, 25, 30, 36, 43, 49, 56, 64, 72, 80, 88, 
                          97, 105, 114, 123, 132, 141, 150, 158, 167, 175, 183, 191, 199, 206, 212, 
                          219, 225, 230, 235, 240, 244, 247, 250, 252, 253, 254, 255, 254, 253, 
                          252, 250, 247, 244, 240, 235, 230, 225, 219, 212, 206, 199, 191, 183, 175, 
                          167, 158, 150, 141, 132, 123, 114, 105, 97, 88, 80, 72, 64, 56, 49, 43, 
                          36, 30, 25, 20, 15, 11, 8, 5, 3, 2, 1, 0 };
void laserBreathe()
{
  for(uint8_t i=0; i < sizeof(pulseValue); i++) { analogWrite(LASEREN, abs(255-pulseValue[i])); delay(30); }
  //for(uint8_t i=0; i < sizeof(pulseValue); i++) { analogWrite(LASEREN, abs(255-(pgm_read_word_near(pulseValue + i)))); delay(30); }
}


double getInternalTemp(void)
{
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3)); // Set the internal reference and mux.
  ADCSRA |= _BV(ADEN); delay(20);  // Enable the ADC
  ADCSRA |= _BV(ADSC);             // Start the ADC
  while (bit_is_set(ADCSRA,ADSC)); // Detect end-of-conversion
  unsigned int iADC = ADCW;
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  // Need to recalibrate 324.31...
  return ((double)(iADC - 324.31 ) / 1.22); //celsius
}

double getInternalTemp(char format) {
  if (format == 'F' || format == 'f') { return (((getInternalTemp() * 9.0)/ 5.0) + 32.0); }
  else if (format == 'C' || format == 'c'){ return getInternalTemp(); }
  else return getInternalTemp();
}

//double getInternalTempF(void) { return (((getInternalTemp() * 9.0)/ 5.0) + 32.0); } //[DEPRECATED]


double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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


