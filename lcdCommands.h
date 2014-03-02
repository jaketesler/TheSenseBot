/***********************************************************************
SerLCD Example Code
SparkFun Electronics 
Joel Bartlett
December 20, 2012 

Modified by Jake Tesler for the COM-BAT System Project, 2012-2013
Modified by Jake Tesler for the SenseBot Project, 2014-

This code uses the information presented in the SerLCD Datasheet 
to create an Arduino example using the SerLCD from SparkFun Electonics. 
Each of the SerLCD's capabilities is broken up into seperate functions
within the sketch. Simply call each function with the correct parameters 
to get the desired result form the LCD screen. 

This code was devoloped for the Arduino IDE v102

To use, connect the following pins
VDD -> 5V
GND -> GND
LCD RX -> Arduino TX (pin 4)

***Don't forgect to disconnect the LCD's RX pin from the TX pin of 
the Arduino's UART line while programming!***

"THE BEER-WARE LICENSE"
As long as you retain this notice you can do whatever you want with this stuff. 
If we meet some day, and you think this stuff is worth it, you can buy me a beer.

************************************************************************/

#define PCBVERSION1.0

#ifdef PCBVERSION1.0
#define LCDPIN 4

#elif defined PCBVERSION2.0
#define LCDPIN A2
#endif

SendOnlySoftwareSerial myLCD(LCDPIN); //5 RX, 4 TX* ##CUSTOM

//-------------------------------------------------------------------------------------------
void LCDclearScreen()
{
  myLCD.write(0xFE);
  myLCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
/*void LCDclearDisplay()
{
  //clears the screen, you will use this a lot!
  myLCD.write(0xFE);
  myLCD.write(0x01); 
}*/
//-------------------------------------------------------------------------------------------
//NEW CUSTOM PROGRAM/UTILITY
void LCDsetPosition(int line, int pos) //set position at line, position (1-4,1-20);
{ 
/*while (pos > 20) //if pos is greater than 20 (max), subtract 1 lines worth of chars and carriage return
  {
    pos -= 20; line++;
    if (line > 4) { line = 1; } //if line > 4 (max), reset lines to 0;
  }*/
  if (pos > 0 && pos <= 20) {pos--;}   //if pos > than min (1) change to index, 1 --> 0, etc. (human --> index)
  else {pos=0;} //else reset to 0
  
  //modify based on lines
  if (line > 4 || line < 1) {line=1;} //if lines outside max reset to 0
//else if (line == 1) //normal index
  else if (line == 2) {pos += 64;}
  else if (line == 3) {pos += 20;}
  else if (line == 4) {pos += 84;}
  
  pos += 128;
  myLCD.write(0xFE); //command flag
  myLCD.write(pos); //write position
}
//-------------------------------------------------------------------------------------------
void LCDclearLine(int line, int pos)
{
  LCDsetPosition(line,pos); 
  for(int count = pos; count < 21; count++){ myLCD.print(" "); }
}

void LCDclearLine(int line)
{
  LCDclearLine(line, 1);
  //LCDsetPosition(line,1); myLCD.print("                    ");
}
//-------------------------------------------------------------------------------------------
void LCDturnDisplayOn()
{
  //this turns the dispaly back ON
  myLCD.write(0xFE); //command flag
  myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDbrightness(int brightness)// 0 = OFF, 255 = Fully ON, everything inbetween = varied brightnbess 
{
  int bright=map(brightness, 0, 255, 128, 157);
  //this function takes an int between 0-255 and turns the backlight on accordingly
  myLCD.write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
  myLCD.write(bright);
}
//-------------------------------------------------------------------------------------------
//NEW CUSTOM PROGRAM/UTILITY (that we no longer use)
/*void LCDclearScreenFull()
{
  LCDsetPosition(1,1); myLCD.print("                    ");
  LCDsetPosition(2,1); myLCD.print("                    ");
  LCDsetPosition(3,1); myLCD.print("                    ");
  LCDsetPosition(4,1); myLCD.print("                    ");
}
//-------------------------------------------------------------------------------------------
void LCDselectLineOne()
{ 
  //puts the cursor at line 0 char 0.
  myLCD.write(0xFE); //command flag
  myLCD.write(128); //position
}
//-------------------------------------------------------------------------------------------
void LCDselectLineTwo()
{ 
  //puts the cursor at line 0 char 0.
  myLCD.write(0xFE); //command flag
  myLCD.write(192); //position
}
//-------------------------------------------------------------------------------------------
void LCDmoveCursorRightOne()
{
  //moves the cursor right one space
  myLCD.write(0xFE); //command flag
  myLCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void LCDmoveCursorLeftOne()
{
  //moves the cursor left one space
  myLCD.write(0xFE); //command flag
  myLCD.write(16); // 0x10
}
//-------------------------------------------------------------------------------------------
void LCDscrollRight()
{
  //same as moveCursorRightOne
  myLCD.write(0xFE); //command flag
  myLCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void LCDscrollLeft()
{
  //same as moveCursorLeftOne
  myLCD.write(0xFE); //command flag
  myLCD.write(24); // 0x18
}
//-------------------------------------------------------------------------------------------
void LCDturnDisplayOff()
{
  //this tunrs the display off, but leaves the backlight on. 
  myLCD.write(0xFE); //command flag
  myLCD.write(8); // 0x08
}
//-------------------------------------------------------------------------------------------
void LCDunderlineCursorOn()
{
  //turns the underline cursor on
  myLCD.write(0xFE); //command flag
  myLCD.write(14); // 0x0E
}
//-------------------------------------------------------------------------------------------
void LCDunderlineCursorOff()
{
  //turns the underline cursor off
  myLCD.write(0xFE); //command flag
  myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDboxCursorOn()
{
  //this turns the box cursor on
  myLCD.write(0xFE); //command flag
  myLCD.write(13); // 0x0D
}
//-------------------------------------------------------------------------------------------
void LCDboxCursorOff()
{
  //this turns the box cursor off
  myLCD.write(0xFE); //command flag
  myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDtoggleSplash()
{
  //this toggles the splash screenif off send this to turn onif on send this to turn off
  myLCD.write(0x7C); //command flag = 124 dec
  myLCD.write(9); // 0x09
}
//-------------------------------------------------------------------------------------------
void LCDbacklight(int brightness)// 128 = OFF, 157 = Fully ON, everything inbetween = varied brightnbess 
{
  //this function takes an int between 128-157 and turns the backlight on accordingly
  myLCD.write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
  myLCD.write(brightness); // any value between 128 and 157 or 0x80 and 0x9D
}
//-------------------------------------------------------------------------------------------
void LCDscrollingMarquee()
{
//This function scroll text across the screen on both lines
  LCDclearScreen(); // it's always good to clear the screen before movonh onto a new print
  for(int j = 0; j < 17; j++)
  {
    LCDselectLineOne();
    for(int i = 0; i < j;i++)
      LCDmoveCursorRightOne();
    myLCD.print("SPARK");
    LCDselectLineTwo();
    for(int i = 0; i < j;i++)
      LCDmoveCursorRightOne();
    myLCD.print(" FUN");
    delay(500); // you must have a delay, otherwise the screen will print and clear before you can see the text
    LCDclearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void LCDcounter()
{
  //this function prints a simple counter that counts to 10
  LCDclearScreen();
  for(int i = 0; i <= 10; i++)
  {
    myLCD.print("Counter = ");
    myLCD.print(i, DEC);
    delay(500);
    LCDclearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void LCDtempAndHumidity()
{
  //this function shows how you could read the data from a temerature and humidity 
  //sensro and then print that data to the SermyLCD.
  
  //these could be varaibles instead of static numbers 
  float tempF = 77.0; 
  float tempC = 25.0;
  float humidity = 67.0;
  
  LCDclearScreen();
  LCDselectLineOne();
  myLCD.print(" Temp = ");
  myLCD.print((long)tempF, DEC);
  myLCD.print("F ");
  myLCD.print((long)tempC, DEC);
  myLCD.print("C");
  LCDselectLineTwo();
  myLCD.print(" Humidity = ");
  myLCD.print((long)humidity, DEC); 
  myLCD.print("%");
  delay(2500);
}
//-------------------------------------------------------------------------------------------
void LCDbacklight()
{
  //this function shows the different brightnesses to which the backlight can be set 
  LCDclearScreen();
  for(int i = 128; i < 158; i+=2)// 128-157 are the levels from off to full brightness
  {
    LCDbacklight(i);
    delay(100);
    myLCD.print("Backlight = ");
    myLCD.print(i, DEC);
    delay(500);
    LCDclearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void LCDcursors()
{
  //this function shows the different cursors avaiable on the SermyLCD
  LCDclearScreen();
  
  LCDboxCursorOn();
  myLCD.print("Box On");
  delay(1500);
  LCDclearScreen();
  
  LCDboxCursorOff();
  myLCD.print("Box Off");
  delay(1000);
  LCDclearScreen();
  
  LCDunderlineCursorOn();
  myLCD.print("Underline On");
  delay(1500);
  LCDclearScreen();
  
  LCDunderlineCursorOff();
  myLCD.print("Underline Off");
  delay(1000);
  LCDclearScreen();
}
*/
