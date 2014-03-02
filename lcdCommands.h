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
void LCDclearDisplay() { LCDclearScreen(); } //deprecated: please use LCDclearScreen()
//-------------------------------------------------------------------------------------------
void LCDsetPosition(int line, int pos) //set position at line, position (1-4,1-20);
{
  if (pos > 0 && pos <= 20) {pos--;}   //if pos > than min (1) change to index, 1 --> 0, etc. (human --> index)
  else {pos=0;} //else reset to 0

  if (line > 4 || line < 1) {line=1;} //if lines outside max reset to 0
//else if (line == 1); //normal index
  else if (line == 2) {pos += 64;}
  else if (line == 3) {pos += 20;}
  else if (line == 4) {pos += 84;}
  
  pos += 128;
  myLCD.write(0xFE); myLCD.write(pos); //command flag + write position
}
//-------------------------------------------------------------------------------------------
void LCDclearLine(int line, int pos) //write whitespace starting at position until end of line
{
  LCDsetPosition(line,pos); 
  for(int count = pos; count <= 20; count++) { myLCD.print(" "); } //write whitespace until end of line
}
//-------------------------------------------------------------------------------------------
void LCDclearLine(int line) { LCDclearLine(line, 1); } //write whitespace until end of line
//-------------------------------------------------------------------------------------------
void LCDturnDisplayOn() //this turns the display back ON
{
  myLCD.write(0xFE); myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDbrightness(int brightness) { //this function takes an int between 0-255 and turns the backlight on accordingly
// 0 = OFF, 255 = Fully ON, everything in-between = varied brightness [out of possible 30 different brightness levels]
  myLCD.write(0x7C); myLCD.write(map(brightness, 0, 255, 128, 157)); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec 
}
//-------------------------------------------------------------------------------------------
void LCDbacklight(int lvl) //this function takes an int between 128-157 and turns the backlight on accordingly
{
  myLCD.write(0x7C); myLCD.write(lvl);  // any value between 128 and 157 or 0x80 and 0x9D
}
//-------------------------------------------------------------------------------------------
void ChangeSerialLCDSplashScreen(String line1, String line2) { //change the splash screen of the LCD panel
/* The two parameters are the 2nd and 3rd line of the display, and
 * must be 20char strings, complete with whitespace offset if needed.
 * NOTE: This should be run in it's own sketch, and you'll only need to run it once.
 * This isn't meant to be run on every boot, as this changes the ROM memory in the LCD itself.
 */
  myLCD.write(0x7C); myLCD.write(0x03); delay(100);
  myLCD.write(0x7C); myLCD.write(0x05); delay(100);
  
  LCDsetPosition(2,1); myLCD.print(line2); //2nd line
  LCDsetPosition(1,1); myLCD.print(line1); //then first line
  
  delay(200); myLCD.write(0x7C); myLCD.write(0x0A);
  while(1);
}
//-------------------------------------------------------------------------------------------
void LCDtoggleSplash() //this toggles the splash screen [if off send this to turn on, if on send this to turn off]
{
  myLCD.write(0x7C); //command flag = 124 dec
  myLCD.write(9); // 0x09
}
//-------------------------------------------------------------------------------------------
/*void LCDclearScreenFull() //[DEPRECATED]
{
  LCDsetPosition(1,1); myLCD.print("                    ");
  LCDsetPosition(2,1); myLCD.print("                    ");
  LCDsetPosition(3,1); myLCD.print("                    ");
  LCDsetPosition(4,1); myLCD.print("                    ");
}
//-------------------------------------------------------------------------------------------
void LCDturnDisplayOff()
{
  //this tunrs the display off, but leaves the backlight on. 
  myLCD.write(0xFE); //command flag
  myLCD.write(8); // 0x08
}
//-------------------------------------------------------------------------------------------
void LCDunderlineCursorOn() //turns the underline cursor on
{
  myLCD.write(0xFE); //command flag
  myLCD.write(14); // 0x0E
}
//-------------------------------------------------------------------------------------------
void LCDunderlineCursorOff() //turns the underline cursor off
{
  myLCD.write(0xFE); //command flag
  myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDboxCursorOn() //this turns the box cursor on
{
  myLCD.write(0xFE); //command flag
  myLCD.write(13); // 0x0D
}
//-------------------------------------------------------------------------------------------
void LCDboxCursorOff() //this turns the box cursor off
{
  myLCD.write(0xFE); //command flag
  myLCD.write(12); // 0x0C
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
  //this function shows the different cursors avaiable
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
