/***********************************************************************
This source code cites "SerLCD Example Code", Joel Bartlett, SparkFun Electronics. 

Created by Jake Tesler for the COM-BAT System Project, 2012-2013
Modified by Jake Tesler for the SenseBot Project, 2014-

"THE BEER-WARE LICENSE"
As long as you retain this notice you can do whatever you want with this stuff. 
If we meet some day, and you think this stuff is worth it, you can buy me a beer.

************************************************************************/

#define PCBVERSION2.0
//DO NOT CHANGE BELOW THIS LINE
#ifdef PCBVERSION1.0
#define LCDPIN 4
#endif

#ifdef PCBVERSION2.0
#define LCDPIN A2
#endif

SendOnlySoftwareSerial myLCD(LCDPIN); //4(A2) TX* ##CUSTOM

//-------------------------------------------------------------------------------------------
void LCDclearScreen()
{
  myLCD.write(0xFE);
  myLCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
void LCDclearDisplay() { LCDclearScreen(); } //deprecated: use LCDclearScreen()
//-------------------------------------------------------------------------------------------
void LCDsetPosition(uint8_t line, uint8_t pos) //set position at line, position (1-4,1-20);
{
  //if (pos > 0 && pos <= 20) {pos--;}   //if pos > than min (1) change to index, 1 --> 0, etc. (human --> index)
  //else {pos=0;} //else reset to 0
  //if (pos > 20 || pos <= 0) {pos=0;}
  if (pos > 20) {pos=0;}
  
//if      (line == 1); //normal index
  if      (line == 2) {pos += 64;}
  //else pos = pos + (line>=3) ? 20 : 84;
  else if (line == 3) {pos += 20;}
  else if (line == 4) {pos += 84;}
  else if (line > 4 || line < 1) {line=1;}

//else {line=1;}
  

  //pos += 127; //should be 128, but we removed the human-computer index and subtracted here instead.
  myLCD.write(0xFE); myLCD.write(pos+127); //command flag + write position
  
  /*if      (line == 2) pos += 64;
  else if (line==1);
  else pos = pos + (line>=3) ? 20 : 84;*/ 
}
//-------------------------------------------------------------------------------------------
void LCDsetPosition(uint8_t line) { LCDsetPosition(line, 1); } //set position at line, position (1-4,1);
//-------------------------------------------------------------------------------------------
void LCDclearLine(uint8_t line, uint8_t pos) //write whitespace starting at position until end of line
{
  LCDsetPosition(line,pos); 
  for(uint8_t count = pos; count < 21; count++) { myLCD.print(" "); } //write whitespace until end of line
}
//-------------------------------------------------------------------------------------------
void LCDclearLine(uint8_t line) { LCDclearLine(line, 1); } //write whitespace until end of line
//-------------------------------------------------------------------------------------------
void LCDwhiteSpaceFromTo(uint8_t line, uint8_t fromPos, uint8_t toPos)
{
  uint8_t curPos = fromPos;
  LCDsetPosition(line,fromPos); 
  do
  {
    myLCD.print(" "); curPos++;
  }
  while (curPos != toPos+1);
}
//-------------------------------------------------------------------------------------------
void LCDwhiteSpace(uint8_t line, uint8_t fromPos, uint8_t length)
{
  LCDsetPosition(line,fromPos); 
  for (uint8_t curPos = 1; curPos < length+1; curPos++) { myLCD.print(" "); }
}
//-------------------------------------------------------------------------------------------
void LCDturnDisplayOn() //this turns the display back ON
{
  myLCD.write(0xFE); myLCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void LCDbrightness(uint8_t brightness) //this function takes an int between 0-255 and turns the backlight on accordingly
// 0 = OFF, 255 = Fully ON, everything in-between = varied brightness [out of possible 30 different brightness levels]
//
{
  myLCD.write(0x7C); myLCD.write(map(brightness, 0, 255, 128, 157)); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec 
}
//-------------------------------------------------------------------------------------------
void LCDbacklight(uint8_t lvl) //this function takes an int between 128-157 and turns the backlight on accordingly
{
  if (lvl<=157 && lvl>=128) {myLCD.write(0x7C); myLCD.write(lvl);}  // any value between 128 and 157 or 0x80 and 0x9D
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

//##PROGRAMMING
//NOTE: Characters in CGRAM are programmed by selecting a position by the following: 0   1   A   A   A   A   A   A, as in (0b01xxxxxx), as in 0x40 for position 0, and 0x52 for a different position
//-------------------------------------------------------------------------------------------
void setGFX(byte pos, byte b1, byte b2, byte b3, byte b4, byte b5, byte b6, byte b7, byte b8){
  myLCD.write((byte)0xFE); // appel de fonction sur le HD44780
  myLCD.write((byte)64 + pos * 8); // accès à la CGRAM
  myLCD.write((byte)b1);
  myLCD.write((byte)b2);
  myLCD.write((byte)b3);
  myLCD.write((byte)b4);
  myLCD.write((byte)b5);
  myLCD.write((byte)b6);
  myLCD.write((byte)b7);
  myLCD.write((byte)b8);
}
//-------------------------------------------------------------------------------------------
void LEDsetCustomCharacters() {
  setGFX(0,142,155,145,145,145,145,145,159); //batt empty
  setGFX(1,142,155,145,145,145,145,159,159); //batt 1/5
  setGFX(2,142,155,145,145,145,159,159,159); //batt 2/5
  setGFX(3,142,155,145,145,159,159,159,159); //batt 3/5
  setGFX(4,142,155,145,159,159,159,159,159); //batt 4/5
  setGFX(5,142,159,159,159,159,159,159,159); //batt full
  //setGFX(4,152,152,128,131,132,132,131,128); // °c
  //setGFX(6,152,152,128,131,132,132,132,131); //deg capitalC //we don't use Celsius here, so optional
  //setGFX(6,129,131,134,143,159,134,140,152); //lightning bolt
  setGFX(6,138,159,145,142,132,132,152,128); // plug
  setGFX(7,152,152,128,135,132,135,132,132); //deg capitalF
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
void LCDbacklightFade()
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
