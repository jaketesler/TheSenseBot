char chars[4] = {'|','/','-','\\'};

#include "serLCD.h"

void cyclone(int numTimes, double length, int line, int col) { //each cyclone is one second usually
  int cycleState = 0;
  //length in ms
  
  length /= 4;
  
  for (int i = 0; i < numTimes; i++)
  {/*
    for(int curChar = 0; curChar < 4; curChar++)
    {
      LCDsetPosition(line, col);
      myLCD.print(char)chars[curChar]);
      delay(length);
    }
    */
    
    LCDsetPosition(line, col);
    myLCD.write(0x7C);
    delay(length);
    LCDsetPosition(line, col);
    myLCD.write(0x2F);
    delay(length);
    LCDsetPosition(line, col);
    myLCD.write(0x2D);
    delay(length);
    LCDsetPosition(line, col);
    //myLCD.write(0xDF);
    myLCD.write(0x5C);
    delay(length);
    
  }
  
  
}

void cyclone(int numTimes, int line, int col)
{
  cyclone(numTimes, (double)1000, line, col);
}



