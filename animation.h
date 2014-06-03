#define PCBVERSION2.0

#ifdef PCBVERSION1.0
#define LASEREN 10
#endif

#ifdef PCBVERSION2.0
#define LASEREN 10
#endif


char cycleChars[4] = {'|','/','-','\\'};

//#include "serLCD.h"

void cyclone(uint8_t numTimes, double length, uint8_t line, uint8_t col) { //each cyclone is one second usually
  //int cycleState = 0;
  //length in ms
  
  length /= 4;
  
  //for (int i = 0; i < numTimes; i++)
  {
    for(int curChar = 0; curChar <= 3; curChar++)
    {
      LCDsetPosition(line, col); myLCD.print(cycleChars[curChar]); delay(length);
    }
    
    /*
    LCDsetPosition(line, col); myLCD.write(0x7C); delay(length);
    LCDsetPosition(line, col); myLCD.write(0x2F); delay(length);
    LCDsetPosition(line, col); myLCD.write(0x2D); delay(length);
    LCDsetPosition(line, col); myLCD.write(0x5C); delay(length);
    //myLCD.write(0xDF); delay(length);
    */
    
  }
  
  
}

void cyclone(uint8_t numTimes, uint8_t line, uint8_t col) { cyclone(numTimes, (double)1000, line, col); }


int8_t animStep = -17; //tracker for confusedAnimation()
static int8_t initAnimStep = -17;

void confusedAnimation(int curStep)
{
  /*if (curStep == -17)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -16)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -15)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing.");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -14)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing..");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -13)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -12)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
  }
  else if (curStep == -11)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -10)
  {
    LCDsetPosition(2,1);
    myLCD.print("                    ");
    LCDsetPosition(2,1);
    myLCD.print("Big");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -9)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -8)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -7)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -6)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("---");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
    LCDsetPosition(3,10);
    myLCD.print("--");
  }
 else if (curStep == -5)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-----");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,8);
    myLCD.print("----");
  }
  else if (curStep == -4)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
//    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  else if (curStep == -3)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  else if (curStep == -2)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  
  else if (curStep == -1)
  {
    LCDsetPosition(2,1);
    myLCD.print("              ");
    LCDsetPosition(3,1);
    myLCD.print("              ");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  
  else if(curStep == 0)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 1)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 2)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 3)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 4)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 5)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 6)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 7)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 8)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 9)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 10)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 11)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 12)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 13)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 14)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ -");
  }
  else if (curStep == 15)
  {
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 16)
  {
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 17)
  {
    LCDsetPosition(3,4);
    myLCD.print(" ");
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 18)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 19)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 20)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 21)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 22)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 23)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 24)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 25)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 26)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 27)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 28)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 29)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 30)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 31)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 32)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 33)
  {
    LCDsetPosition(3,1);
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 34)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 35)
  {
    LCDsetPosition(3,1);
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 36)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xFC);
    myLCD.write(0xA3);
    myLCD.write(0x2A);//another one
  }
  else if (curStep == 37)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 38)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x23);//another one
    myLCD.write(0xA3);
    myLCD.write(0x5E); //another one
  }
  else if (curStep == 39)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 40)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x22);//another one
    myLCD.write(0xA3);
    myLCD.write(0xEC); //another one
  }
  else if (curStep == 41)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 42)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xE0);//another one
    myLCD.write(0xA3);
    myLCD.write(0xF4); //another one
  }
  else if (curStep == 43)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 44)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 45)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 46)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 47)
  {
    LCDsetPosition(2,1);
    myLCD.print("- -");
  }
  else if (curStep == 48)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 49)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 50)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 51)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 52)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 53)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 54)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 55)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print(" ");
  }
  else if (curStep == 56)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 57)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 58)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 59)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 60)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 61)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 62)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 63)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 64)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 65)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 66)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 67)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 68)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 69)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 70)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH");
  }
  else if (curStep == 71)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 72)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 73)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 74)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_- SIGH...");
  }
  else if (curStep == 75)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 76)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 77)
  {
    LCDsetPosition(3,1);
    //myLCD.print("0_o");
    myLCD.print("           ");
    LCDsetPosition(2,1);
    myLCD.print("   ");
  }
  else if (curStep == 78)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 79)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    animStep = initAnimStep; //global var
  }
  else
  {
    curStep = initAnimStep;
    animStep = initAnimStep;
  }*/
}


