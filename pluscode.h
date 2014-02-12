#include <cstdarg.h>


#define PCBVERSION1.0

#ifdef PCBVERSION1.0
#define STAT1 A2
#define MCUONLED A3
#define STAT2 A6
#define ERRORLED A7

#elif defined PCBVERION2.0
#define STAT1 A2
#define MCUONLED A3
#define STAT2 A6
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
