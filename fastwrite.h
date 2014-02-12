//http://skpang.co.uk/blog/archives/323
#define PD2 2
int outPin = 2;                 // Use digital pin 2 as output

void setup()
{
  pinMode(outPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
  PORTD |= 1<<PD2;       // sets output bit 2 high
  PORTD &= ~(1<<PD2);    // sets output bit 2 low
}
