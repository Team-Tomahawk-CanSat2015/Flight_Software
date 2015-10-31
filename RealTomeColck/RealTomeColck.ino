#include "Wire.h"
#define DS1307_I2C_ADDRESS 0x68  // the I2C address of Tiny RTC

void setup() {
 Wire.begin();
 Serial.begin(9600);
}


void loop() 
{
 unsigned long a_time;
 geta_time(&a_time);//get the time data from RTC
 Serial.print (a_time);
 delay (1000);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////**************RTC functions below**************************************/////////////////////

 // Used to Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)        {return ( (val/10*16) + (val%10) );}

// Used to Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)        {return ( (val/16*10) + (val%16) );}




void geta_time(unsigned long *a_time)
{
  byte second, minute, hour;
 // Reset the register pointer
 Wire.beginTransmission(DS1307_I2C_ADDRESS);
 Wire.write(decToBcd(0));
 Wire.endTransmission();
 Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
 second     = bcdToDec(Wire.read() & 0x7f);
 minute     = bcdToDec(Wire.read());
 hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
 *a_time = ((hour * 60 * 60) + (minute * 60) + (second));
}
 
