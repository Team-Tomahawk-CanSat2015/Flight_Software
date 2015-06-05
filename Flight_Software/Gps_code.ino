//CODE MAY SEEM ALITTLE STRANGE BUT DO NOT TRY TO MERGE FUNCTIONS OR EDIT TOO MUCH AS ALOT OF VARIABLES ARE DEPENDENT ON OTHERS.---THANKS ---TAYO
/**
* GPS usage functions
*
* Usage:
* -run setupGPS(); in setup()
* -pull data w getGPSdata();
**/

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean);


 //For unit Testing:
/*void setup()  
{
  Serial.begin(9600);
  setupGPS();
}

void loop (){
  //GPS needs to fix on a satlite else I set all values to (+ or - 9999)
  float latitude, longitude, alt;
  int secsfrom_midnight;
  getGPSdata (&latitude, &longitude, &alt,  &secsfrom_midnight);
  
}
*/

/**
* Setup GPS
**/
void setupGPS()
{
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
}






















//**********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************
//NESSESARY FUNCTIONS ARE BELOW NO NEED TO EDIT ANY THING JUST COPY PASTE
/*--DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS--*/
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
  float useless_value = 1;  //if (c) UDR0 = c;<--TO PRINT OUT FULL NNMEA SENTENCE--PLS DO NOT DELETE  
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

/**
* main GPS Function
* filles the passed variales for:
* - latitude
* - Longitude
**/
void getGPSdata(float *latitude, float *longitude, float *altitude, unsigned int *secsFromMidnight)                
{
  GPS.newNMEAreceived();
  GPS.parse(GPS.lastNMEA());
  if (GPS.fix >= 1) {
      *latitude  = GPS.latitudeDegrees;   
      *longitude = GPS.longitudeDegrees;
      *altitude  = GPS.altitude;     
  }
  else 
  {
  *latitude = 9999;   *longitude = -9999;  *altitude = 9999;
  }
   
  //------------------------------------------
  //Calculate seconds from midnight (0:0:0) 24h hour clock
//  Serial.print("GPS Time: hour:");
//  Serial.print(GPS.hour);
//  Serial.print(", min:");
//  Serial.print(GPS.minute);
//  Serial.print(", sec:");
//  Serial.print(GPS.seconds);
//  Serial.print(", milli:");
//  Serial.println(GPS.milliseconds);
  *secsFromMidnight = (GPS.hour * 60 *60) + (GPS.minute * 60) + (GPS.seconds);
}
