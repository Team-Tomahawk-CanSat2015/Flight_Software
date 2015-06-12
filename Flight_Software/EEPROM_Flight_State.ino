/**
* EEPROM functions
* including CanSat Boot and SaveState
**/
#include <EEPROM.h>
#include "EEPROMAnything.h"


void ClearMemory()
{
  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0);
  }
}

/**
* Boot Sequence Method
* Loads flight software state from memory
* Loads:
* - State
* - PacketCount
**/
void boot()
{
  byte location = 0;
  location+=EEPROM_readAnything(location,state);
  location+=EEPROM_readAnything(location,packet_count);
  location+=EEPROM_readAnything(location,initialize_time);
  location+=EEPROM_readAnything(location,prev_Time);
  location+=EEPROM_readAnything(location,stateStartTime);
  location+=EEPROM_readAnything(location,ground_alt);
  location+=EEPROM_readAnything(location,init_Heading);
  location+=EEPROM_readAnything(location,alt_buffer);
  location+=EEPROM_readAnything(location,alt_buffer_time);
  
//  Serial.println ("***boot***");
//  Serial.print(state);
//  Serial.print(",");
//  Serial.print(packet_count);
//  Serial.print(",");
//  Serial.print(initialize_time);
//  Serial.print(",");
//  Serial.print(prev_Time);
//  Serial.print(",");
//  Serial.print(liftoff_time);
//  Serial.print(",");
//  Serial.print(ground_alt);
//  Serial.print("\n");
}

/**
* Save the Flight Software state to memory
* currently Saving:
* - Flight State
* - packetCount
*/
void saveState()
{ byte location = 0;
  location+=EEPROM_writeAnything(location,state);
  location+=EEPROM_writeAnything(location,packet_count);
  location+=EEPROM_writeAnything(location,initialize_time);
  location+=EEPROM_writeAnything(location,prev_Time);
  location+=EEPROM_writeAnything(location,stateStartTime);
  location+=EEPROM_writeAnything(location,ground_alt);
  location+=EEPROM_writeAnything(location,init_Heading);
  location+=EEPROM_writeAnything(location,alt_buffer);
  location+=EEPROM_writeAnything(location,alt_buffer_time);
}
