/**
* EEPROM functions
* including CanSat Boot and SaveState
**/
#include <EEPROM.h>


void ClearMemory()
{
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
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
  state = EEPROM.read(0);
  packet_count = 0;
  packet_count = EEPROM.read(1);
  packet_count = packet_count<<8;
  packet_count |= EEPROM.read(2);
  
}

/**
* Save the Flight Software state to memory
* currently Saving:
* - Flight State
* - packetCount
*/
void saveState()
{
  EEPROM.write(0,state);
  EEPROM.write(1,highByte(packet_count));
  EEPROM.write(2,lowByte(packet_count));
}
