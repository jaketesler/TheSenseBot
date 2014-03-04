void eeprom_set() {
  unsigned int EEPglobalDelay = 250; //global event delay in ms (milliseconds) {suggested values: 250, 500}
  EEPROM.write(1, EEPglobalDelay);
  
  uint8_t EEPledLuxLevel = 200; //mode/XBee button LED brightness  (x out of 255)
  EEPROM.write(2, EEPledLuxLevel);
  
  uint8_t EEPswitchCount = 30; //the number of intervals that elapse before the light sensor switches reverts sensitivity
  EEPROM.write(3, EEPswitchCount);
  
  boolean EEPpowerSaveEnabled = false; //enable or disable power-save functions for the regulator ###experimental
  EEPROM.write(4, EEPpowerSaveEnabled);
  
  Serial.println("EEPROM Success!");
} 
