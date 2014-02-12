void eeprom_set() {
  unsigned int EEPglobalDelay = 250; //global event delay in ms (milliseconds) {suggested values: 250, 500}
  EEPROM.write(1, 250);
  
  uint8_t EEPledLuxLevel = 200; //mode/XBee button LED brightness  (x out of 255)
  EEPROM.write(2, 200);
  
  uint8_t EEPswitchCount = 15; //the number of intervals that elapse before the light sensor switches reverts sensitivity
  EEPROM.write(3, 15);
  
  boolean EEPpowerSaveEnabled = false; //enable or disable power-save functions for the regulator ###experimental
  EEPROM.write(4, 0);
  
  Serial.println("EEPROM Success!");
} 
