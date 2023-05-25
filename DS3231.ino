/////////////////////RTC DS3231////////////////
    
    #define CLOCK_ADDRESS  0x68 
    tmElements_t getRTCtimeElements() {
      byte sec;
      tmElements_t tm;
    
      I2C_int.beginTransmission(0x68);
      I2C_int.write(0);
      uint8_t err = I2C_int.endTransmission(true);
    //  if(err!= 0){
    //    Serial.println("RTCC not present at address 0x68.");
    //  }
      I2C_int.requestFrom(0x68,7);  
      if (I2C_int.available() < 7) return tm;
      sec = I2C_int.read();
      tm.Second = bcd2dec(sec & 0x7f);   
      tm.Minute = bcd2dec(I2C_int.read() );
      tm.Hour =   bcd2dec(I2C_int.read() & 0x3f);  // mask assumes 24hr clock
      tm.Wday = bcd2dec(I2C_int.read() );
      tm.Day = bcd2dec(I2C_int.read() );
      tm.Month = bcd2dec(I2C_int.read() & 0x7f ); 
      tm.Year = y2kYearToTm((bcd2dec(I2C_int.read())));
      return tm; 
    }
    
    time_t getRTCtime() {
      tmElements_t tm = getRTCtimeElements();
      return (makeTime(tm)); 
    }
    
    
    bool setRTCtime(tmElements_t tm){
        //tmElements_t tm; 
        //breakTime(intime, tm);
        if (tm.Year >= 1000)
            tm.Year = CalendarYrToTm(tm.Year)%100;
        else    //(tm.Year < 100)
            tm.Year = y2kYearToTm(tm.Year);
      // To eliminate any potential race conditions,
      // stop the clock before writing the values,
      // then restart it after.
        I2C_int.beginTransmission(0x68);
        I2C_int.write((uint8_t)0x00); // reset register pointer  
    //  I2C_int.write((uint8_t)0x80); // Stop the clock. The seconds will be written last
        I2C_int.write(dec2bcd(tm.Second));
        I2C_int.write(dec2bcd(tm.Minute));
        I2C_int.write(dec2bcd(tm.Hour));      // sets 24 hour format
        I2C_int.write(dec2bcd(tm.Wday));   
        I2C_int.write(dec2bcd(tm.Day));
        I2C_int.write(dec2bcd(tm.Month));   
        I2C_int.write(dec2bcd(tm.Year)); 
        if (I2C_int.endTransmission() != 0) {
    //    exists = false;
        return false;
      }
      return true;
    }
    
    bool setRTCtime2(tmElements_t tm){
    
      // To eliminate any potential race conditions,
      // stop the clock before writing the values,
      // then restart it after.
        I2C_int.beginTransmission(0x68);
        I2C_int.write((uint8_t)0x00); // reset register pointer  
    //  I2C_int.write((uint8_t)0x80); // Stop the clock. The seconds will be written last
        I2C_int.write(dec2bcd(tm.Second));
        I2C_int.write(dec2bcd(tm.Minute));
        I2C_int.write(dec2bcd(tm.Hour));      // sets 24 hour format
        I2C_int.write(dec2bcd(tm.Wday));   
        I2C_int.write(dec2bcd(tm.Day));
        I2C_int.write(dec2bcd(tm.Month));   
        I2C_int.write(dec2bcd(tm.Year)); 
        if (I2C_int.endTransmission() != 0) {
    //    exists = false;
        return false;
      }
      return true;
    }
    
    byte dec2bcd(byte val) {
      return ((val/10*16) + (val%10));
    }
    byte bcd2dec(byte val) {
      return ((val/16*10) + (val%16));
    }

 
    ////////////////////////////////////////////////////////////
  time_t writeTimeToRtc(uint8_t sec, uint8_t min, uint8_t hr, uint8_t day, uint8_t mon, uint8_t yr) {
    tmElements_t tm;
    tm.Hour = hr;
    tm.Minute = min;
    tm.Second = sec;

    tm.Day  = day;
    tm.Month = mon;
    tm.Year = yr;
    setRTCtime2(tm);
    return makeTime(tm);
}


time_t readDS3231() {
    char datetimeBuffer[30];
    tmElements_t tm;
    time_t tt = getRTCtime();
    breakTime(tt, tm);
    sprintf(datetimeBuffer, "%02d-%02d-%02d  %02d  %02d:%02d:%02d", tm.Day, tm.Month, tm.Year, tm.Wday, tm.Hour, tm.Minute, tm.Second);
    Serial.println(datetimeBuffer); 
    return tt;

}

///////////////////////////////////////////////////////////////////////////////////////
void getTime(byte& year, byte& month, byte& date, byte& DoW, byte& hour, byte& minute, byte& second) {
  byte tempBuffer;
  bool PM;
  bool h12;

  I2C_int.beginTransmission(0x68);
  I2C_int.write(uint8_t(0x00));
  I2C_int.endTransmission();
  
  I2C_int.requestFrom(0x68, 7);

  second = bcd2dec(I2C_int.read());
  minute = bcd2dec(I2C_int.read());
  tempBuffer = bcd2dec(I2C_int.read());
  h12 = tempBuffer & 0b01000000;
  if (h12) {
    PM = tempBuffer & 0b00100000;
    hour = bcd2dec(tempBuffer & 0b00011111);
  } else {
    hour = bcd2dec(tempBuffer & 0b00111111);
  }
  DoW = bcd2dec(I2C_int.read());
  date = bcd2dec(I2C_int.read());
  month = bcd2dec(I2C_int.read() & 0b01111111);
  year = bcd2dec(I2C_int.read());
}





void read_config() {
  String str;
  unsigned char cnt = 0;
  Serial.println("*******Reading Config file*******");
  File file = SPIFFS.open("/config.csv");
  if (!file) {
    Serial.println("- failed to open file for reading");
    return;
  }
 
  if(file.available()) {
    str = file.readStringUntil('\n');
    str = str.substring(str.indexOf(',', 1) + 1);
    slave_id = (uint8_t) str.toInt();
    Serial.print("slave_id: ");
    Serial.println(slave_id, DEC);
  }
  ///////////////////////////////////
  if(file.available()) {
    str = file.readStringUntil('\n');
    str = str.substring(str.indexOf(',', 1) + 1);
    test_mode = (uint8_t) str.toInt();
    Serial.print("test_mode: ");
    Serial.println(test_mode, DEC);
  }
  file.close();
}
/////////////////////////////EEPROM///////////////////////////////
//void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data ) {
//    int rdata = data;
//    I2C_int.beginTransmission(deviceaddress);
//    I2C_int.write((int)(eeaddress >> 8)); // MSB
//    I2C_int.write((int)(eeaddress & 0xFF)); // LSB
//    I2C_int.write(rdata);
//    I2C_int.endTransmission();
//    delay(10);
//  }
//
//  // WARNING: address is a page address, 6-bit end will wrap around
//  // also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
//  void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) {
//    I2C_int.beginTransmission(deviceaddress);
//    I2C_int.write((int)(eeaddresspage >> 8)); // MSB
//    I2C_int.write((int)(eeaddresspage & 0xFF)); // LSB
//    byte c;
//    for ( c = 0; c < length; c++)
//      I2C_int.write(data[c]);
//    I2C_int.endTransmission();
//  }
//
//  byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
//    byte rdata = 0xFF;
//    I2C_int.beginTransmission(deviceaddress);
//    I2C_int.write((int)(eeaddress >> 8)); // MSB
//    I2C_int.write((int)(eeaddress & 0xFF)); // LSB
//    I2C_int.endTransmission();
//    I2C_int.requestFrom(deviceaddress,1);
//    if (I2C_int.available()) rdata = I2C_int.read();
//    delay(10);
//    return rdata;
//  }
//
//  // maybe let's not read more than 30 or 32 bytes at a time!
//  void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
//    I2C_int.beginTransmission(deviceaddress);
//    I2C_int.write((int)(eeaddress >> 8)); // MSB
//    I2C_int.write((int)(eeaddress & 0xFF)); // LSB
//    I2C_int.endTransmission();
//    I2C_int.requestFrom(deviceaddress,length);
//    int c = 0;
//    for ( c = 0; c < length; c++ )
//      if (I2C_int.available()) buffer[c] = I2C_int.read();
//  }


// void setTimeInEeprom(time_t   ttime){
//     tmElements_t tm;
//     breakTime(ttime, tm);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 0, tm.Second);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 1, tm.Minute);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 2, tm.Hour);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 3, tm.Wday);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 4, tm.Day);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 5, tm.Month);
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 6, ((tm.Year >> 8) & 0xff));
//     i2c_eeprom_write_byte(EEPROM_ADDRESS, 7, (tm.Year & 0xff));
// }
//
// time_t getTimeInEeprom(){
//      unsigned int date;
//      char datetimeBuffer[30];
//      tmElements_t tm;
//      tm.Second = i2c_eeprom_read_byte(EEPROM_ADDRESS, 0) ;   
//      tm.Minute = i2c_eeprom_read_byte(EEPROM_ADDRESS, 1) ; 
//      tm.Hour =   i2c_eeprom_read_byte(EEPROM_ADDRESS, 2) ; 
//      tm.Wday = i2c_eeprom_read_byte(EEPROM_ADDRESS, 3) ; 
//      tm.Day = i2c_eeprom_read_byte(EEPROM_ADDRESS, 4) ; 
//      tm.Month = i2c_eeprom_read_byte(EEPROM_ADDRESS, 5) ; 
//       
//      date = i2c_eeprom_read_byte(EEPROM_ADDRESS, 6) << 8;
//      date |= i2c_eeprom_read_byte(EEPROM_ADDRESS, 7);
//      tm.Year =  date;
//
//    //breakTime(recorded_time, tm_recorded);
//
//
//      
//      sprintf(datetimeBuffer, "%02d-%02d-%02d  %02d  %02d:%02d:%02d", tm.Day, tm.Month, tm.Year, tm.Wday, tm.Hour, tm.Minute, tm.Second);
//      Serial.println(datetimeBuffer); 
//    
//      return (makeTime(tm)); 
// }
