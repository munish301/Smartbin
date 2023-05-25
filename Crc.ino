unsigned char crc_test_rx(void)
{
  unsigned char i,h = 0;
  unsigned int crc_temp_reg = 0xffff;
  
  for (h = 0;h <(rx_frame_length - 4); h++)
  {
    crc_temp_reg ^= rx_frame_array[h];          // XOR byte into least sig. byte of crc
    
    for(i = 8;i != 0;i--)
    {    // Loop over each bit
      if ((crc_temp_reg & 0x0001) != 0)
      {      // If the LSB is set
        crc_temp_reg >>= 1;                    // Shift right and XOR 0xA001
        crc_temp_reg ^= 0xA001;
      }
      else                            // Else LSB is not set
      crc_temp_reg >>= 1;                    // Just shift right          //plc send lsb then msb
    }
  }
  unsigned int crc_temp_reg1 = ((rx_frame_array[rx_frame_length-3] <<8) | rx_frame_array[rx_frame_length-4]);
 // Serial.print("CRC RX: "+crc_temp_reg1);
  if (crc_temp_reg == crc_temp_reg1)
    return 1;
  return 0;
}



/*************************************
 Fxn : unsigned int CRC16(unsigned int crc, unsigned int data)
 Info: CRC16 calculation
 *************************************/
unsigned int CRC16(unsigned int crc, unsigned int data)
{
  const unsigned int Poly16=0xA001;
  unsigned int LSB, i;

  crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
  for (i=0; i<8; i++) 
  {
    LSB=(crc & 0x0001);
    crc=crc/2;
    if (LSB)
      crc=crc^Poly16;
  }
  return(crc);
}

/*************************************
 Fxn : void calculateCRC16(uint8_t NUMDATA, unsigned char * data, unsigned char * Crc_HByte,unsigned char * Crc_LByte)
 Info: CRC16 calculation
 *************************************/
void calculateCRC16(uint8_t numData, byte * data, byte * Crc_HByte,byte * Crc_LByte)
{

  unsigned int Crc = 0xFFFF;

  for (uint8_t i=0; i<numData; i++) 
  {
    Crc = CRC16 (Crc, data[i] );
  }
  *Crc_LByte = (Crc & 0x00FF);    // Low byte calculation
  *Crc_HByte = (Crc & 0xFF00) / 256;  // High byte calculation
}








void write_config(String str) {
    //Serial.println("Writing Config file");
  File file = SPIFFS.open("/config.csv", FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open Config file for writing");
    return;
  }
  file.seek(1, SeekEnd);
  file.println(str);
  file.close();
}
