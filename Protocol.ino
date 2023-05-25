/*
START    SLAVE_ADDR  FUNCTION    DATA     LRC_CHECK     END
1 CHAR   1 CHARS     1 CHARS     n CHARS  2 CHARS      2 CHARS(CR+LF)
*/
void createRequest(){
  uint8_t crc_H, crc_L;
  frame_length = 0;
  frame_array[frame_length++] = START_BYTE;
  frame_array[frame_length++] = slave_id;
  frame_array[frame_length++] = REQUEST_SERIAL; //fxn_code
  //DATA = NONE
  calculateCRC16(frame_length, frame_array, &crc_H, &crc_L);
  frame_array[frame_length++] = crc_L; //crc low byte
  frame_array[frame_length++] = crc_H;   //crc high byte
  frame_array[frame_length++] = END_CR; //crc low byte
  frame_array[frame_length++] = END_LF;   //crc high byte  
}

void sendRequest(){
  for(int i=0; i<frame_length; i++){
    SerialIrda.write(frame_array[i]);
    SerialIrda.flush();
    Serial.print(frame_array[i], DEC);
  }  
  Serial.println();     
}

void sendMessage(){
   if(send_counter < 5){
      timer_working_mode = 0;
      digitalWrite(IRF_EN, 1);
      delay(50); //this delay is required
      createRequest();
      sendRequest();
      //delay(10); //this delay is required
      //digitalWrite(IRF_EN, 0);
      send_counter++;
      timerAlarmEnable(timer);
      timerEnable = true;
    }else{
      send_counter = 0;
      timerAlarmDisable(timer);
      timerEnable = false;
      timer_working_mode = 1;
      indicator_counter = 0;
      timerAlarmEnable(timer);
      timerEnable = true;
      digitalWrite(IRF_EN, 0);
      digitalWrite(UL_IN2, 1); //Turn ON Red LED
      Serial.println("Turn ON Red LED");
    }
}
