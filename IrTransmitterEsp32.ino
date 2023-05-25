#include <SPIFFS.h>
#include <WiFi.h>
#include "esp_sleep.h"
//#include <Eeprom24C32_64.h>
//#include <StringTokenizer.h>
//schedule processor
#include <HardwareSerial.h>
#include <esp_adc_cal.h>
#include <Wire.h>
#include <TimeLib.h>
//#include <esp_task_wdt.h>
#include "defs.h"

#define SENSOR_VP                         GPIO_NUM_36
#define RS485_BAUD                        115200

/*__________________________________________
UART  | RX IO   | TX IO   | CTS   | RTS
-------------------------------------------
UART0 | GPIO3   | GPIO1   | N/A   | N/A
UART1 | GPIO9   | GPIO10  | GPIO6 | GPIO11
UART2 | GPIO16  | GPIO17  | GPIO8 | GPIO7
___________________________________________*/

HardwareSerial SerialIrda(1);

//3 seconds WDT
#define WDT_TIMEOUT                       3 //in seconds

unsigned long hb_currentMillis            = 0;
unsigned long hb_previousMillis           = 0; 
unsigned long hb_interval                 = 50;
unsigned long adc_read_interval           = 10000; // 5 minutes
unsigned long adc_read_counts             = 0;
unsigned char debounce_counts             = 20;
int led_blinking_counts                   = 0;
int serial_counts                         = 0;
String irdaRxString                       = "";
char serial_rxd[20];

//esp_adc_cal_value_t val_type;
//esp_adc_cal_characteristics_t *adc_chars = new esp_adc_cal_characteristics_t;
//unsigned long check_time_counter = 0;

tmElements_t tm_recorded;
tmElements_t tm;
char datetimeBuffer[30];

boolean battery_low_flag                  = false;
//unsigned int battery_low_led_blink_counts = 0;

////////////////////////////////////////////////////////////
#define BUTTON_PIN_BITMASK                0x002000000 // 2^25 in hex  

RTC_DATA_ATTR int bootCount               = 0;
RTC_DATA_ATTR boolean lid_close_flag      = false;
RTC_DATA_ATTR boolean lock_code           = false;

RTC_DATA_ATTR time_t recorded_time;
RTC_DATA_ATTR unsigned long seconds_diff;
unsigned long time_in_us                  = 5000000;

///////////////////TIMER///////////////////// 
void IRAM_ATTR onTimer(){
   portENTER_CRITICAL_ISR(&timerMux);
   timer_flag = true;
   portEXIT_CRITICAL_ISR(&timerMux);   
}

void initTimer(){
    // Use 1st timer of 4 
    // 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up 
    timer = timerBegin(0, 80, true);
    // Attach onTimer function to our timer UL_IN2
    timerAttachInterrupt(timer, &onTimer, true);
    // Set alarm to call onTimer function every second 1 tick is 1us
    //=> 1 second is 1000000us 
    // Repeat the alarm (third parameter) 
    timerAlarmWrite(timer, 1000000, true);
    // Start timer 
    //timerAlarmEnable(timer);
    sei();
}

//ADC1_CH6 (GPIO 34)
//ADC1_CH4 (GPIO 32)
void readBatteryStatus(){
    int samples = 50;
    uint32_t sum = 0;
    uint16_t voltage = 0;
    
    for (int x=0; x<samples; x++) {
      delayMicroseconds(100);
      sum += analogRead(A4);
    }
    sum /= samples;

    Serial.print("battery_counts: ");
    Serial.println(sum); 
    if(sum <= BATTERY_DRAIN_LEVEL){
       battery_low_flag = true;
    }else{
       battery_low_flag = false;
       digitalWrite(LED_BL_INDI, LOW);
    }
}


boolean checkSerial() {
  boolean response_flag = false;
  if(SerialIrda.available()){
      rx_frame_length = 0; 
      digitalWrite(LED_RF_RX, 1);
      rx_frame_array[rx_frame_length++] = (uint8_t)SerialIrda.read();
      if(rx_frame_array[rx_frame_length-1] == START_BYTE){
        //response_flag = true;
        while(SerialIrda.available()){
          rx_frame_array[rx_frame_length++] = (uint8_t)SerialIrda.read();
          if(rx_frame_array[rx_frame_length-1] == END_LF){
            if(rx_frame_array[rx_frame_length-2] == END_CR){
                 if(rx_frame_array[1] == slave_id){
                     response_flag = true; 
                     Serial.println("Received CRLF");
                 }
                 break;
            }
          }
        }
        SerialIrda.flush();
      }
    }
    return response_flag;
}

void doActualTask(){
    digitalWrite(UL_IN2, LOW);
    digitalWrite(UL_IN1, LOW); //Turn OFF Green LED
    digitalWrite(UL_IN4, LOW); //Turn OFF Indicator
    
    if(lid_close_flag){ //LID CLOSE -- ACTIVE LOW
       Serial.println("LID CLOSED");
       //time_t tt = readDS3231();
       seconds_diff = getRTCtime() - recorded_time; //in seconds
       Serial.print("seconds_diff: ");  Serial.println(seconds_diff, DEC);
       if(seconds_diff >= LID_CLOSING_TIME_LIMIT){ //lid closing time greater than 4 hours
          lock_code = true;
          Serial.println("Turn ON Red LED");
          digitalWrite(UL_IN2, HIGH); //Turn ON Red LED
       }else{ //end if(seconds_diff >= LID_CLOSING_TIME_LIMIT){
          sendMessage();
       }
    }
} //end void doActualTask(){

void holdGpios(boolean is_enable){
  if(is_enable){
    gpio_hold_en(GPIO_LED_HB);
    gpio_hold_en(GPIO_LED_RF_RX);
    gpio_hold_en(GPIO_LED_BL_INDI);
    //gpio_hold_en(GPIO_NUM_34);
    gpio_hold_en(GPIO_IRF_EN);
    gpio_hold_en(GPIO_UL_IN1);
    gpio_hold_en(GPIO_UL_IN2);
    gpio_hold_en(GPIO_UL_IN3);
    gpio_hold_en(GPIO_UL_IN4);
    gpio_deep_sleep_hold_en();
  }else{ //if(is_enable){
    gpio_hold_dis(GPIO_LED_HB);
    gpio_hold_dis(GPIO_LED_RF_RX);
    gpio_hold_dis(GPIO_LED_BL_INDI);
    gpio_hold_dis(GPIO_IRF_EN);
    //gpio_hold_dis(GPIO_NUM_34);
    gpio_hold_dis(GPIO_UL_IN1);
    gpio_hold_dis(GPIO_UL_IN2);
    gpio_hold_dis(GPIO_UL_IN3);
    gpio_hold_dis(GPIO_UL_IN4);    
    gpio_deep_sleep_hold_dis();
  } //end }else{
}

void letsSleep(boolean is_enable){
  holdGpios(is_enable);
  //Go to sleep now
  Serial.println("Going to sleep now");
  delay(10);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
////////////////////////////////////////////////////////////////
void loopNew(){
  if(!lock_code){
    if(checkSerial()){
      digitalWrite(IRF_EN, 0);
      Serial.print("Response received: ");
      for(int i=0; i<rx_frame_length; i++){
        Serial.print(rx_frame_array[i], DEC);
      }
      if(crc_test_rx() > 0){
        if(rx_frame_array[2] == REQUEST_SERIAL){
          if(rx_frame_array[3] == RESPONSE_OK){
             if(!timerEnable){
               timerAlarmDisable(timer);
             }
             send_counter = 0;
             Serial.println("OK");
             Serial.println("Turn ON Green LED");
             Serial.println("Turn ON UL_IN4");
             Serial.print("LID Opened after ");
             Serial.println(seconds_diff, DEC);
             ///////////////////////////////////////////////////
             digitalWrite(UL_IN2, LOW);
             digitalWrite(UL_IN1, HIGH); //Turn ON Green LED
             digitalWrite(UL_IN4, HIGH); //Turn ON Indicator
             timer_working_mode = 1;
             indicator_counter = 0;
             timerAlarmEnable(timer);
             timerEnable = true;
          }else{
            Serial.println("Turn ON Red LED");
            digitalWrite(UL_IN2, HIGH); //Turn ON Red LED
            letsSleep(true);
          }
        }else{
          Serial.println("Turn ON Red LED");
          digitalWrite(UL_IN2, HIGH); //Turn ON Red LED
          letsSleep(true);
        }
      }else{
        Serial.println("Turn ON Red LED");
        digitalWrite(UL_IN2, HIGH); //Turn ON Red LED
        letsSleep(true);
      }
      digitalWrite(LED_RF_RX, LOW);
    }    
    //turn of indicators and send repeat messages
    if(timer_flag){
       portENTER_CRITICAL_ISR(&timerMux);
       timer_flag = false;
       portEXIT_CRITICAL_ISR(&timerMux); 
       if(timer_working_mode == 0){
          //timerAlarmDisable(timer);
          sendMessage(); 
       }else{
          indicator_counter++;
          if(indicator_counter == 2){ //Stop UL_IN4 After 1 second
            digitalWrite(UL_IN4, LOW); //Turn OFF UL_IN4 
            Serial.println("Turn OFF UL_IN4");
          } else if(indicator_counter > 5){  //Stop Indicator After 5 second
            digitalWrite(UL_IN1, LOW); //Turn OFF Green LED
            digitalWrite(UL_IN2, LOW); //Turn OFF Red LED
            Serial.println("Turn OFF Green LED"); Serial.println("Turn OFF Red LED");
            indicator_counter = 0;
            timerAlarmDisable(timer);
            //digitalWrite(IRF_EN, 0);
            letsSleep(true);
          }
       }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkLidClosed(boolean isRecordLidClosingTime){
     if(digitalRead(IN2) == LOW){
       esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1); //1 = High, 0 = Low
       Serial.println("LID Closed");
       lid_close_flag = true;
       lid_open_flag = false;
       if(isRecordLidClosingTime){
          recorded_time = getRTCtime();
          //setTimeInEeprom(recorded_time);
          Serial.print("LID Closed at "); //Serial.println(recorded_time, DEC);
          breakTime(recorded_time, tm);
          sprintf(datetimeBuffer, "%02d-%02d-%02d  %02d  %02d:%02d:%02d", tm.Day, tm.Month, tm.Year, tm.Wday, tm.Hour, tm.Minute, tm.Second);
          Serial.println(datetimeBuffer); 
       }
    }else{
       esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); //1 = High, 0 = Low
       Serial.println("LID Opened");
       lid_close_flag = false;
       lid_open_flag = true;
    }
}



/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); 
              
              checkLidClosed(true);  
              esp_sleep_enable_timer_wakeup(time_in_us);
              esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); //1 = High, 0 = Low
              letsSleep(true);
    break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
              esp_sleep_enable_timer_wakeup(time_in_us);
              esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); //1 = High, 0 = Low
              checkLidClosed(false);
              if(lid_close_flag){
                 doActualTask();
              }else{
                  delay(10);
                  letsSleep(true);      
              }
    break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by Timer");
              esp_sleep_enable_timer_wakeup(time_in_us);
              esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); //1 = High, 0 = Low
              checkLidClosed(false);
              //delay(10);
              letsSleep(true);
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);  
              esp_sleep_enable_timer_wakeup(time_in_us);
              //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); //1 = High, 0 = Low
              checkLidClosed(false);
              esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); //1 = High, 0 = Low
              letsSleep(true);
              break;
  }
}


void heartBeatBlinking() {
  hb_currentMillis = millis();
  if (hb_currentMillis - hb_previousMillis >= hb_interval) {
      hb_previousMillis = hb_currentMillis;
      //HB Led Blinking//
      if(check_led_blink_flag){
        if(led_blinking_counts++ >=10){
            led_blinking_counts = 0;
            led_pulse_flag = true;
            check_led_blink_flag = false;
        }
      }else{
        if(led_pulse_flag){
          led_pulse_flag = false;
          digitalWrite(LED_HB, HIGH);
          /////////////////////////////
          if(test_mode){
            readBatteryStatus();
            digitalWrite(IRF_EN, 1);
            delay(50); //this delay is required
            createRequest();
            sendRequest();
          }
        }else{
          digitalWrite(LED_HB, LOW);
          check_led_blink_flag = true;
        }
      }
      //END of HB Led Blinking//
  } 
}

uint32_t prevMillis = 0;


//uint32_t readADC_Cal(int ADC_Raw){
//  esp_adc_cal_characteristics_t adc_chars;
//  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
//  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
//}

void setup(){
  ////////////////////////////////////////////////////////////////////  
  pinMode(LED_HB, OUTPUT);
  pinMode(LED_RF_RX, OUTPUT);
  pinMode(LED_BL_INDI, OUTPUT);
  digitalWrite(LED_BL_INDI, LOW);
  pinMode(UL_IN1, OUTPUT);
  pinMode(UL_IN2, OUTPUT);
  pinMode(UL_IN3, OUTPUT);
  pinMode(UL_IN4, OUTPUT);
  pinMode(BATTERY_IN, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IRF_EN, OUTPUT);
  Serial.begin(115200);
  holdGpios(false);
  
  digitalWrite(LED_HB, HIGH);
  digitalWrite(LED_RF_RX, LOW);
  digitalWrite(LED_BL_INDI, LOW);

  digitalWrite(UL_IN1, LOW);
  digitalWrite(UL_IN2, LOW);
  digitalWrite(UL_IN3, LOW);
  digitalWrite(UL_IN4, LOW);

  digitalWrite(IRF_EN, LOW);
  digitalWrite(LED_RF_RX, LOW);
  digitalWrite(LED_BL_INDI, LOW);

  //SerialIrda.begin(19200, SERIAL_8N1, 13, 15);//15-TXD, 13-RXD  old pcb
  SerialIrda.begin(19200, SERIAL_8N1, 15, 13);//15-TXD, 13-RXD
  delay(1);
  I2C_int.begin((uint8_t)21, (uint8_t)22, 100000);
  //delay(1); 
  //ADC1_CHANNEL_4
  //analogSetPinAttenuation(ADC1_CHANNEL_6, ADC_11db); 
  //adcAttachPin(GPIO_NUM_32); 
  //analogSetWidth(12);                       //
  delay(1); //
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed");
    //Flash HB LED fast to indicate error
    digitalWrite(LED_HB, HIGH);
    //while (1); //Hang In There
    slave_id = 1;
    test_mode = 0;
  }
  delay(1);
  read_config();
  /////////////////////////////////////////////////////////
  time_t tt = readDS3231();
  //delay(10);
 // recorded_time = getTimeInEeprom();
  if(recorded_time == 0) recorded_time = tt;
  unsigned long hh = tt - recorded_time;
  Serial.print("recorded_time: "); Serial.println(recorded_time);
  Serial.print("current time: "); Serial.println(tt);
  Serial.print("diff in secs: "); Serial.println(hh);

  delay(1);
  readBatteryStatus();
  if(battery_low_flag){
    digitalWrite(LED_BL_INDI, HIGH); //
  }   
  delay(10);
  digitalWrite(LED_BL_INDI, LOW);    //
  digitalWrite(LED_HB, LOW);         //
  //
  digitalWrite(IRF_EN, HIGH);        //
  //Print the wakeup reason for ESP32
  if(test_mode == 0){
    initTimer();                     //  
    print_wakeup_reason();           //
  }
}

void loop(){
  //This is not going to be called
  loopNew();
  heartBeatBlinking();
  /////////////////////////////////////////////////////////////////////
//  //if button is pressed
//  if(digitalRead(IN1) == LOW){
//      //if(buton_pressed_flag){
//      //  buton_pressed_flag = false;
//        delay(50); //debounce delay
//        //wait for button to become HIGH
//        while(digitalRead(IN1) == LOW);
//
//        
//        digitalWrite(UL_IN2, LOW);
//        digitalWrite(UL_IN1, LOW); //Turn OFF Green LED
//        digitalWrite(UL_IN4, LOW); //Turn OFF Indicator
//        if(digitalRead(IN2) == LOW){ //LID CLOSE -- ACTIVE LOW
//             Serial.println("LID CLOSED");
//             seconds_diff = (getRTCtime() - recorded_time); //in seconds
//              Serial.println(seconds_diff, DEC);
//             if(seconds_diff >= LID_CLOSING_TIME_LIMIT){ //lid closing time greater than 4 hours
//                lock_code = true;
//                Serial.println("Turn ON Red LED");
//                digitalWrite(UL_IN2, HIGH); //Turn ON Red LED
//             }else{
//                sendMessage();
//             }
//        }
//  }    
  //if lid is closed when not in sleep mode, i.e. esp32 in active mode
  if(digitalRead(IN2) == LOW) { //LID CLOSE -- ACTIVE LOW
     if(lid_open_flag) {
        lid_open_flag   = false;  
        lid_close_flag  = true;
        recorded_time   = getRTCtime();
        //setTimeInEeprom(recorded_time);
        Serial.print("LID Closed at "); //Serial.println(recorded_time, DEC);
        breakTime(recorded_time, tm);
        sprintf(datetimeBuffer, "%02d-%02d-%02d  %02d  %02d:%02d:%02d", tm.Day, tm.Month, tm.Year, tm.Wday, tm.Hour, tm.Minute, tm.Second);
        Serial.println(datetimeBuffer); 
     }
  }else if(digitalRead(IN2) == HIGH) {
      lid_open_flag     = true; 
      lid_close_flag    = false;
  }
}
