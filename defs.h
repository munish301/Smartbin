
int req_serial_key                  = 0;

#define EEPROM_ADDRESS              0x57

//uint8_t slave_id[5] = {0};

//static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);

gpio_num_t GPIO_LED_RF_RX         = GPIO_NUM_33;
gpio_num_t GPIO_LED_BL_INDI       = GPIO_NUM_27;
gpio_num_t GPIO_LED_HB            = GPIO_NUM_26;
gpio_num_t GPIO_UL_IN1            = GPIO_NUM_19;
gpio_num_t GPIO_UL_IN2            = GPIO_NUM_18;
gpio_num_t GPIO_UL_IN3            = GPIO_NUM_5;
gpio_num_t GPIO_UL_IN4            = GPIO_NUM_17;
gpio_num_t GPIO_IRF_EN            = GPIO_NUM_16;

//LEDs
const int LED_RF_RX                 = 33;
const int LED_BL_INDI               = 27;
const int LED_HB                    = 26;

//ULN2804
const int UL_IN1                    = 19;
const int UL_IN2                    = 18;
const int UL_IN3                    = 5;
const int UL_IN4                    = 17;

//Inputs
const int IN1                       = 25; //14; //push button
const int IN2                       = 4;  //limit switch

//Irda Enable
const int IRF_EN                    = 16;
//Adc Input
const int BATTERY_IN                = 32;//34;

//Wire
TwoWire I2C_int                     = TwoWire(0);
volatile unsigned int timer_counts  = 0;
volatile bool timer_flag            = false;
//bool timer_enable                 = false;
/* create a hardware timer */
hw_timer_t * timer                  = NULL;

portMUX_TYPE timerMux               = portMUX_INITIALIZER_UNLOCKED;
//                                                                                                   10K      1K
#define BATTERY_DRAIN_LEVEL         2900 //2170 // 9.2V   -- battery charged volt=14.2 V    BAT+--^^^^^--.--^^^^^--GND
//                                                                                                       |
//                                                                                                       |
//#define RESISTOR1                   10K
//#define RESISTOR2                   680E
                                                                                                         
uint8_t timer_working_mode          = 0;
uint8_t indicator_counter           = 0;

//boolean lid_closed                = false;
//#define PROCESSOR_VIN               3300.0
//#define VIN                         10200.0 //15000.0
//#define RESISTOR1                   10000.0 
//#define RESISTOR2                   3300.0
//
//#define BATTERY_DRAIN_VOLTAGE       9200
//
//#define RESISTOR_DIVIDER            (float)RESISTOR2 /(float)(RESISTOR1 + RESISTOR2)
//
//#define getVout(Vin)                (float)Vin*(RESISTOR_DIVIDER)
//#define getVin(Vout)                (float)Vout*((RESISTOR1 + RESISTOR2) /RESISTOR2)

#define LID_CLOSING_TIME_LIMIT      4*3600 //4 hours in seconds  //14400

uint8_t rx_frame_array[40];
uint8_t rx_frame_length             = 0;

uint8_t frame_array[40];
uint8_t frame_length                = 0;
#define START_BYTE                  0x52
//#define SLAVE_ID                    0x01
#define REQUEST_SERIAL              0x10

#define END_CR                      0x0D
#define END_LF                      0x0A


#define RESPONSE_OK                 0x01
#define RESPONSE_FAIL               0x00


#define ACTIVE_LOW                  0
#define ACTIVE_HIGH                 1

#define LID_CLOSE                   ACTIVE_LOW
#define LID_OPEN                    ACTIVE_HIGH


boolean check_led_blink_flag        = true;
boolean led_pulse_flag              = false;



//volatile bool lid_close_flag        = false;
unsigned char send_counter          = 0;
boolean timerEnable                 = false;

boolean lid_open_flag               = false;


uint8_t slave_id = 0;
uint8_t test_mode = 0;
