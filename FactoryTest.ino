#include <M5Stack.h>
#include <Keypad.h>
#include <FastLED.h>
#include <driver/ledc.h>
#include "MFRC522_I2C.h"
#include "SHTSensor.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "qmp6988.h"   // @IAMLIUBO  2021-07-20
#include "UNIT_ENV.h"

int count_flag = 3;
uint8_t setup_flag = 0;
bool RFID_Flag=false;

void joystick(void) {
  M5.Lcd.setCursor(100, 0, 4);
  M5.Lcd.println("JOYSTICK");
  M5.Lcd.setCursor(0, 30 ,4);
  M5.Lcd.println("PIN: X_ADC    35\n        \t  Y_ADC    36\n        \t  BUTTON  2");

  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_35);
    gpio_reset_pin(GPIO_NUM_36);
    pinMode(2, INPUT_PULLUP);
  }
  M5.Lcd.setCursor(10, 140, 4);               //x_data          y_data          button_data
  M5.Lcd.printf("X:%04d  Y:%04d  BUTTON:%d\n", analogRead(35), analogRead(36), digitalRead(2));
  delay(200);
}

//DAC
#define DAC_ADDR 0x4C

void outVoltage(uint8_t ch,uint16_t v){
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x10|(ch<<1));
  Wire.write((v >> 2) & 0xff);
  Wire.write((v << 6) & 0xff);
  Wire.endTransmission();
}
void dac(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_21);
    gpio_reset_pin(GPIO_NUM_22);
    Wire.begin();

    M5.Lcd.setCursor(120, 0, 4);
    M5.Lcd.print("DAC");
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.println("PIN: SCL:22\n          SDA:21");
    M5.Lcd.setCursor(0, 125, 4);
    M5.Lcd.println("CH0:1.25v  CH1:2.50v\nCH2:3.75v  CH3:5.00v");
    // outVoltage(0,256);   //1.25v
    // outVoltage(1,512);   //2.50v
    // outVoltage(2,768);   //3.75v
    // outVoltage(3,1023);  //5.00v
  }
  outVoltage(0,256);   //1.25v
  outVoltage(1,512);   //2.50v
  outVoltage(2,768);   //3.75v
  outVoltage(3,1023);  //5.00v
}

//ADC
#define ADC_ADDR 0x48

uint16_t InVoltage(uint8_t ch){
  uint8_t  data_L = 0;
  uint8_t  data_H = 0;
  uint16_t data_adc = 0;
  Wire.beginTransmission(ADC_ADDR);
  Wire.write(0X01);
  Wire.write(0XC0 | (ch << 4)); 
  Wire.write(0X83);
  Wire.endTransmission();

  Wire.beginTransmission(ADC_ADDR);
  Wire.write(0x00); 
  Wire.endTransmission();

  delay(50);

  Wire.requestFrom(ADC_ADDR, 2);
  while(Wire.available()){
    data_H = Wire.read();
    data_L = Wire.read();
  }

  data_adc = (data_H << 8) | data_L;
  return data_adc;
}

void adc(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_21);
    gpio_reset_pin(GPIO_NUM_22);
    Wire.begin();

    M5.Lcd.setCursor(140, 0, 4);
    M5.Lcd.print("ADC");
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.println("PIN: SCL:22\n          SDA:21\n L:GND R:5v/3.3v To CH0~3");
  }

  M5.Lcd.setCursor(0, 125, 4);
  M5.Lcd.printf(" CH0:%05d CH1:%05d\n", InVoltage(0), InVoltage(1)); //adc_ch0、1
  M5.Lcd.printf(" CH2:%05d CH3:%05d\n", InVoltage(2), InVoltage(3)); //adc_ch2、3
  //delay(500);
}

//Encoder
const int phaseA = 2;
const int phaseB = 13;
const int Button = 15;
#define GET_CODE() uint8_t(digitalRead(phaseA) << 4 | digitalRead(phaseB))
int32_t count = 65536,count_last = 65536,count_change = 0;
uint8_t code = 0,code_old = 0;

void encoder(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_reset_pin(GPIO_NUM_15);
    pinMode(phaseA, INPUT_PULLUP);
    pinMode(phaseB, INPUT_PULLUP);
    pinMode(Button, INPUT_PULLUP);

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("ENCODER");
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.println("PIN: A:2\n          B:13\n          Button:15");

    code = GET_CODE();
    code_old = code;
  }

  uint8_t value = digitalRead(Button);
  code = GET_CODE();
  if(code != code_old) {
    if(code == 0x00) {
      count_last = count;
      if(code_old == 0x10) {
        count--;
        count_change == -65536 ? count_change : count_change--;
      } else {
        count_change == 65536 ? count_change : count_change++;
      }
    }
    code_old = code;
  }
  M5.Lcd.setCursor(0, 125, 4);
  M5.Lcd.printf("Angle = %05d\nButton = %d",count_change,value);
  delay(1);
}

//matrix
#define DATA_PIN    15
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];
#define BRIGHTNESS  5

int led_count = 0;
void matrix(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_15);

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("MATRIX");
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.println("PIN: SIGNAL:15");

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    //NEOPIXEL
    FastLED.setBrightness(BRIGHTNESS);
    led_count = 0;
  }
  led_count++;
  if(led_count > 2000)led_count  = 0;
  if((led_count / 500)  == 0){
    for(int i = 0; i < 64; i++)
    leds[i] = CRGB::Red;
    FastLED.show();
  }else if((led_count / 500)  == 1){
    for(int i = 0; i < 64; i++)
    leds[i] = CRGB::Green;
    FastLED.show();
  }else if((led_count / 500)  == 2){
    for(int i = 0; i < 64; i++)
    leds[i] = CRGB::Blue;
    FastLED.show();
  }
}
/*
//keyboard
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns

byte rowPins[ROWS] = {17,16,21,22};
byte colPins[COLS] = {5, 26, 13, 15};

char keys[ROWS][COLS] = {
  {'A','B','C','D'},
  {'E','F','G','H'},
  {'I','J','K','L'},
  {'M','N','O','P'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
//Keypad keypad;
void key(){
  if(!setup_flag){
    setup_flag = 1;
    //Wire.end();

    //Wire.end();
    //pinMode(21,INPUT_PULLUP);
    //pinMode(22,INPUT_PULLUP);
    gpio_reset_pin(GPIO_NUM_17);
    gpio_reset_pin(GPIO_NUM_16);
    gpio_reset_pin(GPIO_NUM_21);
    gpio_reset_pin(GPIO_NUM_22);
    gpio_reset_pin(GPIO_NUM_5);
    gpio_reset_pin(GPIO_NUM_26);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_reset_pin(GPIO_NUM_15);

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("KEYBOARD");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN: R0:17,R1:16,R2:21,R3:22");
    M5.Lcd.println("PIN: C0: 5,C1:26,C2:13,C3:15");
    keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
    
  }

  char key = keypad.getKey();
  if(key){
    Serial.println(key);
    M5.Lcd.fillRect(150, 150, 80, 50, BLACK);
    M5.Lcd.setCursor(150, 140, 4);
    M5.Lcd.printf("%c",key);
  }
}

//DC-Motor
int dc_count = 0;
void dcmotor(){
   if(!setup_flag){
    setup_flag = 1; 

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("DC-MOTOR");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN : M+:13,M-:15,ENA:21,\nENB:22");

    pinMode(13, OUTPUT);
    pinMode(15, OUTPUT);
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    dc_count = 0;
   }

   dc_count++;
   if(dc_count > 4000)dc_count  = 0;

   digitalWrite(13, LOW);
   digitalWrite(15, HIGH);
}

//Relay
const int In_0 = 13;
int relay_count= 0;
void relay(void){
  if(!setup_flag){
    setup_flag = 1; 
    M5.Lcd.setCursor(100, 30, 4);
    M5.Lcd.println("RELAY * 8");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN: 13 To CH*");
    pinMode(In_0,OUTPUT);
    digitalWrite(In_0, LOW);
    relay_count= 0;
    digitalWrite(In_0, HIGH);
  }
  relay_count++;
  if(relay_count > 2000)relay_count  = 0;
  //if(relay_count == 1)
  //digitalWrite(In_0, LOW);
  //if(relay_count == 1000)
  digitalWrite(In_0, HIGH);
}

//Servo
int servo_count= 0;
int freq = 50;
int ledChannel = 0;
int resolution = 8;
void servo(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_15);

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("SERVO");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN: 15\nConnect to Power");
     
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(15, ledChannel);
    servo_count= 0;
  }

  servo_count++;
   if(servo_count > 3000)servo_count  = 0;
   if((servo_count / 1000)  == 0){
     ledcWrite(ledChannel, 6);//0°
   }
   if((servo_count / 1000)  == 1){
     ledcWrite(ledChannel, 18);//0°
   }

   if((servo_count / 1000)  == 2){
     ledcWrite(ledChannel, 30);//0°
   }
}

//StepMotor 
const int MOTOR_A = 2;
const int MOTOR_B = 5;
const int MOTOR_C = 12;
const int MOTOR_D = 13;
void stmpmotor(void){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_5);
    gpio_reset_pin(GPIO_NUM_12);
    gpio_reset_pin(GPIO_NUM_13);

    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.println("STEPMOTOR");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN :A:2, B:5, C:12, D:13");
    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(MOTOR_C, OUTPUT);
    pinMode(MOTOR_D, OUTPUT);
  }

   digitalWrite(MOTOR_A, HIGH);
   digitalWrite(MOTOR_B, HIGH);
   digitalWrite(MOTOR_C, LOW);
   digitalWrite(MOTOR_D, LOW);
   delay(2);
   digitalWrite(MOTOR_A, LOW);
   digitalWrite(MOTOR_B, HIGH);
   digitalWrite(MOTOR_C, HIGH);
   digitalWrite(MOTOR_D, LOW);
   delay(2);

   digitalWrite(MOTOR_A, LOW);
   digitalWrite(MOTOR_B, LOW);
   digitalWrite(MOTOR_C, HIGH);
   digitalWrite(MOTOR_D, HIGH);
   delay(2);
   digitalWrite(MOTOR_A, HIGH);
   digitalWrite(MOTOR_B, LOW);
   digitalWrite(MOTOR_C, LOW);
   digitalWrite(MOTOR_D, HIGH);
   delay(2);
}
MFRC522 mfrc522(0x28); 
void ShowReaderDetails() {
  // Get the MFRC522 software version
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.print(F("MFRC522 Software Version: 0x"));
  Serial.print(v, HEX);
  if (v == 0x91)
    Serial.print(F(" = v1.0"));
  else if (v == 0x92)
    Serial.print(F(" = v2.0"));
  else
    Serial.print(F(" (unknown)"));
  Serial.println("");
  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF)) {
    Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
  }
}
void rfid(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_22);
    gpio_reset_pin(GPIO_NUM_21);
    Wire.begin();                   // Initialize I2C

    M5.Lcd.setCursor(140, 0, 4);
    M5.Lcd.print("RFID");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN : SCL:22,SDA:21");
    // M5.Lcd.setCursor(127, 210, 4);
    // M5.Lcd.println("Enter");

    //if(RFID_Flag){
      mfrc522.PCD_Init();             // Init MFRC522
      ShowReaderDetails();            // Show details of PCD - MFRC522 Card Reader details
      //RFID_Flag=false;
    //}
    //Serial.println(F("Scan PICC to see UID, type, and data blocks..."));
   //M5.Lcd.println("Scan PICC to see UID, type, and data blocks...");
  }
  /*if (!mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial()) {
    //delay(50);
    //M5.Lcd.println("Scan PICC to see UID, type, and data blocks...");
    //return;
    
  }

  if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {
    delay(50);
    return;
  }
  
  // Now a card is selected. The UID and SAK is in mfrc522.uid.
  
  // Dump UID
  M5.Lcd.println(" ");
  M5.Lcd.setCursor(0, 90, 4);
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    M5.Lcd.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    M5.Lcd.print(mfrc522.uid.uidByte[i], HEX);
  } 
  Serial.println();
}


//TEMPERATURE
SHT3X sht30;
void dht(){
  float tmp = 0.0;
  float hum = 0.0;
  if(!setup_flag){
    setup_flag = 1;
    Wire.begin();
    M5.Lcd.setCursor(100, 0, 4);
    M5.Lcd.print("TEMPERATURE");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN : SCL:22,SDA:21");
  }

  // Wire.beginTransmission(addr);
  // uint8_t error = Wire.endTransmission();
  // if(error !=0) {
  //   meiyou
  // }

  if(sht30.get()==0){
    tmp = sht30.cTemp;
    hum = sht30.humidity;
  }
  M5.Lcd.setCursor(0, 100, 4);
  M5.Lcd.printf("Temp: %2.1f  \r\nHumi: %2.0f%%\r\n", tmp, hum);
  delay(200);
}

//Adafruit_BMP280 bme;/   // @IAMLIUBO   2021-07-20
QMP6988 qmp6988;          // @IAMLIUBO   2021-07-20

void air(){
   if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_22);
    gpio_reset_pin(GPIO_NUM_21);
    Wire.begin();

    qmp6988.init();   // @IAMLIUBO   2021-07-20
    M5.Lcd.setCursor(80, 0, 4);
    M5.Lcd.print("AIR_PRESSURE");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN : SCL:22,SDA:21");

//    while(!bme.begin(0x76)){  
//     Serial.println("Could not find a valid BMP280 sensor, check wiring!");
//    }
  }

//  float pressure = bme.readPressure();   // @IAMLIUBO   2021-07-20
  float pressure = qmp6988.calcPressure(); // @IAMLIUBO   2021-07-20
  M5.Lcd.setCursor(0, 100, 4);
  M5.Lcd.printf("Pressure:%2.0fPa\r\n",pressure);
  delay(100);
}

const int Analog = 35;
const int Digtal = 2;
uint16_t a_data;
uint16_t d_data;
void luminosity(){
  if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_35);
    gpio_reset_pin(GPIO_NUM_2);

    M5.Lcd.setCursor(80, 0, 4);
    if(count_flag == 7)
    M5.Lcd.print("LUMINOSITY");
    else 
    M5.Lcd.print("MICROPHONE");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN: Analog:35,Digtal:2");

    pinMode(Digtal, INPUT_PULLUP);
  }

  a_data = analogRead(Analog);
  d_data = digitalRead(Digtal);

  Serial.printf("Analog:%0d Digtal:%0d\n", a_data, d_data);

  M5.Lcd.setCursor(30, 120, 4);
  M5.Lcd.printf("Analog:%0d Digtal:%0d\n", a_data, d_data);
  
  delay(50);
  
}
int rx_num = 0;
int rx_count = 0;
void uart232(){
    if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_16);
    gpio_reset_pin(GPIO_NUM_17);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    M5.Lcd.setCursor(80, 30, 4);
    M5.Lcd.print("RS232");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN :TX:17,RX:16");
    rx_count = 0;
    rx_num = 0;
   }

   if(Serial2.available()) {
    int ch = Serial2.read();
    //Serial.write(ch);
    if(ch == 'a'){
      rx_num++;
    }
    M5.Lcd.setCursor(30, 120, 4);
    M5.Lcd.printf("rx_num = %d\n", rx_num);
  }
  rx_count++;
  if(rx_count > 100)rx_count = 0;
  if(rx_count == 10)
    Serial2.write('a');
}

//int rx_num = 0;
//int rx_count = 0;
void uart485(){
    if(!setup_flag){
    setup_flag = 1;
    gpio_reset_pin(GPIO_NUM_16);
    gpio_reset_pin(GPIO_NUM_17);
    gpio_reset_pin(GPIO_NUM_21);
    gpio_reset_pin(GPIO_NUM_22);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    Serial1.begin(115200, SERIAL_8N1, 22, 21);
    M5.Lcd.setCursor(80, 0, 4);
    M5.Lcd.print("RS485");
    M5.Lcd.setCursor(0, 60, 4);
    M5.Lcd.println("PIN :TX:17,RX:16");
    rx_count = 0;
    rx_num = 0;
   }

   if(Serial1.available()) {
    int ch = Serial1.read();
    //Serial.write(ch);
    if(ch == 'a'){
      rx_num++;
    }
    M5.Lcd.setCursor(30, 120, 4);
    M5.Lcd.printf("rx_num = %d\n", rx_num);
  }
  rx_count++;
  if(rx_count > 100)rx_count = 0;
  if(rx_count == 10)
    Serial2.write('a');
}
*/
void setup() {
  M5.begin();
  dacWrite(25, 0);

  // M5.Lcd.setCursor(90, 0, 4);
  // M5.Lcd.print("FactoryTest");

  M5.Lcd.setCursor(220, 210, 4);
  M5.Lcd.println("mode");

  M5.Lcd.setCursor(40, 210, 4);
  M5.Lcd.println("Back");
  M5.Lcd.drawLine(0,120,320,120,YELLOW);
}

void loop() {
  M5.update();
  if(M5.BtnA.isPressed()){
    count_flag--;
    setup_flag = 0;
   M5.Lcd.fillRect(0, 0, 360, 210, BLACK);
    while(digitalRead(39) == LOW);
  }else if(M5.BtnB.isPressed()){
    RFID_Flag=true;
    while(digitalRead(38) == LOW);
  }else if(M5.BtnC.isPressed()){
   count_flag++;
   setup_flag = 0;
   M5.Lcd.fillRect(0, 0, 360, 210, BLACK);
   while(digitalRead(37) == LOW);
  }
  if(count_flag <= 0) count_flag = 0;
  else if(count_flag > 16) count_flag = 0;
  M5.Lcd.drawLine(0,120,320,120,YELLOW);
  M5.Lcd.setCursor(0, 0, 4);
  M5.Lcd.println(count_flag+1);

  switch(count_flag){
    case 0:joystick();
    break;
    case 1:dac();
    break;
     case 2:adc();
    break;
    case 3:encoder();
    break;
    case 4:matrix();
    break;
 /*   case 5:dht();
    break;
    case 6:air();
    break;
    case 7:luminosity();
    break;
    case 8:key();
    break;
    case 9:dcmotor();
    break;
    case 10:relay();
    break;
    case 11:servo();
    break;
    case 12:stmpmotor();
    //M5.Lcd.fillRect(120,210,70,40,BLACK);
    break;
    case 13:rfid();
    break;
    case 14:uart232();
    //M5.Lcd.fillRect(120,210,70,40,BLACK);
    break;
    case 15:uart485();
    break;
    case 16:luminosity(); //microphone
    break; */
    default:break;
  }
}
