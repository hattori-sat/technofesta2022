//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//bluetooth
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//超音波センサ
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
const int trigPin = 5;
const int echoPin = 18;
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
long duration;
float distanceCm;
float distanceInch;

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//モータドライバとモータ制御
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
#define PIN_IN1 25
#define PIN_IN2 26
#define PIN_VREF 4  // PWM
#define PIN_VREF_R 2
#define PIN_IN1_R 12
#define PIN_IN2_R 14

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//ADXK345
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
#include <Arduino.h>
#include <Wire.h>

//device address
#define ADXL345_I2CADDR_DEFAULT (0x53)

//Register list
#define ADXL345_DEVID 0x00
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38

//Range Bits
typedef enum {
  Range2g = 0b00,
  Range4g = 0b01,
  Range8g = 0b10,
  Range16g = 0b11
} adxl345_range_t;

uint8_t buf[6];
float acc[3];
float kgain = 0.004;

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//GYRO
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
#define GYRO 0x68        // gyro I2C address
#define REG_GYRO_X 0x1D  // IMU-3000 Register address for GYRO_XOUT_H
float ggain = 1/65.5;

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//エンコーダ
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
volatile int  encoder_cnt=0;
byte encoder_a = 34;

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//setup
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  /*bluetooth*/
  SerialBT.begin("ESP32test1");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  /*超音波*/
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  pinMode(encoder_a, INPUT_PULLUP);

  /*モータドライバとモータ制御*/
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcSetup(0, 7812.5, 8);  // 7812.5Hz, 8Bit(256段階)
  ledcAttachPin(PIN_VREF, 0);
  ledcWrite(0, 255);  //  50%(1.7V)
  pinMode(PIN_IN1_R, OUTPUT);
  pinMode(PIN_IN2_R, OUTPUT);
  ledcSetup(1, 7812.5, 8);  // 7812.5Hz, 8Bit(256段階)
  ledcAttachPin(PIN_VREF_R, 1);
  ledcWrite(1, 255);  //  50%(1.7V)

  /*ADXL345*/
  Wire.begin(21, 22);  //I2C端子
  delay(3000);

  //測定モードON
  uint8_t val = 1 << 3U;
  WriteReg(ADXL345_POWER_CTL, val);
  /*  
      //confirmation
        ReadReg(ADXL345_POWER_CTL,buf,1); 
        Serial.print("POWER_CTL:");
        Serial.println(buf[0]);
*/
  //FIFO OFF (Bypass設定）
  val = 0;
  WriteReg(ADXL345_FIFO_CTL, val);
  /*  
      //confirmation   
        ReadReg(ADXL345_FIFO_CTL,buf,1); 
        Serial.print("FIFO_CTL:");
        Serial.println(buf[0]);
*/

  //測定レンジ±16g Full resolutionモード設定
  val = (uint8_t)Range16g | 1 << 3U;
  WriteReg(ADXL345_DATA_FORMAT, val);
  /*  
     //confirmation 
      ReadReg(ADXL345_DATA_FORMAT,buf,1); 
      Serial.print("DATA_FORMAT:");
      Serial.println(buf[0]); 
*/

  /*GYRO*/
  WriteRegGyro(0x16, 0x0B);

  /*エンコーダ（片方だけにします）*/
  pinMode(encoder_a,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoder_a), encoder_pulse, RISING); 
}

//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
//loop関数
//HKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHKHK
void loop() {
  /*
  SerialBT.println("hello");
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  */
  /*モータ制御コマンドまち*/
  motor();
  delay(20);

  /*加速度*/
  //データ6バイト分一気読み
  ReadReg(ADXL345_DATAX0, buf, 6);
  //符号ビットを考慮
  int16_t temp = two_complement((buf[1] << 8U | buf[0]) & 0x1FFF, 13);
  //物理値変換
  acc[0] = temp * kgain;
  temp = two_complement((buf[3] << 8U | buf[2]) & 0x1FFF, 13);
  acc[1] = temp * kgain;
  temp = two_complement((buf[5] << 8U | buf[4]) & 0x1FFF, 13);
  acc[2] = temp * kgain;
  SerialBT.println("*********************************");
  SerialBT.print("accx = ");
  SerialBT.print(acc[0]-0.08);
  SerialBT.println(" g");
  SerialBT.print("accy = ");
  SerialBT.print(acc[1]-0.14);
  SerialBT.println(" g");
  SerialBT.print("accz = ");
  SerialBT.print(acc[2]+0.3);
  SerialBT.println(" g");

  delay(100);

  /*GYRO*/
  //WriteReg(0x16,0x0B);
  ReadRegGyro(REG_GYRO_X, buf, 6);
  temp = two_complement((buf[0] << 8U | buf[1]) & 0x1FFF, 13);
  //物理値変換
  acc[0] = temp * ggain;

  temp = two_complement((buf[2] << 8U | buf[3]) & 0x1FFF, 13);
  acc[1] = temp * ggain;
  temp = two_complement((buf[4] << 8U | buf[5]) & 0x1FFF, 13);
  acc[2] = temp * ggain;
  SerialBT.println("*********************************");
  SerialBT.print("gyrox = ");
  SerialBT.print(acc[0]+4.06);
  SerialBT.println(" °/s");
  SerialBT.print("gyroy = ");
  SerialBT.print(acc[1]-3.53);
  SerialBT.println(" °/s");
  SerialBT.print("gyroz = ");
  SerialBT.print(acc[2]+0.92);
  SerialBT.println(" °/s");
  delay(100);

  /*超音波*/
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  // Prints the distance in the SerialBT Monitor
  SerialBT.println("*********************************");
  SerialBT.print("Distance (cm): ");
  SerialBT.println(distanceCm);
  delay(50);

  /*エンコーダ*/
   SerialBT.println("*********************************");
  SerialBT.print("Encoder: ");
  SerialBT.println(encoder_cnt);
   encoder_cnt = 0;
   delay(1000);

  
}

void motor() {
  if (SerialBT.available()) {
    char mode = SerialBT.read();
    switch (mode) {
      case 0x30:
        fore();
        break;
      case 0x31:
        right();
        break;
      case 0x32:
        left();
        break;
      case 0x33:
        back();
        break;
    }
  }
}

void fore() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, LOW);
  delay(5000);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, HIGH);
}

void right() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN1_R, LOW);
  digitalWrite(PIN_IN2_R, HIGH);
  delay(5000);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, HIGH);
}

void left() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, LOW);
  delay(5000);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, HIGH);
}

void back() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, LOW);
  digitalWrite(PIN_IN2_R, HIGH);
  delay(4000);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN1_R, HIGH);
  digitalWrite(PIN_IN2_R, HIGH);
}

//指定のアドレスに値を書き込む関数
void WriteReg(uint8_t addrs, uint8_t val) {
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.write(val);
  Wire.endTransmission();
}

//指定のアドレスから値を読む関数(lengthで読み込むバイト数を指定する)
void ReadReg(uint8_t addrs, uint8_t* buf, uint8_t length) {
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_I2CADDR_DEFAULT, length, true);

  for (uint8_t i = 0; i < length; i++) {
    buf[i] = Wire.read();
  }
}

//2の補数計算(マイナス値を算出する）
int16_t two_complement(uint16_t val, uint8_t bits) {
  int16_t val_out;

  if ((val) & ((uint16_t)1 << (bits - 1))) {
    val_out = val - (1U << bits);
  } else {
    val_out = val;
  }
  return val_out;
}

void encoder_pulse() {
  encoder_cnt++;                    //エンコーダカウントをインクリメント
}

//指定のアドレスから値を読む関数(lengthで読み込むバイト数を指定する)
void ReadRegGyro(uint8_t addrs, uint8_t* buf, uint8_t length) {
  Wire.beginTransmission(GYRO);
  Wire.write(addrs);
  Wire.endTransmission();
  Wire.requestFrom(GYRO, length, true);

  for (uint8_t i = 0; i < length; i++) {
    buf[i] = Wire.read();
  }
}

//指定のアドレスに値を書き込む関数
void WriteRegGyro(uint8_t addrs, uint8_t val) {
  Wire.beginTransmission(GYRO);
  Wire.write(addrs);
  Wire.write(val);
  Wire.endTransmission();
}