/*
参考資料：https://tekuteku-embedded.xyz/2022/09/15/adxl345/
内容が丁寧でわかりやすい．
他のプログラムでは失敗していたので，resolutionのせいではないかと考えている．
それか表示で上位bitにデータが入ってしまっていたか．
*/

#include <Arduino.h>
#include <Wire.h>
 
//device address
#define ADXL345_I2CADDR_DEFAULT (0x53)
 
//Register list
#define ADXL345_DEVID       0x00
#define ADXL345_THRESH_TAP  0x1d
#define ADXL345_OFSX        0x1e
#define ADXL345_OFSY        0x1f
#define ADXL345_OFSZ        0x20
#define ADXL345_DUR         0x21
 
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_BW_RATE     0x2c
#define ADXL345_POWER_CTL   0x2d
#define ADXL345_INT_SOURCE  0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32
#define ADXL345_DATAX1      0x33
#define ADXL345_DATAY0      0x34
#define ADXL345_DATAY1      0x35
#define ADXL345_DATAZ0      0x36
#define ADXL345_DATAZ1      0x37
#define ADXL345_FIFO_CTL    0x38
 
//Range Bits
typedef enum {
  Range2g   = 0b00,
  Range4g   = 0b01,
  Range8g   = 0b10,
  Range16g  = 0b11
} adxl345_range_t;
 
uint8_t buf[6];
float acc[3];
float kgain = 0.004;
 
//指定のアドレスに値を書き込む関数
void WriteReg(uint8_t addrs, uint8_t val)
{
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.write(val);
  Wire.endTransmission(); 
}
 
//指定のアドレスから値を読む関数(lengthで読み込むバイト数を指定する)
void ReadReg(uint8_t addrs,uint8_t * buf,uint8_t length)
{
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_I2CADDR_DEFAULT,length,true);
 
  for(uint8_t i =0;i<length;i++)
  {
  buf[i] = Wire.read();
  }
}
 
//2の補数計算(マイナス値を算出する）
int16_t two_complement(uint16_t val,uint8_t bits)
{
  int16_t val_out;
   
  if ((val) & ((uint16_t)1 << (bits - 1))) {
    val_out = val - (1U<<bits);
  }
  else{
    val_out = val;
  }
  return val_out;
}
void setup(){
  Serial.begin(115200);
  Wire.begin(21, 22); //I2C端子
  delay(3000);
 
  //測定モードON
  uint8_t val = 1 << 3U;
  WriteReg(ADXL345_POWER_CTL,val);
/*  
      //confirmation
        ReadReg(ADXL345_POWER_CTL,buf,1); 
        Serial.print("POWER_CTL:");
        Serial.println(buf[0]);
*/
  //FIFO OFF (Bypass設定）
  val = 0;
  WriteReg(ADXL345_FIFO_CTL,val);
/*  
      //confirmation   
        ReadReg(ADXL345_FIFO_CTL,buf,1); 
        Serial.print("FIFO_CTL:");
        Serial.println(buf[0]);
*/
 
  //測定レンジ±16g Full resolutionモード設定
  val = (uint8_t)Range16g | 1<<3U;
  WriteReg(ADXL345_DATA_FORMAT,val);
/*  
     //confirmation 
      ReadReg(ADXL345_DATA_FORMAT,buf,1); 
      Serial.print("DATA_FORMAT:");
      Serial.println(buf[0]); 
*/
}
void loop(){
  //データ6バイト分一気読み
  ReadReg(ADXL345_DATAX0,buf,6);
  //符号ビットを考慮
  int16_t temp = two_complement((buf[1]<<8U | buf[0])&0x1FFF,13);
  //物理値変換
  acc[0]=temp*kgain;
 
  temp = two_complement((buf[3]<<8U | buf[2])&0x1FFF,13);
  acc[1]=temp*kgain;
  temp = two_complement((buf[5]<<8U | buf[4])&0x1FFF,13);
  acc[2]=temp*kgain; 
 
  Serial.println("*********************************");
  Serial.print("accx = ");
  Serial.print(acc[0]);
  Serial.println(" g");
  Serial.print("accy = ");
  Serial.print(acc[1]);
  Serial.println(" g");
  Serial.print("accz = ");
  Serial.print(acc[2]);
  Serial.println(" g"); 
   
  delay(1000);
}