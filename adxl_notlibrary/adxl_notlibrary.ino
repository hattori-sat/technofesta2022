#include <Wire.h>
#define DEVICE (0x53)  //ADXL345 device address
#define TO_READ (6)    //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ];  //6 bytes buffer for saving data read from the device

char str[512];          //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;  //first axis-acceleration-data register on the ADXL345
int x, y, z;            //three axis acceleration data

double roll = 0.00, pitch = 0.00;  //Roll & Pitch are the angles which rotate by the axis X and y

//in the sequence of R(x-y-z),more info visit
// https://www.dfrobot.com/wiki/index.php?title=How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing#Introduction

void setup() {
  Wire.begin();          // join i2c bus (address optional for master)
  Serial.begin(57600);  // start serial for output
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  writeTo(DEVICE, 0x31, 0);
}
void loop() {
  readFrom(DEVICE, regAddress, TO_READ, buff);  //read the acceleration data from the ADXL345
  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];
  y = (((int)buff[3]) << 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  //we send the x y z values as a string to the serial port
  Serial.print("The acceleration info of x, y, z are:");
  //sprintf(str, "%d %d %d", x, y, z);
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  //Serial.print(str);
  Serial.write(10);
  //Roll & Pitch calculate
  RP_calculate();
  Serial.print("Roll:");
  Serial.println(roll);
  Serial.print("Pitch:");
  Serial.println(pitch);
  Serial.println("");
  //It appears that delay is needed in order not to clog the port
  delay(500);
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device);  //start transmission to device
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();          //end transmission
}
//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  //指定したアドレスのI2Cスレーブに対して送信処理を始めます。この関数の実行後、write()でデータをキューへ送り、endTransmission()で送信を実行します。
  Wire.beginTransmission(device);  //start transmission to device
  //スレーブデバイスがマスタからのリクエストに応じてデータを送信するときと、マスタがスレーブに送信するデータをキューに入れるときに使用します。beginTransmission()とendTransmission()の間で実行します。
  Wire.write(address);             //sends address to read from
  Wire.endTransmission();          //end transmission
  Wire.beginTransmission(device);  //start transmission to device
  Wire.requestFrom(device, num);   // request 6 bytes from device
  /*
  他のデバイスにデータを要求します。そのデータはavailable()とreceive()関数を使って取得します。
  Arduino1.0.1で3つ目のパラメータ(省略可)が追加され、一部のI2Cデバイスとの互換性が高まりました。
  【パラメータ】
  address: データを要求するデバイスのアドレス(7ビット)
  quantity: 要求するデータのバイト数
  stop(省略可): trueに設定するとstopメッセージをリクエストのあと送信し、I2Cバスを開放します(デフォルト)。falseに設定するとrestartメッセージをリクエストのあと送信し、バスを開放しないことで他のマスタデバイスがメッセージ間にリクエストを出すのを防ぎます。
  */
  int i = 0;
  while (Wire.available())  //device may send less than requested (abnormal)
  {
    buff[i] = Wire.read();  // receive a byte
    i++;
  }
  Wire.endTransmission();  //end transmissio
}
//calculate the Roll&Pitch
void RP_calculate() {
  double x_Buff = float(x);
  double y_Buff = float(y);
  double z_Buff = float(z);
  roll = atan2(y_Buff, z_Buff) * 57.3;
  pitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
}