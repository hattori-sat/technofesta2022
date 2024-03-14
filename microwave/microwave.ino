/*
https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/

pulseInが重要な関数

60msecは時間を空けないといけない．

http://www.musashinodenpa.com/arduino/ref/index.php?f=0&pos=2450

ピンに入力されるパルスを検出します。たとえば、パルスの種類(value)をHIGHに指定した場合、pulseIn関数は入力がHIGHに変わると同時に時間の計測を始め、
またLOWに戻ったら、そこまでの時間(つまりパルスの長さ)をマイクロ秒単位で返します。タイムアウトを指定した場合は、その時間を超えた時点で0を返します。

この関数で計測可能な時間は、経験上、10マイクロ秒から3分です。あまりに長いパルスに対してはエラーとなる可能性があります。

【パラメータ】

pin: パルスを入力するピンの番号
value: 測定するパルスの種類。HIGHまたはLOW
timeout(省略可): タイムアウトまでの時間(単位・マイクロ秒)。デフォルトは1秒 (unsigned long)

【戻り値】

パルスの長さ(マイクロ秒)。パルスがスタートする前にタイムアウトとなった場合は0 (unsigned long)。

*/
const int trigPin = 5;
const int echoPin = 18;
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void setup() {
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop() {
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
  distanceCm = duration * SOUND_SPEED/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);
  
  delay(1000);
}