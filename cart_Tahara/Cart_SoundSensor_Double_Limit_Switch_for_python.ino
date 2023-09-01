// https://github.com/NicoHood/PinChangeInterrupt のライブラリを使用
#include "PinChangeInterrupt.h"
// https://github.com/PaulStoffregen/MsTimer2　のライブラリを使用
#include <MsTimer2.h>

//Pin番号定義
#define start_pin 11
#define runbrake 10
#define cwccw 9
#define pwm 5
#define INT 8
#define AlarmReset 7
#define Speed 1
#define Bumper1 4
#define Bumper2 6
#define TRIG 12
#define ECHO 13

#define threshold 15.0 //2台間の距離の閾値

uint8_t count = 0;
char data[32];

//パラメータ
float r = 0.0250;           //車輪半径[m]
float rot = 0.0000;         //車輪の回転数[rot]
double distance_m_pre = 0;  //1サイクル前までの移動距離[m]
double distance_m_now = 0;  //現在の移動距離[m]
double distance_m = 0;      //スタート位置を基準とした移動距離[m]
int cart_direction = 1;     //現在の移動方向(左:1 右:-1 エラー:0)
int prev_direction = 1;     //1サイクル前の移動方向(左:1 右:-1 エラー:0)
volatile unsigned long pulse = 0;    //エンコーダで数えたパルス数
unsigned long total_left_pulse = 0;  //左へ移動したときの合計のpulse
unsigned long total_right_pulse = 0; //右へ移動したときの合計のpulse

// https://rb-station.com/blogs/article/hc-sr04-arduino
// https://haizairenmei.com/2018/10/27/arduino_noise/
float duration = 0.0;
float distance = 0.0;
float prev_distance= 0.0;
float speed_of_sound = 331.5 + 0.6 * 25; // 25℃の気温の想定
const int k = 0.8;
float rc = 0;

int pin_value;
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 1000; // デバウンス処理の待機時間
int stop_flag = 0; //ポーリング時に終了させる

float move_val;
float distance_cm;
int PWM = 30;
int direction = 0;

void setup() {

  pinMode(start_pin, OUTPUT);     //H:START L:STOP
  pinMode(runbrake, OUTPUT);  //H:RUN L:BREAK(Instant stop)
  pinMode(cwccw, OUTPUT);     //H:CW(Right) L:CCW(Left)
  pinMode(INT, OUTPUT);       //Don't use. Always LOW.
  pinMode(AlarmReset, OUTPUT);//Don't use. Always LOW.
  pinMode(pwm, OUTPUT);       //This pin is Analog pin. Output 0～5V.

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  digitalWrite(INT, HIGH);
  digitalWrite(AlarmReset, HIGH);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  //最初はモータをストップさせておく
  digitalWrite(start_pin, HIGH);
  digitalWrite(runbrake, HIGH);

  pinMode(Bumper1, INPUT_PULLUP);
  pinMode(Bumper2, INPUT_PULLUP);

  Serial.begin(115200);

  //割り込み処理定義
  attachInterrupt(Speed, RecognizeRotation, FALLING);
  // attachInterrupt(digitalPinToInterrupt(Bumper1), Lmt, FALLING);
  attachPinChangeInterrupt(digitalPinToPCINT(Bumper1), Bump1, FALLING);
  attachPinChangeInterrupt(digitalPinToPCINT(Bumper2), Bump2, FALLING);

  for(int i = 0;i < 100;i ++){
    digitalWrite(TRIG, LOW); 
    delayMicroseconds(2); 
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10); 
    digitalWrite(TRIG, LOW);
    duration = pulseIn(ECHO, HIGH); // 往復にかかった時間が返却される[マイクロ秒]

    if(duration > 0) {
      duration = duration / 2; // 往路にかかった時間
      distance = duration * speed_of_sound * 100 / 1000000;
      rc += distance / 100;  
    }
    delay(1);
  }
  MsTimer2::set(1000, sound);
  MsTimer2::start();
}

//パルスの立下り数を検出
void RecognizeRotation(void){
  pulse++;
}

void Bump1() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime >= debounceDelay) {
    lastInterruptTime = currentMillis;
    stop_flag = 1;
  }
}

void Bump2() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime >= debounceDelay) {
    lastInterruptTime = currentMillis;
    stop_flag = 2;
  }
}

void sound(){
  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10); 
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH); // 往復にかかった時間が返却される[マイクロ秒]

  if (duration > 0) {
    duration = duration / 2; // 往路にかかった時間
    distance = duration * speed_of_sound * 100 / 1000000;
    rc = k*rc + (1-k)*(float)distance;
    if(rc > 400.0){
      rc = 400.0;
    }
  }
  if(rc < threshold){
    stop_flag = 3;
  }
}

//移動距離の計算
void Calculate_distance(void){
  // rotはモータの回転数 
  rot = pulse / 450.0;  //パルス450回でモーターが1回転
  float distance_m = rot * (2.0 * 3.1416 * r); //rは車輪半径 2πr
  distance_cm = distance_m * 100; //単位を[m]から[cm]へ変換
}

//モータ正回転
void Forward_motor(int vel){
  //運転
  digitalWrite(start_pin,LOW);     //New circuit
  digitalWrite(runbrake,LOW);  //New circuit
  //正転
  digitalWrite(cwccw,HIGH);     //New circuit
  //速度
  analogWrite(pwm, abs(PWM));
}

//モータ逆回転
void Reverse_motor(int vel){
  //運転
  digitalWrite(start_pin,LOW);     //New circuit
  digitalWrite(runbrake,LOW);  //New circuit
  //逆転
  digitalWrite(cwccw,LOW);      //New circuit
  //速度
  analogWrite(pwm, abs(PWM));
}

//モータ停止
void Stop_motor(void){
  //停止
  digitalWrite(start_pin,HIGH);      //New circuit
  digitalWrite(runbrake,HIGH);   //New circuit
  //速度
  analogWrite(pwm, 0);
}

/* 
 * 停止は減速の立下りが加速の立上りとほぼ同じ挙動をする
 * そのため、速度-時間曲線は等脚台形になる(ハズ)
 * それに対し瞬時停止はほぼ90度の角度で立下る
 * 挙動についてはモータの取扱説明書または引継資料を参照のこと
 */

//速度制御
void Control_Motor(void){
  distance_cm = 0.0;
  pulse = 0.0;

  // 進行方向
  if (direction == -1) {
    while(move_val > distance_cm){
        Forward_motor(PWM);
        Calculate_distance();
        // sound();
        if(stop_flag == 1 || stop_flag == 2 || stop_flag == 3){
          break;
        }
    }
      Stop_motor();
      Calculate_distance();
  }

  // 逆方向
  if (direction == 1) {
    while(move_val > distance_cm){
      Reverse_motor(PWM);
      Calculate_distance();
      // sound();
      if(stop_flag == 1 || stop_flag == 2 || stop_flag == 3){
          break;
        }
    }
      Stop_motor();
      Calculate_distance();
  }
}

float Conversion(int num1, int num2, int num3, int num4){
  // 目標位置計算  
  // 百の位
  int Num1 = num1 - '0';      
  if (Num1 > 0 && Num1 < 9){
    Num1 = Num1*100;           
  } else Num1 = 0;
  
  // 十の位
  int Num2 = num2 - '0';
  if (Num2 > 0 && Num2 < 10){
    Num2 = Num2*10;            
  } else Num2 = 0;

  // 一の位
  int Num3 = num3 - '0';
  if (Num3 > 0 && Num3 < 10){
    Num3 = Num3*1;            
  } else Num3 = 0;

  // 1/10の位
  int Num4 = num4 - '0';
  if (Num4 > 0 && Num4 < 10){
    Num4 = Num4*1;            
  } else Num4 = 0;

  float Num = Num1 + Num2 + Num3 + Num4*0.1; // 合計

  return Num;
}

// //メインプログラム
// void loop() {
// //  if(Serial.available() > 0)
//   if(Serial.available())
//   {
//     prev_direction = cart_direction;
//     receive_data = Serial.read();
//     Control_Motor();
//     Send_Data();
//   }
// }

void loop() {
  if(Serial.available()){
    data[count] = Serial.read();
    if(data[count] == ','){

      move_val = Conversion(data[1], data[2], data[3], data[4]);
      int pin_value = digitalRead(Bumper1);

      if(data[0] == 'F'){
        direction = -1;
        // 安全防止策
        // if(pin_value == 0){
        //   direction = 1;
        // }
      }else{
        direction = 1;
        // if(pin_value == 0){
        //   direction = -1;
        // }
      }

      Control_Motor();
      if(stop_flag == 1){
        Serial.print("1(Bump),");Serial.print(distance_cm);Serial.print(",");Serial.println(rc);
      }else if(stop_flag == 2){
        Serial.print("2(Bump),");Serial.print(distance_cm);Serial.print(",");Serial.println(rc);
      }else if(stop_flag == 3){
        Serial.print("3(Sound),");Serial.print(distance_cm);Serial.print(",");Serial.println(rc);
      }else{
        Serial.print("0(Fin),");Serial.print(distance_cm);Serial.print(",");Serial.println(rc);
      }
      stop_flag = 0;
      count = 0;
    }else{
      count ++;
    }
  }
}
