//Pin番号定義
#define motorEnabled 11
#define runbrake 10
#define motorDirection 9
#define pwm 5
#define encoder 1
//使っていないpin
#define AlarmReset 7
#define INT 8


#define WHEEL_DIAMETER 0.05 // m // モーターの直径
#define WHEEL_TREAD 0.25 // m // 車輪のトレッド幅
#define ENC_COUNTS 30 // エンコーダーのカウント数

// エンコーダーの状態
int encoderA = 0;
int encoderPos = 0;
int lastEncoderPos = 0; 
float forward = 0; 


// 現在の自己位置
float x = 0.0;
volatile int pulse = 0; //エンコーダで数えたパルス数
float cmd_vel = 0; // 速度を表す変数



void setup() {
  //----------------------------------------------------//
  //motor
  pinMode(motorEnabled, OUTPUT);     //H:START L:STOP
  pinMode(runbrake, OUTPUT);  //H:RUN L:BREAK(Instant stop)
  pinMode(motorDirection, OUTPUT);     //H:Right L:Left)  
  pinMode(pwm, OUTPUT);       //This pin is Analog pin. Output 0～5V.
  //最初はモータをストップさせておく
  digitalWrite(motorEnabled, HIGH);
  digitalWrite(runbrake, HIGH);

  //使っていないpin
  //アラーム系
  pinMode(AlarmReset, OUTPUT);
  digitalWrite(AlarmReset, HIGH);
  //モータの制御状況がわかるらしいが使わない
  digitalWrite(INT, HIGH);
  Serial.begin(9600); // シリアル通信を開始する
  attachInterrupt(encoder, RecognizeRotation, RISING); //FALLING);  //割り込み処理定義
}

//パルスの立下り数を検出
void RecognizeRotation(void){
  pulse++;
}

//--------------------------------------------------------------------------------------------------//
//cart_motor
//移動距離の計算
void estimatePosition(void){
  encoderPos=pulse;
  int encoderDiff = encoderPos - lastEncoderPos;
  float distance = forward * (encoderDiff * 2.0 * 3.14159 * (WHEEL_DIAMETER / 2.0)) / ENC_COUNTS;   // 車輪の移動距離を計算する
  x += distance; // 新しい自己位置を計算する
  lastEncoderPos = encoderPos;
  int int_x = int(x);
  //String str_x = String(int_x);
  Serial.println(int_x);
}


//モータ正回転
void Forward_motor(int vel){
  digitalWrite(motorEnabled,LOW);  //運転
  digitalWrite(runbrake,LOW);  //New circuit
  digitalWrite(motorDirection,HIGH);  //正転
  analogWrite(pwm, abs(vel)); //速度指令
}

//モータ逆回転
void Reverse_motor(int vel){
  digitalWrite(motorEnabled,LOW);  //運転
  digitalWrite(runbrake,LOW);  //New circuit
  digitalWrite(motorDirection,LOW);     //逆転
  analogWrite(pwm, abs(vel)); //速度指令
}

//モータ停止
void Stop_motor(void){
  digitalWrite(motorEnabled,HIGH);   //速度0
  digitalWrite(runbrake,HIGH);   
  analogWrite(pwm, 0); 
}

//モータ瞬時停止
void InstantStop_motor(void){
  //瞬時停止
  digitalWrite(runbrake,HIGH); 
  digitalWrite(motorEnabled,HIGH); 
  analogWrite(pwm, 0); //速度0
}


//速度制御
void Control_Motor(float cmd_vel){
  //正転
  if(cmd_vel>0 &&  cmd_vel<128){
    Forward_motor(cmd_vel);
    forward=1;
  }
  //逆転
  else if(cmd_vel<0 && cmd_vel>-127){
    Reverse_motor(cmd_vel);
    forward=-1;
  }
  //停止
  else{
    Stop_motor();
    forward=0;
  }
}


void recieve_cmd() {
  String inputString = ""; // 受信した文字列を格納する変数
  inputString = Serial.readStringUntil("\n");
  float cmd_vel = atof(inputString.c_str()); //文字列を数字に変換(double型)
  Control_Motor(cmd_vel);
  
}





//メインプログラム
void loop() {
  estimatePosition();
  if(Serial.available())
  {
    recieve_cmd();
  }
}
