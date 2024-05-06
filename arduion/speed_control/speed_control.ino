#include <L298N.h>//驅動器函數庫
#include <PID_v1.h>//pid控制器
byte LeftMotorEN = 3;//左馬達pwm腳
byte LeftMotorIN1 = 5;//左馬達IN1
byte LeftMotorIN2 = 4;//左馬達IN2(轉向控制)

byte RightMotorEN = 6;//右馬達pwm腳
byte RightMotorIN1 = 8;//右馬達IN1
byte RightMotorIN2 = 7;//右馬達IN2(轉向控制)

byte Left_EncoderA =9;//左霍爾編碼器A相
byte Left_EncoderB =10;//左霍爾編碼器B相

byte Right_EncoderA =12;//右爾編碼器A相
byte Right_EncoderB =11;//右霍爾編碼器B相
double out_left_speed = 0;
double out_right_speed=0;
double Left_turn = 0;
double Right_turn =0;
double P=300;
double I=1000;
double D=2;

double left_target_speed;//左目標轉速
double left_measured_speed;//左測量(實際)轉速Right_turn
double left_pwm;//左輸出pwm

double right_target_speed;//右目標轉速
double right_measured_speed;//右測量(實際)轉速
double right_pwm;//右輸出pwm

long Left_EncoderCount=0;//左編碼器記錄(用於算轉速)
long Right_EncoderCount=0;//右編碼器記錄(用於算轉速)
bool PinB[] = {0,0};
bool PinA[] = {0,0};
bool RPinB[] = {0,0};
bool RPinA[] = {0,0};
String input;
L298N LeftMotor(LeftMotorEN, LeftMotorIN1, LeftMotorIN2);//左馬達設定
L298N RightMotor(RightMotorEN, RightMotorIN1, RightMotorIN2);//右馬達設定
PID leftPID(&left_measured_speed, &left_pwm, &left_target_speed, P, I, D, DIRECT);//PID設定
PID rightPID(&right_measured_speed, &right_pwm, &right_target_speed, P, I, D, DIRECT);//PID設定
int out_pwm = 0;

void Left_Encoder(long &EncoderCount) {//左計算觸發次數(正轉為+ 反轉維-)
  PinB[1] = digitalRead(Left_EncoderA);
  PinA[1] = digitalRead(Left_EncoderB);
  if (PinB[0] != PinB[1]){
    if (PinB[1] == LOW) {
      if (PinA[1] == HIGH) {
        EncoderCount++;
      }
      else {
        EncoderCount--;
      }
    }
    PinB[0] = PinB[1];
  }
}
void Right_Encoder(long &EncoderCount) {//左計算觸發次數(正轉為+ 反轉維-)
  RPinB[1] = digitalRead(Right_EncoderA);
  RPinA[1] = digitalRead(Right_EncoderB);
  //Serial.print("EncoderCount:");
  //Serial.println(EncoderCount);
  if (RPinB[0] != RPinB[1]){
    if (RPinB[1] == LOW) {
      if (RPinA[1] == HIGH) {
        EncoderCount++;
      }
      else {
        EncoderCount--;
      }
    }
    RPinB[0] = RPinB[1];
  }
}


void left_speed(){//換算成轉速並賦歸
  out_left_speed = (static_cast<double>(Left_EncoderCount)/(0.05*1000));
  Left_EncoderCount = 0;

  LeftMotor.reset();
  /*
  Serial.print("left_speet:");
  Serial.print(left_measured_speed);
   Serial.print("\tleft_pwm:");
  Serial.print(left_pwm);
  Serial.print("\t set_speed:");
  Serial.print(left_target_speed);
  Serial.print("\n");
  */
  
  String a = String(out_left_speed) + " " + String(out_right_speed);
  Serial.println(a);
  LeftMotor.setSpeed(left_pwm);
  left_measured_speed = abs(out_left_speed);
  leftPID.Compute();
}

void right_speed(){//換算成轉速並賦歸
  out_right_speed  =  (static_cast<double>(Right_EncoderCount)/(0.05*1000));
  Right_EncoderCount = 0;
   if (right_measured_speed != 0 ){
  //Serial.print("Right_EncoderCount:");
  //Serial.print(Right_EncoderCount);
  //Serial.print("right speed:");
  //Serial.print(right_measured_speed);
  //Serial.print(" ")
  }
  right_measured_speed = abs(out_right_speed);
  RightMotor.reset();
  rightPID.Compute();
  RightMotor.setSpeed(right_pwm);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  delay(1000);//等待旋轉穩定
  LeftMotor.setSpeed(0);
  RightMotor.setSpeed(0);
  //Serial.print("go");
  
}
//PID leftPID(&left_measured_speed, &left_pwm, &left_target_speed, P, I, D, DIRECT);//PID設定
void loop() {
  // put your main code here, to run repeatedly:
  if ( Left_turn==1) LeftMotor.run(1);
  else LeftMotor.run(0);
  if ( Right_turn==1) RightMotor.run(1);
  else RightMotor.run(0);


  Left_Encoder(Left_EncoderCount);
  Right_Encoder(Right_EncoderCount);
  LeftMotor.forwardFor(50, left_speed);
  RightMotor.forwardFor(50, right_speed);
  //RightMotor.setSpeed(200);
  if (Serial.available() > 0) {
    input = Serial.readStringUntil(' '); // 讀取第一個數字
    if (input.toDouble()<0) Left_turn=1;
    else Left_turn =0;
    left_target_speed = abs(input.toDouble()); // 將讀取的字符串轉換為整數值

    input = Serial.readStringUntil('\n'); // 讀取第二個數字
    if (input.toDouble()<0) Right_turn=1;
    else Right_turn =0;
    right_target_speed = abs(input.toDouble()); // 將讀取的字符串轉換為整數值
  }

}
