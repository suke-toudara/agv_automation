#include "MotorController.h"
#include <Arduino.h>
#include <Servo.h>

#define PIN_MOTOR_L 4
#define PIN_MOTOR_R 5
#define PIN_ENCODER_L_A 3
#define PIN_ENCODER_L_B 19
#define PIN_ENCODER_R_A 2
#define PIN_ENCODER_R_B 18

unsigned long current_time = 0, prev_time_10ms = 0, prev_time_100ms;
String reciev_str, send_str;
#define MOTOR_NUM 2
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
MotorController motor_controllers[2];
const float L_KP = 1.5;  //CuGoV3
const float L_KI = 0.02; //CuGoV3
const float L_KD = 0.1;  //CuGoV3
const float R_KP = 1.5;  //CuGoV3
const float R_KI = 0.02; //CuGoV3
const float R_KD = 0.1;  //CuGoV3
const float L_LPF = 0.95;
const float R_LPF = 0.95;
// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;

/*serial_param*/
char header = 'S';
char footer = 'E';
String receivedData = ""; 
boolean receivingData = false;
int sp_reciev_str[2];

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

  delay(10);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);
  // 外部割込みピンは直接レジスタをたたく。（Arduinoコンパイラの場合、同じブロックであるD4~D7も反応してしまう）

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  motor_direct_instructions(1500, 1500);
  delay(100); // 10msでもOKでした。確実に回すために設定（気持ち）
}

void leftEncHandler() {
  motor_controllers[MOTOR_LEFT].updateEnc();
}

void rightEncHandler() {
  motor_controllers[MOTOR_RIGHT].updateEnc();
}


void motor_direct_instructions(int left, int right)
{
  motor_controllers[MOTOR_LEFT].servo_.writeMicroseconds(left);
  motor_controllers[MOTOR_RIGHT].servo_.writeMicroseconds(right);
}


void send_speed()
{

    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。開発用
    //Serial.print(motor_controllers[MOTOR_LEFT].rpm_); // 制御量を見るため。開発用
    //Serial.print("\n");
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。開発用
    //Serial.println(motor_controllers[MOTOR_RIGHT].rpm_);    
    //Serial.print("0.0");  // カウンタが見たいだけ。あとで戻す

}

void job_10ms()
{
  for (int i = 0; i < MOTOR_NUM; i++) { 
    motor_controllers[i].driveMotor();
  }

}

void job_100ms()
{
  send_speed();
}

void set_motor_cmd()
{
  for (int i = 0; i < MOTOR_NUM; i++) {
    Serial.print(sp_reciev_str[i]);
    motor_controllers[i].setTargetRpm(float(sp_reciev_str[i]));
  } 
}

void loop()
{
  current_time = micros();  // オーバーフローまで約40分

  // 10msJob:drive_motor,calc_rpm
  if (current_time - prev_time_10ms > 10000) { // TODO 10秒で1msくらいズレる
    job_10ms();
    prev_time_10ms = current_time;
  }
  // 100msJob:print_rpm
  //if (current_time - prev_time_100ms > 100000) { // TODO 1秒で1msくらいズレる
    //job_100ms();
    //prev_time_100ms = current_time;
  //}

  if (Serial.available() > 0) {
     char receivedChar = Serial.read();
     if (receivedChar == header) {
        receivedData = "";
        receivingData = true;
     } else if (receivedChar == footer && receivingData) {
      if(receivedData.length() > 0){
        int _ = sscanf(receivedData.c_str(), "%d,%d", &sp_reciev_str[0], &sp_reciev_str[1]);
        set_motor_cmd();
        receivingData = false;
        receivedData = "";
      }else{
        for (int i = 0; i < MOTOR_NUM; i++) {
          motor_controllers[i].setTargetRpm(0.0);
        }
      }
     } else if (receivingData){
      receivedData += receivedChar;
     }
  }
 }
