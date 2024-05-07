#include <avr/io.h>
#include <util/delay.h>
#define SET_BIT(REG_NAME, BIT_NUMBER) REG_NAME |= (1 << BIT_NUMBER)
#define CLEAR_BIT(REG_NAME, BIT_NUMBER) REG_NAME &= (~(1 << BIT_NUMBER))
#define TOGGLE_BIT(REG_NAME, BIT_NUMBER) REG_NAME ^= (1 << BIT_NUMBER)
#define READ_BIT(REG_NAME, BIT_NUM) (REG_NAME >> BIT_NUM) & 1

#define speedL 11
#define IN1 7
#define IN2 8
#define IN3 6
#define IN4 2
#define speedR 10
// #define turning_speed 80
int P, D, I = 0, previousError, PIDvalue;
int lsp, rsp;
int lfspeed = 40;
float Kp = 0.008;
float Kd = 0;
float Ki = 0.0;
int error;

void setup() {
  pinMode(speedL, OUTPUT);
  pinMode(speedR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.begin(9600);
}

void loop() {
  int sensor1 = analogRead(A1); //left
  int sensor2 = analogRead(A2);
  int sensor3 = analogRead(A3);
  int sensor4 = analogRead(A4);
  int sensor5 = analogRead(A5); //right
  while (analogRead(A5) >= 200 && analogRead(A1) >= 200 && analogRead(A2) >= 200 && analogRead(A3) >= 200 && analogRead(A4) >= 200) {
    int turn_speed = 40;
    error = previousError;
    calculatePID();

    lsp = turn_speed + PIDvalue;
    rsp = turn_speed - PIDvalue;

    if (lsp > 100) lsp = 100;
    if (lsp < 0) lsp = 0;
    if (rsp > 100) rsp = 100;
    if (rsp < 0) rsp = 0;
    analogWrite(speedL, lsp);
    analogWrite(speedR, rsp);
    
  Serial.print("lsp ");
  Serial.print(lsp);
  Serial.print("   ");
  Serial.print("rsp ");
  Serial.println(rsp);
  }
  calculateError(sensor1, sensor2, sensor3, sensor4, sensor5);
  calculatePID();
  motorPIDcontrol();
}

void calculateError(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5) {
    Serial.print("sensor1 ");
  Serial.print(sensor1);
  Serial.print("   ");
  Serial.print("sensor2 ");
  Serial.print(sensor2);
  Serial.print("sensor3 ");
  Serial.print(sensor3);
  Serial.print("   ");
  Serial.print("sensor4 ");
  Serial.print(sensor4);
  Serial.print("sensor5 ");
  Serial.println(sensor5);
  error = (sensor1 + sensor2) - (sensor5 + sensor4); //left-right
  //-ve error means go left
  //+ve error means go right
  if(error<0) //right>left
  error-=sensor3;
  else error+=sensor3;
  // if(sensor1<100 && sensor2>200 && sensor3>200 && sensor4>200 && sensor5>200) 
  // error+=error;
  // if (sensor1>200 && sensor2>200 && sensor3>200 && sensor4>200 && sensor5<100)
  // error+=error;
  
  Serial.print("error= ");
  Serial.println(error);
}
void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}
void motorPIDcontrol() {
  lsp = lfspeed + PIDvalue;
  rsp = lfspeed - PIDvalue;
  Serial.print("PIDvalue  ");
  Serial.print(PIDvalue);
  if (lsp > 80) lsp = 80;
  if (lsp < 0) lsp = 0;
  if (rsp > 80) rsp = 80;
  if (rsp < 0) rsp = 0;
  analogWrite(speedL, lsp);
  analogWrite(speedR, rsp);
  Serial.print("lsp ");
  Serial.print(lsp);
  Serial.print("   ");
  Serial.print("rsp ");
  Serial.println(rsp);
}