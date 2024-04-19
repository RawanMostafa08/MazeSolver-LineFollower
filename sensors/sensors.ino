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

//#define turning_speed 80

int P, D, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 80;
float Kp = 0.083;
float Kd = 0.23;
float Ki = 0;
char data;
int direction; //0-->left  1-->straight  2-->right
void adjustMotors(int dir);
int setDir(int error,int sensor3);

void setup() {
  direction = 1;
  previousError = 0;
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
  linefollow();
}

void linefollow() {
  int sensor1 = digitalRead(A1);
  int sensor2 = digitalRead(A2);
  int sensor3 = digitalRead(A3);
  int sensor4 = digitalRead(A4);
  int sensor5 = digitalRead(A5);
  //    while( digitalRead(A1)==1 && digitalRead(A2) ==1 && digitalRead(A3) ==1 && digitalRead(A4) ==1 && digitalRead(A5) ==1) {
  //     int turn_speed = 120;
  //     error = previousError;
  //     P = error;
  //     D = error - previousError;

  //     PIDvalue = (Kp * P) + (Kd * D);
  //     previousError = error;

  //     lsp = turn_speed + PIDvalue;
  //     rsp = turn_speed - PIDvalue;

  //     if (lsp > 255) lsp = 255;
  //     if (lsp < 0)   lsp = 0;
  //     if (rsp > 255) rsp = 255;
  //     if (rsp < 0)   rsp = 0;
  //     analogWrite(speedL, lsp);
  //     analogWrite(speedR, rsp);
  //   }
  // Serial.println("sensor1: " + String(sensor1) + ", sensor2: " + String(sensor2) + ", sensor3: " + String(sensor3) + ", sensor4: " + String(sensor4) + ", sensor5: " + String(sensor5));

  //1-->white    0-->black
  error = (sensor1 + sensor2) - (sensor5 + sensor4);  //left-right
  direction=setDir(error,sensor3);
  // Serial.print("direction loop");
  // Serial.println(direction);
  adjustMotors(direction);

  // _delay_ms(500);


  //   static int I = 0;
  //   I += error;
  //   P = error;
  //   D = error - previousError;


  //   PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  //   previousError = error;

  //   lsp = lfspeed + PIDvalue;
  //   rsp = lfspeed - PIDvalue;

  //   if (lsp > 255) lsp = 255;
  //   if (lsp < 0)   lsp = 0;
  //   if (rsp > 255) rsp = 255;
  //   if (rsp < 0)   rsp = 0;


  //   analogWrite(speedL, lsp);
  //   analogWrite(speedR, rsp);
  //   delay(25);
}

int setDir(int error,int sensor3)
{
  int dir=1;
  if (error < 0) {     //right>left --->go left
    dir = 0;
    // Serial.println("l");
  } else if (error > 0) {  //left>right --->go right
    dir = 2;
    // Serial.println("r");
  } else if(error==0 && sensor3==0) {
    // Serial.println("s");
    dir = 1;
  }
  return dir;
}

void adjustMotors(int dir)
{
  Serial.print("ana fl adjust   ");
  Serial.println(dir);
  if(dir==1)
  {
    analogWrite(speedL, 80);
    analogWrite(speedR, 150);
    Serial.println("ana fl s");
  }
  else if(dir==0)
  {
    analogWrite(speedL, 70);
    analogWrite(speedR, 80);
    Serial.println("ana fl l");
  }
  else if(dir==2)
  {
    analogWrite(speedL, 80);
    analogWrite(speedR, 70);
    Serial.println("ana fl r");
  }
}
