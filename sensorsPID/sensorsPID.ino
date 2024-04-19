#include <avr/io.h>
#include <util/delay.h>
#define SET_BIT(REG_NAME, BIT_NUMBER) REG_NAME |= (1 << BIT_NUMBER)
#define CLEAR_BIT(REG_NAME, BIT_NUMBER) REG_NAME &= (~(1 << BIT_NUMBER))
#define TOGGLE_BIT(REG_NAME, BIT_NUMBER) REG_NAME ^= (1 << BIT_NUMBER)
#define READ_BIT(REG_NAME, BIT_NUM) (REG_NAME >> BIT_NUM) & 1

#define speedL 11
#define IN1 8
#define IN2 7
#define IN3 6
#define IN4 2
#define speedR 10

// #define turning_speed 80

int P, D,I=0, previousError, PIDvalue;
int lsp , rsp ;
// int lfspeed = 80;
int lfspeed = 80;
// float Kp = 0.083;
// float Kd = 0.23;
// float Ki = 0;
float Kp = 10;
float Ki = 0;
float Kd = 0;
int error;
int getError(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5);
int calculatePID();

void setup()
{
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

void loop()
{
    int sensor1 = digitalRead(A1);
    int sensor2 = digitalRead(A2);
    int sensor3 = digitalRead(A3);
    int sensor4 = digitalRead(A4);
    int sensor5 = digitalRead(A5);
    calculateError(sensor1, sensor2, sensor3, sensor4, sensor5);
    int PIDval = calculatePID();
    motorPIDcontrol();
}

void calculateError(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
    if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 0))
        error = 4;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 0) && (sensor5 == 0))
        error = 3;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 0) && (sensor5 == 1))
        error = 2;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0) && (sensor5 == 1))
        error = 1;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 1) && (sensor5 == 1))
        error = 0;

    else if ((sensor1 == 1) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 1) && (sensor5 == 1))
        error = -1;

    else if ((sensor1 == 1) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = -2;

    else if ((sensor1 == 0) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = -3;

    else if ((sensor1 == 0) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = -4;
}
int calculatePID()
{
     P = error;
     I = I + error;
     D = error - previousError;
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;
    return PIDvalue;
}

void motorPIDcontrol()
{
    lsp = lfspeed + PIDvalue;
    rsp = lfspeed - PIDvalue; 
    if (lsp > 150) lsp = 150;
    if (lsp < 60) lsp = 60;
    if (rsp > 150) rsp = 150;
    if (rsp < 60) rsp = 60;
    analogWrite(speedL, lsp);
    Serial.print(rsp);
    Serial.print("   ");
    Serial.println(lsp);
    analogWrite(speedR, rsp);
}