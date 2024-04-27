#include <avr/io.h>
#include <util/delay.h>
#define SET_BIT(REG_NAME, BIT_NUMBER) REG_NAME |= (1 << BIT_NUMBER)
#define CLEAR_BIT(REG_NAME, BIT_NUMBER) REG_NAME &= (~(1 << BIT_NUMBER))
#define TOGGLE_BIT(REG_NAME, BIT_NUMBER) REG_NAME ^= (1 << BIT_NUMBER)
#define READ_BIT(REG_NAME, BIT_NUM) (REG_NAME >> BIT_NUM) & 1

#define speedL 11 
#define IN1 7
#define IN2 8
#define IN3 2
#define IN4 6
#define speedR 10

// #define turning_speed 80

int P, D,I=0, previousError, PIDvalue;
int lsp , rsp ;
// int lfspeed = 80;
int lfspeed = 70;
// float Kp = 0.083;
// float Kd = 0.23;
// float Ki = 0;
float Kp = 8;
float Ki = 0;
float Kd = 3;
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
        error = -4;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 0) && (sensor5 == 0))
        error = -3;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 0) && (sensor5 == 1))
        error = -2;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0) && (sensor5 == 1))
        error = -1;

    else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 1) && (sensor5 == 1))
        error = 0;
    else if ((sensor1 == 1) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 1) && (sensor5 == 1))
        error = 1;
    else if ((sensor1 == 1) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = 2;
    else if ((sensor1 == 0) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = 3;
    else if ((sensor1 == 0) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1))
        error = 4;
    // else if ((sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1) && (sensor5 == 1)) 
    //     error=-5;
    // error=-error;
//     else 
//     {
// if(error<0)
// error =-5;
// else error=5;

    // }

        Serial.print("error= ");
     Serial.println(error);
     
      
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
    Serial.print("PIDvalue  ");
    Serial.print(PIDvalue);
    // if (lsp > 110) lsp = 110;
    if (lsp < 70) lsp = 70;
    // if (rsp > 110) rsp = 110;
    if (rsp < 70) rsp = 70;
    analogWrite(speedL, lsp);
    Serial.print("lsp ");
    Serial.print(lsp);
    Serial.print("   ");
    Serial.print("rsp ");
    Serial.println(rsp);
    analogWrite(speedR, rsp);
}