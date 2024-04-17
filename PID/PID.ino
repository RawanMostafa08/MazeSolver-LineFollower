// #define SET_BIT(REG_NAME, BIT_NUMBER) REG_NAME |= (1 << BIT_NUMBER)
// #define CLEAR_BIT(REG_NAME, BIT_NUMBER) REG_NAME &= (~(1 << BIT_NUMBER))
// #define TOGGLE_BIT(REG_NAME, BIT_NUMBER) REG_NAME ^= (1 << BIT_NUMBER)
// #define READ_BIT(REG_NAME, BIT_NUM) (REG_NAME >> BIT_NUM) & 1
// #include <avr/io.h>
// #include <util/delay.h>

// //////////////////////////////////PID Values////////////////////////////////
// // Kp as initial = Max Motor Speed / Max Error fl Line
// double Kp, Ki, Kd;
// double lastError = 0;
// const int goal = 0;              // Center of the line
// long sensor[] = {0, 1, 2, 3, 4}; // leftmost - 0, rightmost - 4

// //////////////////////////////////Speeds////////////////////////////////
// const unsigned char MAX_SPEED = 100;
// int rspeed;
// int lspeed;
// void setup()
// {
//     // put your setup code here, to run once:
//     Kp = 0.028;
//     Kd = 0.2;
// }

// void loop()
// {
//     // put your main code here, to run repeatedly:
//      sensor_average = 0;
//      sensor_sum = 0;
//      i = 0;
//      for(int i = -2; i <= 2; i++)
//      {
//        sensor[i]=analogRead(i);
//        sensor_average = sensor[i]*i*1000; //weighted mean
//        sensor_sum += sensor[i];
//      }
//      int position = int(sensor_average / sensor_sum);
//      int error = goal - position; // Calc Error 3500 - pos

//     int error =               // sensor1 - sensor3;
//         int PID = pid(error); // Calc PID value
//     lastError = error;        // Make lastError with right error
//     // Adjust Motors Now based on PID value
//     // Speed One UP and Slow One Down
//     //  SetMotor 1 Speed(constraint(MAX_SPEED+PID , 0 , MAX_SPEED));
//     //  SetMotor 2 Speed(constraint(MAX_SPEED-PID , 0 , MAX_SPEED));
//     lspeed = lspeed + PID;
//     rspeed = rspeed - PID;
//     if (lspeed > MAX_SPEED)
//         lspeed = MAX_SPEED;
//     if (lspeed < 0)
//         lspeed = 0;
//     if (rspeed > MAX_SPEED)
//         rspeed = MAX_SPEED;
//     if (rspeed < 0)
//         rspeed = 0;
//   analogWrite(  left motor , lspeed);
//   analogWrite( right motor , rspeed);
// }

// double pid(double error)
// {
//     double derivative = error - lastError;
//     double output = Kp * error + Kd * derivative; // Calc PID value
//     return output;
// }






#define speedL 11
#define IN1 7
#define IN2 8
#define IN3 6
#define IN4 5
#define speedR 10

//#define turning_speed 80

int P, D, previousError, PIDvalue, error;
int lsp, rsp;      
int lfspeed = 80;
float Kp = 0.083; 
float Kd = 0.23;
float Ki = 0;
char data;


void setup()
{
  DDRC &= ~(0x1F);
  previousError = 0;
  pinMode(speedL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(speedR, OUTPUT);
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.begin(9600);
}


void loop()
{
  linefollow();
}

void linefollow()
{

  int sensor1 = analogRead(A1); //most left
  int sensor2 = analogRead(A2);
  int sensor3 = analogRead(A3);
  int sensor4 = analogRead(A4); 
  int sensor5 = analogRead(A5); //most right
   while( analogRead(A1) >= 200 && analogRead(A2) >= 200 && analogRead(A3) >= 200 && analogRead(A4) >= 200 && analogRead(A5) >= 200) {
    int turn_speed = 120;
    error = previousError;
    P = error;
    D = error - previousError;
  
    PIDvalue = (Kp * P) + (Kd * D);
    previousError = error;
  
    lsp = turn_speed + PIDvalue;
    rsp = turn_speed - PIDvalue;
  
    if (lsp > 255) lsp = 255;
    if (lsp < 0)   lsp = 0;
    if (rsp > 255) rsp = 255;
    if (rsp < 0)   rsp = 0;
    analogWrite(speedL, lsp);
    analogWrite(speedR, rsp);
  }
Serial.println("sensor1: " + String(sensor1) + ", sensor2: " + String(sensor2) + ", sensor3: " + String(sensor2) + ", sensor4: " + String(sensor4) + ", sensor5: " + String(sensor5));

//    Serial.flush();
  // if (sensor1 >= 500 and sensor2 >= 500 and sensor3 >= 500 and sensor4 >= 500 and sensor5 >= 500) {
  // brake();
  // if (error < 0) {
  //  change_speed(turning_speed, turning_speed);
  //  setup_motors(0, 1);
  //  }
  // if (error > 0) {
  //  change_speed(turning_speed, turning_speed);
  //  setup_motors(1, 0);
  //   }
  // } else {

  error = (sensor1+sensor2) - (sensor5 + sensor4);
  if(error < 0){
    sensor3 *= -1;
  }
  error += sensor3;
  if(error > 900) {
    error = 900;
  }
//  else if(error<15&&error>-15){ 
//    if(Serial.available()){
//      data=Serial.read();
//      if(data == '1') lfspeed = 150;
//      else lfspeed = 120;
//    }
//  }
  
//  Serial.println(error);
  static int I = 0;
  I += error;
  P = error; 
  D = error - previousError;


  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed + PIDvalue;
  rsp = lfspeed - PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0)   lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0)   rsp = 0;
  analogWrite(speedL, lsp);
  analogWrite(speedR, rsp);
  delay(25);
}


