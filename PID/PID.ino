#define speedR 10
#define IN1 7
#define IN2 8
#define IN3 2
#define IN4 6 
#define speedL 11

//#define turning_speed 80

int P, D, previousError, PIDvalue, error;
int lsp, rsp;      
int lfspeed = 100;
float Kp = 0.083; 
float Kd = 0.23;
float Ki = 0;
char data;


void setup()
{
  // DDRC &= ~(0x1F);
   pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
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

  int sensor1 = analogRead(A1);
  int sensor2 = analogRead(A2);
  int sensor3 = analogRead(A3);
  int sensor4 = analogRead(A4);
  int sensor5 = analogRead(A5);
  while(analogRead(A5) >= 200 && analogRead(A1) >= 200 && analogRead(A2) >= 200 && analogRead(A3) >= 200 && analogRead(A4) >= 200) {
    int turn_speed = 80;
    error = previousError;
    P = error;
    D = error - previousError;
  
    PIDvalue = (Kp * P) + (Kd * D);
    previousError = error;
  
    lsp = turn_speed + PIDvalue;
    rsp = turn_speed - PIDvalue;
  
  if (lsp > 100) lsp = 100;
  if (lsp < 0)   lsp = 0;
  if (rsp > 100) rsp = 100;
  if (rsp < 0)   rsp = 0;
    analogWrite(speedL, lsp);
    analogWrite(speedR, rsp);
  }
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
  Serial.print("error  ");
  Serial.print(error);
  Serial.print("  sensor1  ");
  Serial.print(sensor1);
  Serial.print("  sensor2  ");
  Serial.print(sensor2);
  Serial.print("  sensor3  ");
  Serial.print(sensor3);
  Serial.print("  sensor4  ");
  Serial.print(sensor4);
  Serial.print("  sensor5  ");
  Serial.println(sensor5);
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

  if (lsp > 100) lsp = 100;
  if (lsp < 0)   lsp = 0;
  if (rsp > 100) rsp = 100;
  if (rsp < 0)   rsp = 0;
  analogWrite(speedL, lsp);
  analogWrite(speedR, rsp);

  Serial.print("  speedL  ");
  Serial.print(lsp);
  Serial.print("  speedR  ");
  Serial.println(rsp);
  delay(25);
}






//void brake() {
// digitalWrite(IN1, HIGH); // make left motor A brake
// digitalWrite(IN2, HIGH);
// digitalWrite(IN3, HIGH); // make right motor B brake
// digitalWrite(IN4, HIGH);
//}

//void setup_motors(int forward_a, int forward_b) {
// if (forward_a == 1) {
//  digitalWrite(IN1, LOW);
//  digitalWrite(IN2, HIGH);
// } else {
//  digitalWrite(IN1, LOW);
//  digitalWrite(IN2, LOW);
// }
// if (forward_b == 1) {
//  digitalWrite(IN3, HIGH);
//  digitalWrite(IN4, LOW);
// } else {
//  digitalWrite(IN3, LOW);
//  digitalWrite(IN4, LOW);
// }
//}

//void change_speed(int leftSpeed, int rightSpeed) {
// analogWrite(speedR, rightSpeed); 
// analogWrite(speedL, leftSpeed); 
//}