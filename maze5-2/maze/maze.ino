const int SensorPin0 = A1;
const int SensorPin1 = A2;
const int SensorPin2 = A3;
const int SensorPin3 = A4;
const int SensorPin4 = A5;

const int BackSensorleft = 4;    // left
const int BackSensorcenter = 5;  // right
const int BackSensorright = 3;   // right

const int leftSpeed = 10;  // means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 11;
const int in1 = 7;  // in1
const int in2 = 8;  // in2

// left 1 and left 2 control the direction of rotation of left motor
const int in3 = 2;  // in3
const int in4 = 6;  // in4

// char path[100] = "";
// unsigned char pathLength = 0; // the length of the path
// int pathIndex = 0;
int error = 0;
// int extraInch = 200;
int LFSensor[5] = { 0, 0, 0, 0, 0 };
int BKSensor[3] = { 0, 0, 0 };

// #define turning_speed 80
int P, D, I = 0, previousError, PIDvalue;
int lsp, rsp;
int lfspeed = 33;
int turnspeed = 40;
double Kp = 4;
double Ki = 0;
double Kd = 10;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define RIGHT_TURN 4
#define LEFT_TURN 5

int mode = STOPPED, previousMode = STOPPED;
int status = false;
void PID() {
  if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 1 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = 0;
  else if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 1 && LFSensor[3] == 1 && LFSensor[4] == 0)
    error = 1;
  else if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0 && LFSensor[3] == 1 && LFSensor[4] == 0)
    error = 2;
  else if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0 && LFSensor[3] == 1 && LFSensor[4] == 1)
    error = 3;
  else if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0 && LFSensor[3] == 0 && LFSensor[4] == 1)
    error = 4;
  else if (LFSensor[0] == 0 && LFSensor[1] == 1 && LFSensor[2] == 1 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = -1;
  else if (LFSensor[0] == 0 && LFSensor[1] == 1 && LFSensor[2] == 0 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = -2;
  else if (LFSensor[0] == 1 && LFSensor[1] == 1 && LFSensor[2] == 0 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = -3;
  else if (LFSensor[0] == 1 && LFSensor[1] == 0 && LFSensor[2] == 0 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = -4;
  else if (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0 && LFSensor[3] == 0 && LFSensor[4] == 0)
    error = 0;

  P = error;
  I += error;
  D = (error - previousError);
  PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;
  if (lsp > 150)
    // lsp = 150;
    lsp = 255;
  if (lsp < 0)
    lsp = 0;
  if (rsp > 150)
    // rsp = 150;
    rsp = 255;
  if (rsp < 0)
    rsp = 0;

  // Serial.print("  rsp");
  // Serial.print(lsp);
  // Serial.print("  lsp");
  // Serial.println(rsp);


  analogWrite(leftSpeed, rsp);
  analogWrite(rightSpeed, lsp);
}

void readLightSensor() {
  // 0 --> white
  LFSensor[0] = !digitalRead(SensorPin0);
  LFSensor[1] = !digitalRead(SensorPin1);
  LFSensor[2] = !digitalRead(SensorPin2);
  LFSensor[3] = !digitalRead(SensorPin3);
  LFSensor[4] = !digitalRead(SensorPin4);
}
void readBackSensor() {

  BKSensor[0] = digitalRead(BackSensorleft);    // left
  BKSensor[1] = digitalRead(BackSensorcenter);  // right
  BKSensor[2] = digitalRead(BackSensorright);
}

void testLFSensor() {
  Serial.print(LFSensor[0]);
  Serial.print(" ");
  Serial.print(LFSensor[1]);
  Serial.print(" ");
  Serial.print(LFSensor[2]);
  Serial.print(" ");
  Serial.print(LFSensor[3]);
  Serial.print(" ");
  Serial.println(LFSensor[4]);

  Serial.print(BKSensor[0]);
  Serial.print(" ");
  Serial.print(BKSensor[1]);
  Serial.print(" ");
  Serial.println(BKSensor[2]);
}

void Read_IR_sensors() {
  readLightSensor();
  readBackSensor();
  // leftState=digitalRead(leftSensor);
  // rightState = digitalRead(rightSensor);

  // testLFSensor();
  // far right sensor LFSensor[4]
  // far left sensor LFSensor[0]
  previousMode = mode;
  if (LFSensor[0] && LFSensor[1] && LFSensor[2] && LFSensor[3] && LFSensor[4]) {
    // 11111

    mode = CONT_LINE;
    // error = 0;
  } else if (!LFSensor[0] && LFSensor[2] && LFSensor[3] && LFSensor[4]) {
    // 0XXX1
    mode = RIGHT_TURN;
    // error = 0;
  } else if (LFSensor[0] && LFSensor[1] && LFSensor[2] && !LFSensor[4]) {
    // 1XXX0
    // 100X0
    mode = LEFT_TURN;
    // error = 0;
  } else if (!LFSensor[0] && !LFSensor[1] && !LFSensor[2] && !LFSensor[3] && !LFSensor[4] && previousMode == FOLLOWING_LINE && BKSensor[0] == 0 && BKSensor[1] == 0 && BKSensor[2] == 0) {
    // 00000
    mode = NO_LINE;
    // error = 0;
  } else
    mode = FOLLOWING_LINE;
}

void setup() {
  Serial.begin(9600);
  pinMode(SensorPin0, INPUT);
  pinMode(SensorPin1, INPUT);
  pinMode(SensorPin2, INPUT);
  pinMode(SensorPin3, INPUT);
  pinMode(SensorPin4, INPUT);

  pinMode(BackSensorright, INPUT);
  pinMode(BackSensorcenter, INPUT);
  pinMode(BackSensorleft, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopCar() {
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  PID();
}
void adjustDirection() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left() {               // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(in1, HIGH);  //right
  digitalWrite(in2, LOW);   //right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(rightSpeed, turnspeed);
  analogWrite(leftSpeed, 0);
  Serial.println("lefttttt");
}
void Uturn() {              // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(in1, LOW);   //right
  digitalWrite(in2, HIGH);  //right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(rightSpeed, 38);
  analogWrite(leftSpeed, 38);
  Serial.print("uturn");
}

void right() {
  digitalWrite(in1, HIGH);  //right
  digitalWrite(in2, LOW);   //right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, turnspeed);
}

void mazeSolve() {

  // // Serial.println("In Maze");
  while (!status)  // it does not reach the end
  {
    // Serial.println("In status");
    // Serial.print("Mode");
    // Serial.println(mode);

    Read_IR_sensors();

    switch (mode) {
      case NO_LINE:
        Serial.println("uturn");
        if (previousMode == FOLLOWING_LINE) {
          Serial.println("ana fe no line");
          // forward();
          stopCar();
          delay(500);
          Uturn();
          delay(1000);
          Serial.println("ana abl el while");
          while (!(BKSensor[0] == 0 && BKSensor[1] == 1 && BKSensor[2] == 0)) {
            Serial.println("ana fel while ele fe no line");
            readBackSensor();
          }
          stopCar();
          Serial.println("ana ba3d el sto0p");
          delay(500);

          mode = FOLLOWING_LINE;
        }
        // else {

        // }
        // //   // recIntersection('B');
        break;

      case RIGHT_TURN:
        Serial.println("RIGHT_TURN");
        adjustDirection();
        ///Reaching right turn
        readBackSensor();

        Serial.print("Bara while  ");
        Serial.println(BKSensor[2]);

        while (!(BKSensor[2] == 1 && BKSensor[1] == 1))  //not black
        {                                                // 0 -> there is right turn
          readBackSensor();
          Serial.print("Gowa while  ");
          Serial.println(BKSensor[2]);
        }

        stopCar();
        delay(500);
        // Serial.println("Ana ba3d elwhile yrgala");

        right();
        delay(1000);
        //hastna le7ad ma elky f nos yeb2a white
        // while(BKSensor[1] != 0)
        // {
        //           readBackSensor();
        // }
        //ba3den yeb2a ely f nos eswed
        while (!(BKSensor[0] == 0 && BKSensor[1] == 1 && BKSensor[2] == 0))  //not black
        {
          readBackSensor();
        }
        stopCar();
        delay(500);
        mode = FOLLOWING_LINE;
        // recIntersection('R');
        break;

      case LEFT_TURN:
      case CONT_LINE:
        Serial.println("LEFT_TURN");
        adjustDirection();
        while (!(BKSensor[0] == 1 && BKSensor[1] == 1))  //not black
        {                                                // 0 -> there is right turn
          readBackSensor();
        }
        stopCar();
        delay(500);

        left();
        delay(1000);

        while (!(BKSensor[0] == 0 && BKSensor[1] == 1 && BKSensor[2] == 0))  //not black
        {
          readBackSensor();
        }
        stopCar();
        delay(500);
        mode = FOLLOWING_LINE;

        // recIntersection('S');
        break;
      case FOLLOWING_LINE:
        Serial.println("Forward");
        testLFSensor();
        forward();
        // recIntersection('L');
        break;
    }
  }
}

void loop() {
  mazeSolve();  // First pass to solve the maze
  // right(50);
  // right(50);
  // Uturn();
  // right();
  // readLightSensor();
  // testLFSensor();
  // forward();
}