#include <EEPROM.h>

int add = 0;
void readStringFromEEPROM(int address, char* buffer, int bufSize) {
    for (int i = 0; i < bufSize; i++) {
        buffer[i] = EEPROM.read(address + i); // Read each character from EEPROM
        if (buffer[i] == '\0') { // Stop if null terminator is found
            break;
 }
}
}
void writeStringToEEPROM(int address, const char* data) {
    int length = strlen(data); // Get length of the string
    for (int i = 0; i < length; i++) {
        EEPROM.write(address + i, data[i]); // Write each character to EEPROM
    }
    EEPROM.write(address + length, '\0'); // Write null terminator to mark end of string
}

const int motorLeftA = 2;       // Digital pin for left motor A
const int motorLeftB = 6;       // Digital pin for left motor B
const int motorRightA = 7;      // Digital pin for right motor A
const int motorRightB = 8;      // Digital pin for right motor B
const int BackSensorleft = 3;   // left
const int BackSensorright = 5;  // right
const int veryBackSensor = 4;   // right
// Read sensor values
const int sensorPin[5] = { A1, A2, A3, A4, A5 };  // Array of digital pins for the sensors
const int numSensors = 5;                         // Number of sensors
int LFSensor[numSensors];
int BKSensor[3] = { 0, 0, 0 };
#define LEFT_MOTOR_PIN 10
#define RIGHT_MOTOR_PIN 11

//SPEED 100
// Max Kp in P controller = 18
double Kp = 13;
double Ki = 0;
double Kd = 30;
int baseSpeed = 65;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define RIGHT_TURN 4
#define LEFT_TURN 5


double error, integral = 0.0, derivative, output;
double previous_error = 0.0;  // Initialize previous error for derivative calculation
int mode = STOPPED, previousMode = STOPPED;
int status = false;
int turnspeed = 65;
int uturnspeed = 65;
char* path = "";
unsigned char pathLength = 0;  // the length of the path
int pathIndex = 0;
void readSensors() {
  for (int i = 0; i < numSensors; i++) {
    LFSensor[i] = !digitalRead(sensorPin[i]);
  }
  BKSensor[0] = digitalRead(BackSensorleft);  // left
  BKSensor[1] = digitalRead(veryBackSensor);  // right
  BKSensor[2] = digitalRead(BackSensorright);
}

void testLFSensor() {
  // Serial.print(LFSensor[0]);
  // Serial.print(" ");
  // Serial.print(LFSensor[1]);
  // Serial.print(" ");
  // Serial.print(LFSensor[2]);
  // Serial.print(" ");
  // Serial.print(LFSensor[3]);
  // Serial.print(" ");
  // Serial.println(LFSensor[4]);

  // Serial.print(BKSensor[0]);
  // Serial.print(" ");
  // Serial.print(BKSensor[1]);
  // Serial.print(" ");
  // Serial.println(BKSensor[2]);
}

void Read_IR_sensors() {
  readSensors();
  testLFSensor();
  previousMode = mode;
  // if (BKSensor[0] && BKSensor[2]) {
  //   mode = CONT_LINE;
  // } else
  if (BKSensor[0]) {
    mode = LEFT_TURN;
  } else if (LFSensor[0] || LFSensor[1] || LFSensor[2] || LFSensor[3] || LFSensor[4]) {
    mode = FOLLOWING_LINE;
  } else if (BKSensor[2]) {
    mode = RIGHT_TURN;
  } else if ((!LFSensor[0] && !LFSensor[1] && !LFSensor[2] && !LFSensor[3] && !LFSensor[4]) && !BKSensor[1]) {
    mode = NO_LINE;
  }
}
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

  integral += error;
  derivative = (error - previous_error);
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  controlMotors(output, error);
}

void controlMotors(double speed, double error) {
  int leftSpeed, rightSpeed;

  leftSpeed = baseSpeed + (speed);   // Adjust scaling factor for left speed reduction
  rightSpeed = baseSpeed - (speed);  // Adjust scaling factor for right speed increase
  if (leftSpeed > 255)
    leftSpeed = 255;
  if (leftSpeed < 0)
    leftSpeed = 0;
  if (rightSpeed > 255)
    rightSpeed = 255;
  if (rightSpeed < 0)
    rightSpeed = 0;

  analogWrite(LEFT_MOTOR_PIN, leftSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightSpeed);
  // Serial.print("  Error: ");
  // Serial.print(error);

  // Serial.print("  Right= ");
  // Serial.print(leftSpeed);
  // Serial.print("  Left=");
  // Serial.println(rightSpeed);
}

void stopCar() {
  analogWrite(LEFT_MOTOR_PIN, 0);
  analogWrite(RIGHT_MOTOR_PIN, 0);
}

void forward() {
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
  PID();
}

void adjustDirection() {
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
}

void left() {  // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
  analogWrite(RIGHT_MOTOR_PIN, turnspeed);
  analogWrite(LEFT_MOTOR_PIN, turnspeed);
  // Serial.println("lefttttt");
}

void Uturn() {  // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
  analogWrite(RIGHT_MOTOR_PIN, uturnspeed);
  analogWrite(LEFT_MOTOR_PIN, uturnspeed);
  // Serial.print("uturn");
}

void right() {
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, HIGH);
  analogWrite(RIGHT_MOTOR_PIN, turnspeed);
  analogWrite(LEFT_MOTOR_PIN, turnspeed);
}

void Backward() {
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, HIGH);
  analogWrite(RIGHT_MOTOR_PIN, turnspeed);
  analogWrite(LEFT_MOTOR_PIN, turnspeed);
}

void mazeSolve() {

  Read_IR_sensors();

  switch (mode) {
    case NO_LINE:
      stopCar();
      delay(500);
      Uturn();
      delay(1000);
      Serial.println("ana abl el while");
      while (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0) {
        Serial.println("ana fel while ele fe no line");
        readSensors();
      }
      stopCar();
      Serial.println("ana ba3d el stop");
      delay(500);
      recIntersection('B');
      break;

    case RIGHT_TURN:
      Serial.println("RIGHT_TURN");

      stopCar();
      delay(500);

      right();

      delay(500);

      stopCar();
      delay(500);
      mode = FOLLOWING_LINE;
      recIntersection('R');
      break;

    case LEFT_TURN:
    case CONT_LINE:
      Serial.println("LEFT_TURN");
      stopCar();
      delay(500);

      left();

      delay(500);

      stopCar();
      delay(500);
      mode = FOLLOWING_LINE;

      recIntersection('L');
      break;
    case FOLLOWING_LINE:
      Serial.println("Forward");
      forward();
      recIntersection('S');
      break;
  }
}

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging (optional)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(BackSensorright, INPUT);
  pinMode(veryBackSensor, INPUT);
  pinMode(BackSensorleft, INPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);


  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorLeftB, LOW);

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  analogWrite(LEFT_MOTOR_PIN, 30);
  analogWrite(RIGHT_MOTOR_PIN, 30);
}

void loop() {
  //   mazeSolve();  // First pass to solve the maze
  // Serial.print("11111111111111111111111");
  mazeOptimization();
}


void StopMotors() {
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, LOW);
}

void MoveMotors() {
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorLeftB, LOW);
}

void recIntersection(char direction) {
  writeStringToEEPROM(add,direction);
  path += direction;  // Store the intersection in the path variable.
  // pathLength ++;
  simplifyPath();  // Simplify the learned path.
}
void simplifyPath() {
  readStringFromEEPROM(add,path,100);
  // only simplify the path if the second-to-last turn was a 'B'
  if (pathLength < 3 || path[pathLength - 2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for (i = 1; i <= 3; i++) {
    switch (path[pathLength - i]) {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle) {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }

  // The path is now two steps shorter.
  pathLength -= 2;
}

void mazeOptimization(void) {
  while (!status) {
    Read_IR_sensors();
    switch (mode) {
      case FOLLOWING_LINE:
        forward();
        break;
      case CONT_LINE:
        // if (pathIndex >= pathLength) mazeEnd ();
        // else
        {
          mazeTurn(path[pathIndex]);
          pathIndex++;
        }
        break;
      case LEFT_TURN:
        // if (pathIndex >= pathLength) mazeEnd ();
        // else
        {
          mazeTurn(path[pathIndex]);
          pathIndex++;
        }
        break;
      case RIGHT_TURN:
        // if (pathIndex >= pathLength) mazeEnd ();
        // else
        {
          mazeTurn(path[pathIndex]);
          pathIndex++;
        }
        break;
    }
  }
}
void mazeTurn(char dir) {
  switch (dir) {
    case 'L':  // Turn Left

      Serial.println("LEFT_TURN");
      stopCar();
      delay(500);

      left();

      delay(500);

      stopCar();
      delay(500);
      mode = FOLLOWING_LINE;
      break;
    case 'R':  // Turn Right
      Serial.println("RIGHT_TURN");

      stopCar();
      delay(500);

      right();

      delay(500);

      stopCar();
      delay(500);
      mode = FOLLOWING_LINE;
      break;

    case 'B':  // Turn Back
      stopCar();
      delay(500);
      Uturn();
      delay(1000);
      // Serial.println("ana abl el while");
      while (LFSensor[0] == 0 && LFSensor[1] == 0 && LFSensor[2] == 0) {
        // Serial.println("ana fel while ele fe no line");
        readSensors();
      }
      stopCar();
      // Serial.println("ana ba3d el stop");
      delay(500);
      break;

    case 'S':  // Go Straight
      forward();
      break;
  }
}



void mazeEnd(void) {
  stopCar();
  Serial.println("The End  ==> Path: ");
  for (int i = 0; i < pathLength; i++)
    Serial.print(path[i]);
  //Serial.print(path[i]);
  Serial.println("");
  Serial.print("  pathLenght ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = STOPPED;
}
