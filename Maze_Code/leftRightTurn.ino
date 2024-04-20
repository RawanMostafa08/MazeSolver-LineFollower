#include <Wire.h>
// control pins for left and right motors
const int leftSpeed = 11; // means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 10;
const int left2 = 8; // left 1 and left 2 control the direction of rotation of left motor
const int left1 = 7;
const int right1 = 6;
const int right2 = 5;
//<---------------------------------------------------------Maze constants-------------------->
const int lineFollowSensor0 = A3;
const int lineFollowSensor1 = A2;
const int lineFollowSensor2 = A7;
// will  be modified
const int lineFollowSensor3 = 12;
const int lineFollowSensor4 = 13;
int LFSensor[5] = {0, 0, 0, 0, 0};
int mode = 0;
unsigned int status = 0; // solving = 0; reach end = 1
int extraInch = 200;
//-------------------------------------------------
// Specific Maze Phase 2 (optimization) definitions and variables

unsigned char dir;

// The path variable will store the path that the robot has taken:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)

char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
/* read line sensors values

Sensor Array 	Error Value
0 0 0 0 1	 4
0 0 0 1 1	 3
0 0 0 1 0	 2
0 0 1 1 0	 1
0 0 1 0 0	 0
0 1 1 0 0	-1
0 1 0 0 0	-2
1 1 0 0 0	-3
1 0 0 0 0	-4

1 1 1 1 1        0 Robot found continuous line - test if an intersection or end of maze
0 0 0 0 0        0 Robot found no line: turn 180o
*/

/*
Sensor  Array   Error Value
0 0 1   2
0 1 1   1
0 1 0   0
1 1 0  -1
1 0 0  -2
*/
const int iniMotorPower = 250;
const int adj = 0;
float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6
const int THRESHOLD = 150;
// PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;
//<---------------------------------------------------------Maze constants--------------------/>
// IR sensors
const int numSensors = 3;                  // Number of IR sensors
int sensorPins[numSensors] = {A7, A3, A2}; // Digital pins for sensors
int sensorValues[numSensors];              // Array to store sensor readings

const int MPU = 0x68;                                           // MPU6050 I2C address
float AccX, AccY, AccZ;                                         // linear acceleration
float GyroX, GyroY, GyroZ;                                      // angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int turned = false;
const int maxSpeed = 60; // max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 10; // min PWM value at which motor moves
float angle;             // due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; // rough estimate of PWM at the speed pin of the stronger motor, while driving straight
// and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false;    // it the car driving forward OR rotate/stationary
bool prevIsDriving = true; // equals isDriving in the previous iteration of void loop()
bool paused = false;       // is the program paused
bool forwardCond = false;
bool rightCond = false;
bool leftCond = false;
void setup()
{
  Serial.begin(9600);
  Wire.begin();                // Initialize comunication
  Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);            // Talk to the register 6B
  Wire.write(0x00);            // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  // end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);

  for (int i = 0; i < numSensors; i++)
  {
    pinMode(sensorPins[i], INPUT); // Set sensor pins as input
  }
  currentTime = micros();
}
void getCurrentAngle()
{
  // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; // GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  // combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.99 * gyroAngleX + 0.01 * accAngleX;
  pitch = 0.99 * gyroAngleY + 0.01 * accAngleY;
  angle = roll; // if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
  // for me, turning right reduces angle. Turning left increases angle.
}

void forwardAction()
{
  // drive forward
  Serial.println("forward");
  isDriving = true;
}
void rightAction()
{
  // turn right
  turned = true;
  Serial.println("right");
  targetAngle -= 90;
  if (targetAngle <= -180)
  {
    targetAngle += 360;
  }
  isDriving = false;
}
void leftAction()
{
  // turn left
  Serial.println("left");
  targetAngle += 90;
  if (targetAngle > 180)
  {
    targetAngle -= 360;
  }
  isDriving = false;
}
void stopAction()
{
  // stop or brake
  Serial.println("stop");
  isDriving = false;
}
void turn_exact()
{
  getCurrentAngle();

  static int countStraight;

  if (isDriving != prevIsDriving)
  {
    leftSpeedVal = equilibriumSpeed;
    countStraight = 0;
    Serial.print("mode changed, isDriving: ");
    Serial.println(isDriving);
  }
  if (isDriving)
  {
    if (abs(targetAngle - angle) < 3)
    {
      if (countStraight < 20)
      {
        countStraight++;
      }
      else
      {
        countStraight = 0;
        equilibriumSpeed = leftSpeedVal; // to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
        Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
        Serial.println(equilibriumSpeed);
      }
    }
    else
    {
      countStraight = 0;
    }
    driving();
  }
  else
  {
    rotate();
  }
  prevIsDriving = isDriving;

  Serial.print("angle : ");
  Serial.println(angle);
}
void driving()
{                                              // called by void loop(), which isDriving = true
  int deltaAngle = round(targetAngle - angle); // rounding is neccessary, since you never get exact values in reality
  forward();
  if (deltaAngle != 0)
  {
    controlSpeed();
    rightSpeedVal = maxSpeed;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

void controlSpeed()
{ // this function is called by driving ()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;

  // setting up propoertional control, see Step 3 on the website
  if (deltaAngle > 30)
  {
    targetGyroX = 60;
  }
  else if (deltaAngle < -30)
  {
    targetGyroX = -60;
  }
  else
  {
    targetGyroX = 2 * deltaAngle;
  }

  if (round(targetGyroX - GyroX) == 0)
  {
    ;
  }
  else if (targetGyroX > GyroX)
  {
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); // would increase GyroX
  }
  else
  {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate()
{ // called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1)
  {
    stopCar();
  }
  else
  {
    if (angle > targetAngle)
    { // turn left
      left();
    }
    else if (angle < targetAngle)
    { // turn right
      right();
    }

    // setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30)
    {
      targetGyroX = 60;
    }
    else
    {
      targetGyroX = 2 * abs(deltaAngle);
    }

    if (round(targetGyroX - abs(GyroX)) == 0)
    {
      ;
    }
    else if (targetGyroX > abs(GyroX))
    {
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); // would increase abs(GyroX)
    }
    else
    {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

int changeSpeed(int motorSpeed, int increment)
{
  motorSpeed += increment;
  if (motorSpeed > maxSpeed)
  { // to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  }
  else if (motorSpeed < minSpeed)
  {
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void calculateError()
{
  // When this function is called, ensure the car is stationary. See Step 2 for more info

  // Read accelerometer values 200 times
  c = 0;
  while (c < 300)
  {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  // Divide the sum by 1000 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 300;
  AccErrorY = AccErrorY / 300;
  c = 0;

  // Read gyro values 1000 times
  while (c < 300)
  {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  // Divide the sum by 1000 to get the error value
  GyroErrorX = GyroErrorX / 300;
  GyroErrorY = GyroErrorY / 300;
  GyroErrorZ = GyroErrorZ / 300;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  // For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Serial.print("AccX");
  // Serial.println(AccX);
  // Serial.print("AccY");
  // Serial.println(AccY);
  // Serial.print("AccZ");
  // Serial.println(AccZ);
}

void readGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Serial.print("GyroX");
  // Serial.println(GyroX);
  // Serial.print("GyroY");
  // Serial.println(GyroY);
  // Serial.print("GyroZ");
  // Serial.println(GyroZ);
}

void stopCar()
{
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward()
{                             // drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, HIGH); // the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void left()
{ // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void right()
{
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}
void loop ()
{
  forward();
  delay(1000);
  targetAngle-=90;
  turn_exact();
  forward();
  delay(3000);
  targetAngle+=90;
  turn_exact();
  stopCar();
  delay(3000);
}