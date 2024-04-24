
const int SensorPin0 = A5;
const int SensorPin1 = A4;
const int SensorPin2 = A3;
const int SensorPin3 = A2;
const int SensorPin4 = A1;

const int BackSensor0 = A6;   
const int BackSensor1 = A7;


const int leftSpeed = 11; // means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 10;
const int in1 = 7; //in1
const int in2 = 8; //in2

 // left 1 and left 2 control the direction of rotation of left motor
const int in3 = 2;  //in3
const int in4 = 6;  //in4

char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
int error = 0;
int extraInch = 200;
int LFSensor[5] = {0,0,0,0,0};
int BKSensor[2] = {0,0};


int P, D,I=0, previousError, PIDvalue;
int lsp , rsp ;
// int lfspeed = 80;
int lfspeed = 80;
// float Kp = 0.083;
// float Kd = 0.23;
// float Ki = 0;
float Kp = 8;
float Ki = 0.3;
float Kd = 3;



#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define RIGHT_TURN 4
#define LEFT_TURN 5


int mode = STOPPED;
int status = false;
void readLightSensor(){
  LFSensor[0] = !digitalRead(SensorPin0); 
  LFSensor[1] = !digitalRead(SensorPin1); 
  LFSensor[2] = !digitalRead(SensorPin2); 
  LFSensor[3] = !digitalRead(SensorPin3); 
  LFSensor[4] = !digitalRead(SensorPin4); 
}

void readBackSensor(){
  BkSensor[0] = !digitalRead(BackSensor0); 
  BKSensor[1] = !digitalRead(BackSensor1); 
}

void testSensorValues(){
  Serial.print(LFSensor[0]);
    Serial.print(" ");
  Serial.print(LFSensor[1]);
    Serial.print(" ");
  Serial.print(LFSensor[2]);
    Serial.print(" ");
  Serial.print(LFSensor[3]);
    Serial.print(" ");
  Serial.println(LFSensor[4]);
}

void testBackSensorValues(){
  Serial.print(BKSensor[0]);
    Serial.print(" ");
  Serial.println(BKSensor[1]);
}

void Read_IR_sensors()
{
  readLightSensor();
  // leftState=digitalRead(leftSensor);
  // rightState = digitalRead(rightSensor);
 
  testSensorValues();
  // far right sensor LFSensor[4]
  // far left sensor LFSensor[0]
  if (LFSensor[0] && LFSensor[1] && LFSensor[2] && LFSensor[3] && LFSensor[4])
  {
    // 11111

    mode=CONT_LINE;
   
    error = 0;
  }
  else if (!LFSensor[0] && LFSensor[4])
  {
    // 0XXX1
    mode = RIGHT_TURN;
    error = 0;
  }
  else if (LFSensor[0] && !LFSensor[4])
  {
    // 1XXX0
    mode = LEFT_TURN;
    error = 0;
  }
  else if (!LFSensor[0] && !LFSensor[1] && !LFSensor[2]&& !LFSensor[3] && !LFSensor[4])
  {
    // 00000
    mode = NO_LINE;
    Serial.print("hi   ");
    Serial.println(mode);
    error = 0;
  }
  else if (!LFSensor[0] && !LFSensor[1] && !LFSensor[2]&& LFSensor[3] && !LFSensor[4])
  {
    // Sensor Value
    // 0 0 0 1 0   2
    mode = FOLLOWING_LINE;
    error = 2;
  }
  else if (!LFSensor[0] && !LFSensor[1] && LFSensor[2]&& LFSensor[3] && !LFSensor[4])
  {
    // Sensor Value
    // 0 0 1 1 0
    mode = FOLLOWING_LINE;
    error = 1;
  }
  else if (!LFSensor[0] && !LFSensor[1] && LFSensor[2]&& !LFSensor[3] && !LFSensor[4])
  {
    // 0 0 1 0 0
    mode = FOLLOWING_LINE;
    error = 0;
  }
  else if (!LFSensor[0] && LFSensor[1] && LFSensor[2]&& !LFSensor[3] && !LFSensor[4])
  {
    // 0 1 1 0 0 -1
    mode = FOLLOWING_LINE;
    error = -1;
  }
  else if (!LFSensor[0] && LFSensor[1] && !LFSensor[2]&& !LFSensor[3] && !LFSensor[4])
  {
    // 0 1 0 0 0  -2
    mode = FOLLOWING_LINE;
    error = -2;
  }
  Serial.print("  mode: ");
  Serial.print(mode);
  Serial.print("  error:");
  Serial.println(error);
}


void setup(){
  Serial.begin(9600);
  pinMode(SensorPin0,INPUT);
  pinMode(SensorPin1,INPUT);
  pinMode(SensorPin2,INPUT);
  pinMode(SensorPin3,INPUT);
  pinMode(SensorPin4,INPUT);

  pinMode(BackSensor0,INPUT);
  pinMode(BackSensor1,INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
}


void stopCar()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(rightSpeed, 0);
    analogWrite(leftSpeed, 0);
}

void forward()
{                              // drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
   
    // the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);
   
    analogWrite(rightSpeed, 80);
    analogWrite(leftSpeed, 80);
    Serial.print("in forward");

}

void left()
{ // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); 
    digitalWrite(in4, LOW);
    analogWrite(rightSpeed, 80);
    analogWrite(leftSpeed, 80);
    Serial.print("in left");
}

void right()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);
   
    analogWrite(rightSpeed, 80);
    analogWrite(leftSpeed, 80);
}

void waitBefore(){
  while(BKSensor[0]&&BKSensor[1]){
  readBackSensor();
  }
}

void waitAfter(){
  while(BKSensor[0]&&BKSensor[1]){
  readBackSensor();
    }
}
void mazeSolve()
{
  // Serial.println("In Maze");
  while (!status) // it does not reach the end
  {
  // Serial.println("In status");
  Serial.print("Mode");
  Serial.println(mode);
  

    Read_IR_sensors();
    switch (mode)
    {
    case NO_LINE:
      Serial.println("NO_LINE");
      waitBefore();
      left();
     delay(500);

      break;

    case CONT_LINE:
      Serial.println("CONT_LINE");
      waitBefore();
      left();
      delay(500);
      break;

    case RIGHT_TURN:
      Serial.println("RIGHT_TURN");
      // wait for 2 sensor at least one 
      waitBefore();
      right();

       delay(500);
      break;

    case LEFT_TURN:
      Serial.println("LEFT_TURN");
      waitBefore();
      left();
       delay(500);
      break;

    case FOLLOWING_LINE:
    Serial.println("Forward");
      forward();
    
    }
  }
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
    analogWrite(leftSpeed, lsp);
    Serial.print(rsp);
    Serial.print("   ");
    Serial.println(lsp);
    analogWrite(rightSpeed, rsp);
}

void runExtraInch(void)
{
  motorPIDcontrol();
  delay(extraInch);
  stopCar();
}
void recIntersection(char direction)
{
  path[pathLength] = direction; // Store the intersection in the path variable.
  pathLength ++;
  // simplifyPath(); // Simplify the learned path.
}

void followingLine(void)
{
   //readLFSsensors(); 
   calculatePID();
   motorPIDcontrol();   
}

//----------------------------------------------
void mazeEnd(void)
{
  stopCar();
  for(int i=0;i<pathLength;i++)
    Serial.print(path[i]);
    //Serial.print(path[i]);
  Serial.print("  pathLenght ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = STOPPED;
}

// void mazeSolve(void)
// {
//     while (!status)
//     {
//         Read_IR_sensors();  
//         switch (mode)
//         {   
//           case NO_LINE:  
//             stopCar();
//             // left();
//              Serial.println("no line");
//             // goAndTurn (LEFT, 180);
//             recIntersection('B');
//             break;
          
//           case CONT_LINE: 
//             // runExtraInch();
//             Read_IR_sensors();
//             if (mode != CONT_LINE) 
//             {
//                Serial.println("cont line");
//               // left();
//               // goAndTurn (LEFT, 90); 
//               recIntersection('L');
//               } // or it is a "T" or "Cross"). In both cases, goes to LEFT
//             else mazeEnd(); 
//             break;
            
//          case RIGHT_TURN: 
//             // runExtraInch();
//             Read_IR_sensors();
//             if (mode == NO_LINE) 
//             {
//               // goAndTurn (RIGHT, 90);
//               stopCar();
//               right();
//               recIntersection('R');
//               }
//             else recIntersection('S');
//             break;   
            
//          case LEFT_TURN: 
//             // goAndTurn (LEFT, 90); 
//             // left();
//             Serial.println("turning left");
//             recIntersection('L');
//             break;   
         
//          case FOLLOWING_LINE: 
//             followingLine();
//             break;      
        
//          }
//     }
// }
void loop(){
  // left();
  // right();
  // forward();
  // delay(1000);
  // stopCar();
  // readLightSensor();
  // testSensorValues();
  mazeSolve();

}