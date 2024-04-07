// LFSensor more to the Left is "0"
const int lineFollowSensor0 = A3;
const int lineFollowSensor1 = A2;
const int lineFollowSensor2 = A7;
const int lineFollowSensor3 = 12;
const int lineFollowSensor4 = 13;
int LFSensor[5] = {0, 0, 0, 0, 0};
int mode = 0;

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
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6
const int THRESHOLD = 150;

void checkPIDvalues()
{
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);
}

void setup()
{
  // line follow sensors
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);
  checkPIDvalues();
}
void testSensorValues()
{
  Serial.print("LFSensor: ");
  Serial.print(LFSensor[0]);
  Serial.print(" - ");
  Serial.print(LFSensor[1]);
  Serial.print(" - ");
  Serial.print(LFSensor[2]);
  Serial.print(" - ");
  Serial.print(LFSensor[3]);
  Serial.print(" - ");
  Serial.println(LFSensor[4]);
}
void loop()
{
  LFSensor[0] = analogRead(lineFollowSensor0);
  LFSensor[1] = analogRead(lineFollowSensor1);
  LFSensor[2] = analogRead(lineFollowSensor2);
  LFSensor[3] = analogRead(lineFollowSensor3);
  LFSensor[4] = analogRead(lineFollowSensor4);
  testSensorValues();
  //far right sensor LFSensor[4]
  //far left sensor LFSensor[0]
  if ((LFSensor[0] < THRESHOLD) && (LFSensor[1] < THRESHOLD) && (LFSensor[2] < THRESHOLD) && (LFSensor[3] < THRESHOLD) && (LFSensor[4] < THRESHOLD))
  {
    //11111
    mode = CONT_LINE; 
    error = 0;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[4] < THRESHOLD))
  {
    //0XXX1
    mode = RIGHT_TURN;
    error = 0;
  }
  else if ((LFSensor[0] < THRESHOLD) && (LFSensor[4] == 0))
  {
    //1XXX0
    mode = LEFT_TURN;
    error = 0;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    mode = NO_LINE;
    error = 0;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] <THRESHOLD) && (LFSensor[4] == 0))
  {
    //Sensor Value 
    //0 0 0 1 0   2
    mode = FOLLOWING_LINE;
    error = 2;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] < THRESHOLD) && (LFSensor[3] < THRESHOLD) && (LFSensor[4] == 0))
  {
    //Sensor Value 
    //0 0 1 1 0
    mode = FOLLOWING_LINE;
    error = 1;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] < THRESHOLD) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    // 0 0 1 0 0
    mode = FOLLOWING_LINE;
    error = 0;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] < THRESHOLD) && (LFSensor[2] < THRESHOLD) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    //0 1 1 0 0 -1
    mode = FOLLOWING_LINE;
    error = -1;
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] < THRESHOLD) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    //0 1 0 0 0  -2
    mode = FOLLOWING_LINE;
    error = -2;
  }
  Serial.print("  mode: ");
  Serial.print(mode);
  Serial.print("  error:");
  Serial.println(error);
}