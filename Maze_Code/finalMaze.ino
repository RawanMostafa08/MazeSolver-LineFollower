const int SensorPin0 = A1;
const int SensorPin1 = A2;
const int SensorPin2 = A3;
const int SensorPin3 = A4;
const int SensorPin4 = A5;

const int BackSensor0 = 4; // left
const int BackSensor1 = 3; // right

const int leftSpeed = 11; // means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 10;
const int in1 = 7; // in1
const int in2 = 8; // in2

// left 1 and left 2 control the direction of rotation of left motor
const int in3 = 2; // in3
const int in4 = 6; // in4

char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
int error = 0;
int extraInch = 200;
int LFSensor[5] = {0, 0, 0, 0, 0};
int BKSensor[2] = {0, 0};

// #define turning_speed 80
int P, D, I = 0, previousError, PIDvalue;
int lsp, rsp;
int lfspeed = 150;
// float Kp = 0.01;
// float Kd = 0.9;
// float Ki = 0;
float Kp = 0.03;
float Kd = 0.2;
float Ki = 0;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define RIGHT_TURN 4
#define LEFT_TURN 5

int mode = STOPPED;
int status = false;
void calculateError_LF(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
    error = (sensor5 + sensor4) - (sensor1 + sensor2);
    if (error < 0)
        error -= sensor3;
    else
        error += sensor3;
    if (sensor1 < 100 && sensor2 > 200 && sensor3 > 200 && sensor4 > 200 && sensor5 > 200)
        error += error;
    if (sensor1 > 200 && sensor2 > 200 && sensor3 > 200 && sensor4 > 200 && sensor5 < 100)
        error += error;

    Serial.print("error= ");
    Serial.println(error);
}
void calculatePID_LF()
{
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;
}
void motorPIDcontrol_LF()
{
    lsp = lfspeed + PIDvalue;
    rsp = lfspeed - PIDvalue;
    Serial.print("PIDvalue  ");
    Serial.print(PIDvalue);
    if (lsp > 200)
        lsp = 200;
    if (lsp < 0)
        lsp = 0;
    if (rsp > 200)
        rsp = 200;
    if (rsp < 0)
        rsp = 0;
    analogWrite(leftSpeed, lsp);
    analogWrite(rightSpeed, rsp);
    Serial.print("lsp ");
    Serial.print(lsp);
    Serial.print("   ");
    Serial.print("rsp ");
    Serial.println(rsp);
}
void readLightSensor()
{
    LFSensor[0] = !digitalRead(SensorPin0);
    LFSensor[1] = !digitalRead(SensorPin1);
    LFSensor[2] = !digitalRead(SensorPin2);
    LFSensor[3] = !digitalRead(SensorPin3);
    LFSensor[4] = !digitalRead(SensorPin4);
}

void readBackSensor()
{
    BKSensor[0] = !digitalRead(BackSensor0); // left
    BKSensor[1] = !digitalRead(BackSensor1); // right
}

void testSensorValues()
{
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

        mode = CONT_LINE;

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
    else if (!LFSensor[0] && !LFSensor[1] && !LFSensor[2] && !LFSensor[3] && !LFSensor[4])
    {
        // 00000
        mode = NO_LINE;
        error = 0;
    }
    else if (!LFSensor[0] && !LFSensor[1] && !LFSensor[2] && LFSensor[3] && !LFSensor[4])
    {
        // Sensor Value
        // 0 0 0 1 0   2
        mode = FOLLOWING_LINE;
        error = 2;
    }
    else if (!LFSensor[0] && !LFSensor[1] && LFSensor[2] && LFSensor[3] && !LFSensor[4])
    {
        // Sensor Value
        // 0 0 1 1 0
        mode = FOLLOWING_LINE;
        error = 1;
    }
    else if (!LFSensor[0] && !LFSensor[1] && LFSensor[2] && !LFSensor[3] && !LFSensor[4])
    {
        // 0 0 1 0 0
        mode = FOLLOWING_LINE;
        error = 0;
    }
    else if (!LFSensor[0] && LFSensor[1] && LFSensor[2] && !LFSensor[3] && !LFSensor[4])
    {
        // 0 1 1 0 0 -1
        mode = FOLLOWING_LINE;
        error = -1;
    }
    else if (!LFSensor[0] && LFSensor[1] && !LFSensor[2] && !LFSensor[3] && !LFSensor[4])
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

void setup()
{
    Serial.begin(9600);
    pinMode(SensorPin0, INPUT);
    pinMode(SensorPin1, INPUT);
    pinMode(SensorPin2, INPUT);
    pinMode(SensorPin3, INPUT);
    pinMode(SensorPin4, INPUT);

    pinMode(BackSensor0, INPUT);
    pinMode(BackSensor1, INPUT);

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
{ // drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough

    // the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // calculateError(LFSensor[0], LFSensor[1], LFSensor[2], LFSensor[3], LFSensor[4]);
    calculateError_LF(LFSensor[0], LFSensor[1], LFSensor[2], LFSensor[3], LFSensor[4]);
    calculatePID_LF();
    motorPIDcontrol_LF();
    Serial.print("in forward");
}

void left(int speed)
{ // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(rightSpeed, speed);
    analogWrite(leftSpeed, speed);
    Serial.print("in left");
}

void right(int speed)
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(rightSpeed, speed);
    analogWrite(leftSpeed, speed);
}

void waitBefore()
{
    while (BKSensor[0] && BKSensor[1])
    {
        readBackSensor();
    }
}

void waitAfter()
{
    while (BKSensor[0] && BKSensor[1])
    {
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

        // calculatePID();
        // motorPIDcontrol();
        switch (mode)
        {
        case NO_LINE:
            Serial.println("NO_LINE");

            left(80);
            delay(50);
            while (mode != FOLLOWING_LINE)
            {
                Read_IR_sensors();
            }
            // recIntersection('B');
            break;

        case CONT_LINE:
            Serial.println("CONT_LINE");

            while (BKSensor[0] != 0)
            { // 0 -> there is left turn
                readBackSensor();
            }
            left(80);
            delay(50);
            while (mode != FOLLOWING_LINE)
            {
                Read_IR_sensors();
            }
            // recIntersection('L');
            break;

        case RIGHT_TURN:
            Serial.println("RIGHT_TURN");
            // wait for 2 sensor at least one

            while (BKSensor[1] != 0)
            { // 0 -> there is right turn
                readBackSensor();
            }
            right(80);
            delay(50);
            while (mode != FOLLOWING_LINE)
            {
                Read_IR_sensors();
            }
            // recIntersection('R');
            break;

        case LEFT_TURN:
            Serial.println("LEFT_TURN");

            while (BKSensor[0] != 0)
            { // 0 -> there is left turn
                readBackSensor();
            }
            left(80);
            delay(50);
            while (mode != FOLLOWING_LINE)
            {
                Read_IR_sensors();
            }
            // recIntersection('S');
            break;
        case FOLLOWING_LINE:
            Serial.println("Forward");
            forward();
            // recIntersection('L');
            break;
        }
        Serial.print("error : ");
        Serial.println(error);
    }
}

void recIntersection(char direction)
{
    path[pathLength] = direction; // Store the intersection in the path variable.
    pathLength++;
    simplifyPath(); // Simplify the learned path.
}

void simplifyPath()
{
    // only simplify the path if the second-to-last turn was a 'B'
    if (pathLength < 3 || path[pathLength - 2] != 'B')
        return;

    int totalAngle = 0;
    int i;
    for (i = 1; i <= 3; i++)
    {
        switch (path[pathLength - i])
        {
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
}
void mazeOptimization(void)
{
    while (!status)
    {
        Read_IR_sensors();
        switch (mode)
        {
        case FOLLOWING_LINE:
            forward();
            break;
        case CONT_LINE:
            if (pathIndex >= pathLength)
                mazeEnd();
            else
            {
                mazeTurn(path[pathIndex]);
                pathIndex++;
            }
            break;
        case LEFT_TURN:
            if (pathIndex >= pathLength)
                mazeEnd();
            else
            {
                mazeTurn(path[pathIndex]);
                pathIndex++;
            }
            break;
        case RIGHT_TURN:
            if (pathIndex >= pathLength)
                mazeEnd();
            else
            {
                mazeTurn(path[pathIndex]);
                pathIndex++;
            }
            break;
        }
    }
}
void mazeEnd(void)
{
    stopCar();
    Serial.println("The End  ==> Path: ");
    for (int i = 0; i < pathLength; i++)
        Serial.print(path[i]);
    // Serial.print(path[i]);
    Serial.println("");
    Serial.print("  pathLenght ==> ");
    Serial.println(pathLength);
    status = 1;
    mode = STOPPED;
}
void mazeTurn(char dir)
{
    switch (dir)
    {
    case 'L': // Turn Left
        Serial.println("LEFT_TURN");

        while (BKSensor[0] != 0)
        { // 0 -> there is left turn
            readBackSensor();
        }
        left(80);
        delay(50);
        while (mode != FOLLOWING_LINE)
        {
            Read_IR_sensors();
        }
        break;
    case 'R': // Turn Right
        Serial.println("LEFT_TURN");

        while (BKSensor[1] != 0)
        { // 0 -> there is left turn
            readBackSensor();
        }
        right(80);
        delay(50);
        while (mode != FOLLOWING_LINE)
        {
            Read_IR_sensors();
        }
        break;
    case 'B': // Turn Back
        Serial.println("LEFT_TURN");

        while (BKSensor[0] != 0)
        { // 0 -> there is left turn
            readBackSensor();
        }
        left(80);
        delay(50);
        while (mode != FOLLOWING_LINE)
        {
            Read_IR_sensors();
        }
        // recIntersection('S');
        break;
    case 'S': // Go Straight
        forward();
        break;
    }
}

void loop()
{
    Serial.println("Start First Pass");
    mazeSolve(); // First pass to solve the maze
    Serial.println("End First Pass");

    // delay(5000);
    // Serial.println("Starting 2nd Pass");
    // pathIndex = 0;
    // status = 0;
    // mazeOptimization(); // run the maze as fast as possible
    // Serial.println("End 2nd Pass");

    // mode = STOPPED;
    // status = 0; // 1st pass
    // pathIndex = 0;
    // pathLength = 0;
}