
const int SensorPin0 = 5;
const int SensorPin1 = 9;
const int SensorPin2 = A3;
const int SensorPin3 = A2;
const int SensorPin4 = A1;

int error = 0;
int LFSensor[5] = {0, 0, 0, 0, 0};

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define RIGHT_TURN 4
#define LEFT_TURN 5

int mode = STOPPED;
int status = false;
void readLightSensor()
{
    LFSensor[0] = !digitalRead(SensorPin0);
    LFSensor[1] = !digitalRead(SensorPin1);
    LFSensor[2] = !digitalRead(SensorPin2);
    LFSensor[3] = !digitalRead(SensorPin3);
    LFSensor[4] = !digitalRead(SensorPin4);
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
        Serial.print("hi   ");
        Serial.println(mode);
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
            break;

        case CONT_LINE:
            Serial.println("CONT_LINE");
            break;

        case RIGHT_TURN:
            Serial.println("RIGHT_TURN");
            break;

        case LEFT_TURN:
            Serial.println("LEFT_TURN");
            break;
        }
    }
}
void loop()
{
    // readLightSensor();
    // testSensorValues();
    mazeSolve();
}