const int motorLeftA = 2;      // Digital pin for left motor A
const int motorLeftB = 6;     // Digital pin for left motor B
const int motorRightA = 7;     // Digital pin for right motor A
const int motorRightB = 8;     // Digital pin for right motor B

const int led = 9;

  // Read sensor values
const int sensorPin[5] = {A1, A2, A3, A4, A5}; // Array of digital pins for the sensors
const int numSensors = 5; // Number of sensors

#define LEFT_MOTOR_PIN 11
#define RIGHT_MOTOR_PIN 10

//BEST RESULT FOR SPEED 30
// double Kp = 3.8;
// double Ki = 0.000;
// double Kd = 0.1;

//BEST RESULT FOR SPEED 100
// double Kp = 17;
// double Ki = 0.000;
// double Kd = 0.3;

//SPEED 100
// Max Kp in P controller = 18
double Kp = 9;
double Ki = 0;
double Kd = 50;


//  11/5 
//  speed=50
//  kp=9
//  kd=50
//  ki=0

//CURRECT PID PARAMETERS
// KP = 7
// KI = 0
// KD = 0.3
//SPEED = 45


//CURRECT NEW PID PARAMETERS
// KP = 7.8
// KI = 0
// KD = 10
//SPEED = 45
int baseSpeed = 50;


// char Incoming_value = 0;
// void setup() {
//   // put your setup code  here, to run once:
// Serial.begin(9600);
// pinMode(13,OUTPUT);
// }

// void  loop() {
//   if (Serial.available()  > 0)
//     {
//       Incoming_value = Serial.read();
//       Serial.print("/n");
//       if (Incoming_value == '1')
//         digitalWrite(13,HIGH);
//     }
// }


double error, integral = 0.0, derivative, output;
double previous_error = 0.0; // Initialize previous error for derivative calculation

void setup() {
  pinMode(led, OUTPUT);

  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    
    
  digitalWrite(motorRightA,HIGH);
  digitalWrite(motorRightB,LOW);
  digitalWrite(motorLeftA,HIGH);
  digitalWrite(motorLeftB,LOW);

  digitalWrite(led, LOW);


  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  Serial.begin(9600); // Start serial communication for debugging (optional)
    analogWrite(LEFT_MOTOR_PIN,30);
  analogWrite(RIGHT_MOTOR_PIN,30);

}

void loop() {

  if (Serial.available()) {
    String command = Serial.readStringUntil(' '); // Read until space
    if (command == "kp") {
      Kp = Serial.parseFloat(); // Parse the float value after the command
      Serial.print("Set Kp to: ");
      Serial.println(Kp);
    } else if (command == "ki") {
      Ki = Serial.parseFloat();
      Serial.print("Set Ki to: ");
      Serial.println(Ki);
    } else if (command == "kd") {
      Kd = Serial.parseFloat();
      Serial.print("Set Kd to: ");
      Serial.println(Kd);
      } else if (command == "speed") {
      baseSpeed = Serial.parseFloat();
      Serial.print("Set Speed to: ");
      Serial.println(baseSpeed);
      } else if (command == "s") {
      // Stop the motors
      StopMotors();
      Serial.println("Motors stopped");
    } else if (command == "m") {
      // Start the motors
      MoveMotors();
      Serial.println("Motors started");
    } else if (command == "0") {
      digitalWrite(led, LOW);
      // Start the motors
      Serial.println("0 recieved");
    } else if (command == "1") {
      // Start the motors
            digitalWrite(led, HIGH);

      Serial.println("1 recieved");
    }
      else {
      // Invalid command
      Serial.println("Invalid command");
    }

  }

  // Read sensor values
  int sensorValues[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = digitalRead(sensorPin[i]);
  }
    // Serial.print("  Sensors value: ");
    // Serial.print(sensorValues[0]);
    // Serial.print(sensorValues[1]);
    // Serial.print(sensorValues[2]);
    // Serial.print(sensorValues[3]);
    // Serial.print(sensorValues[4]);


if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 1)
    error = 0;
else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1)
    error = 1;
else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 1)
    error = 2;
else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0)
    error = 3;
else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 0)
    error = 4;
else if (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 1)
    error = -1;
else if (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)
    error = -2;
else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)
    error = -3;
else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)
    error = -4;
// while (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)
// { 
//   StopMotors();
//     for (int i = 0; i < numSensors; i++) {
//     sensorValues[i] = digitalRead(sensorPin[i]);
//   }
//     // Serial.print("  Sensors value: ");
//     // Serial.print(sensorValues[0]);
//     // Serial.print(sensorValues[1]);
//     // Serial.print(sensorValues[2]);
//     // Serial.print(sensorValues[3]);
//     // Serial.println(sensorValues[4]);
  
// }

// MoveMotors();
// if (!(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)){ //if not all white
//   integral = 0;
// }
  
  // Update integral and derivative
  integral += error; // Assuming constant time step (adjust if needed)
  derivative = (error - previous_error); // No need to divide by dt if time step is constant
  
  // Calculate output (control signal for motor speed)
  output = Kp * error + Ki * integral + Kd * derivative;

  previous_error = error;
  // Apply motor control based on output and error
  controlMotors(output, error);

  // Optional: Print sensor values and motor speed for debugging

}

void controlMotors(double speed, double error) {
  int leftSpeed, rightSpeed;

    leftSpeed = baseSpeed - (speed);  // Adjust scaling factor for left speed reduction
    rightSpeed = baseSpeed + (speed); // Adjust scaling factor for right speed increase
  if(leftSpeed > 255)
  leftSpeed = 255;
  if(leftSpeed < 0)
  leftSpeed = 0;
  if(rightSpeed > 255)
  rightSpeed = 255;
  if(rightSpeed < 0)
  rightSpeed = 0;

  analogWrite(LEFT_MOTOR_PIN, leftSpeed);
  analogWrite(RIGHT_MOTOR_PIN,rightSpeed);
    // Optional: Print sensor values and motor speed for debugging
          Serial.print("  Kp= ");
  Serial.print(Kp);
        Serial.print("  Ki= ");
  Serial.print(Ki);
        Serial.print("  Kd= ");
  Serial.print(Kd);
  //     Serial.print("  Error: ");
  // Serial.print(error);
  Serial.print("  Right= ");
  Serial.print(leftSpeed);
  Serial.print("  Left=");
  Serial.println(rightSpeed);

}

void StopMotors(){
 digitalWrite(motorRightA,LOW);
  digitalWrite(motorRightB,LOW);
  digitalWrite(motorLeftA,LOW);
  digitalWrite(motorLeftB,LOW);
}
void MoveMotors(){
 digitalWrite(motorRightA,HIGH);
  digitalWrite(motorRightB,LOW);
  digitalWrite(motorLeftA,HIGH);
  digitalWrite(motorLeftB,LOW);
}