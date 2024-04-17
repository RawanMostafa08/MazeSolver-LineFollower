  // put your setup code here, to run once:
  // IN1 -> d8 --- IN4 -> d5
  e:\CMP3\second_semester\Embedded\project\optical_encoder3\optical_encoder3.ino// ENA -> d11
  // ENB -> d10

#define enA 11
#define enB 10
#define in1 8
#define in2 7
#define in3 6
#define in4 2

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT); 

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);

  Serial.begin(9600);

  int pwmOutput = 255 ;
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutput); // Send PWM signal to L298N Enable pin

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void loop() {
  

  float s0 = analogRead(A0);
  float s1 = analogRead(A1);
  float s2 = analogRead(A2);
  float s3 = analogRead(A3);
  float s4 = analogRead(A4);

  Serial.print(s0);
  Serial.print("\t");
  Serial.print(s1);
  Serial.print("\t");
  Serial.print(s2);
  Serial.print("\t");
  Serial.print(s3);
  Serial.print("\t");
  Serial.print(s4);
  Serial.println();
  delay(20);
}