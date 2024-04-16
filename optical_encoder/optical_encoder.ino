// #include <avr/io.h>

// const int encoderPin = 3;  // Interrupt pin connected to LM393 output (DO)
// // unsigned long transitions = 0;  // Global variable to store transitions
// unsigned long revolutions = 0;  // Global variable to store transitions
// static int previousReading = 1;  // Initialize previous reading as HIGH



// void setup() {
//   Serial.begin(9600);
//   // Set encoderPin as input with internal pull-up (optional, if pull-up resistor used)
//   pinMode(encoderPin, INPUT_PULLUP);


//   // Clear Timer/Counter0 control register A (TCCR0A)
//   TCCR0A = 0;

//   // Set Timer/Counter0 to normal mode (WGM00, WGM01 = 0,0)
//   TCCR0A &= ~(1 << WGM01);
//   TCCR0A &= ~(1 << WGM00);

//   // Set clock source to external rising edge (CS00, CS01, CS02 = 1,0,0)
//   TCCR0B = (1 << CS00);
//   TCCR0B &= ~(1 << CS01);
//   TCCR0B &= ~(1 << CS02);

//   // Enable Timer/Counter0 overflow interrupt (TOIE0)
//   TIMSK0 |= (1 << TOIE0);

//   // Initialize serial communication for debugging (optional)
//   Serial.println("LM393 Encoder Connected");

// }

// void loop() {
//   //Print revolutions in the main loop (optional)
//   Serial.print("Revolutions: ");
//   Serial.println(revolutions);
//   // Serial.println(digitalRead(encoderPin));
//   delay(1000);  // Update rate (adjust as needed)
//   // int currentReading = digitalRead(encoderPin);

//   // if (currentReading != previousReading) {
//   //   // Transition detected (high to low or low to high)
//   //   transitions++;
//   // }

//   // previousReading = currentReading;

//   // // Print transitions (adjust as needed)
//   // Serial.print("Transitions: ");
//   // Serial.println(transitions);
//   // delay(1000);
// }

//Interrupt Service Routine (ISR) for Timer/Counter0 overflow
// ISR(TIMER0_OVF_vect) {
//   revolutions++;  // Increment revolution counter for each overflow
//   TCNT0 = 0;      // Reset Timer/Counter0 value
// }

// #include "TimerOne.h"
// unsigned int counter=0;

// int b1a = 6;  // L9110 B-1A
// int b1b = 9;  // L9110 B-1B

// void docount()  // counts from the speed sensor
// {
//   counter++;  // increase +1 the counter value
// }

// void timerIsr()
// {
//   Timer1.detachInterrupt();  //stop the timer
//   Serial.print("Motor Speed: ");
//   int rotation = (counter / 20);  // divide by number of holes in Disc
//   Serial.print(rotation,DEC);
//   Serial.println(" Rotation per seconds");
//   counter=0;  //  reset counter to zero
//   Timer1.attachInterrupt( timerIsr );  //enable the timer
// }

// void setup()
// {
//   Serial.begin(9600);

//  pinMode(b1a, OUTPUT);
//  pinMode(b1b, OUTPUT);

//   Timer1.initialize(1000000); // set timer for 1sec
//   attachInterrupt(0, docount, RISING);  // increase counter when speed sensor pin goes High
//   Timer1.attachInterrupt( timerIsr ); // enable the timer
// }

// void loop()
// {
//   int potvalue = analogRead(1);  // Potentiometer connected to Pin A1
//   int motorspeed = map(potvalue, 0, 680, 255, 0);
//   analogWrite(b1a, motorspeed);  // set speed of motor (0-255)
//   digitalWrite(b1b, 1);  // set rotation of motor to Clockwise
// }

// volatile long temp1, temp2, counter1=0, counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder

// void setup() {
//   Serial.begin(9600);

//   pinMode(4, INPUT_PULLUP);  // internal pullup input pin 2

//   pinMode(3, INPUT_PULLUP);  // internalเป็น pullup input pin 3
//                              //Setting up interrupt
//   //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
//   attachInterrupt(0, ai0, RISING);

//   //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
//   attachInterrupt(1, ai1, RISING);
// }

// void loop() {
//   // Send the value of counter
//   if (counter1 != temp1) {
//     Serial.print("counter1 ");
//     Serial.println(counter1);
//     temp1 = counter1;
//   }
//   if (counter2 != temp2) {
//     Serial.print("counter2 ");
//     Serial.println(counter2);
//     temp2 = counter2;
//   }
// }

// void ai0() {
//   // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
//   // Check pin 3 to determine the direction
//   // if (digitalRead(3) == LOW) {
//     counter1++;
//   // }
//   // else counter1--;
// }

// void ai1() {
//   // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
//   // Check with pin 2 to determine the direction
//   // if (digitalRead(4) == LOW) {
//     counter2++;
//   // }
//   // else counter2--;
// }

volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
    
void setup() {
  Serial.begin (9600);

  pinMode(4, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
   //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
  }
   
  void loop() {
  // Send the value of counter
  if( counter != temp ){
  Serial.println (counter);
  temp = counter;
  }
  }
   
  void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  Serial.println("hello world");
  if(digitalRead(3)==LOW) {
  Serial.println("hello worldif");

  counter++;}
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(4)==LOW) {
  counter--;
  }
  }
