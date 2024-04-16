// #include <avr/interrupt.h>

// const int encoderPin = 4;  // Interrupt pin connected to LM393 output (DO)
// // volatile unsigned long revolutions = 0;  // Global variable to store revolutions
// volatile unsigned char currentTCNT0 = 0;  // Stores the current value of TCNT0

// void setup() {
//   Serial.begin(9600);
//   // Set encoderPin as input with internal pull-up (optional, if pull-up resistor used)
//   // pinMode(encoderPin, INPUT_PULLUP);
//   DDRD &= ~(1<<PD4);

//   // Clear Timer/Counter0 control register A (TCCR0A)
//   TCCR0A = 0;
//   TCNT0 = 0;


//   // Set Timer/Counter0 to normal mode (WGM00, WGM01 = 0,0)
//   TCCR0A &= ~(1 << WGM01);
//   TCCR0A &= ~(1 << WGM00);

//   // Set clock source to external rising edge (CS00, CS01, CS02 = 1,0,0)
//   TCCR0B |= (1 << CS00);
//   TCCR0B |= (1 << CS01);
//   TCCR0B |= (1 << CS02);

//   // Enable Timer/Counter0 overflow interrupt (TOIE0)
//   // TIMSK0 |= (1 << TOIE0);

//   // Initialize serial communication for debugging (optional)
//   Serial.println("LM393 Encoder Connected");
// }

// void loop() {
//   currentTCNT0 = TCNT0;  // Read TCNT0 value and store it in a volatile variable
//   Serial.print("Current TCNT0: ");
//   Serial.println(TCNT0);
//   // delay(1000);  // Update rate (adjust as needed)
// }






#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t previousCapture = 0;

void setup() {
  Serial.begin(9600);

  // Configure ICP1 pin (PB0) as input
  DDRB &= ~(1 << PD4);

  // Clear Timer/Counter1 register
  TCNT1 = 0;

  // Clear Input Capture Flag
  TIFR1 |= (1 << ICF1);

  // Configure capture on rising edge
  TCCR1B |= (1 << ICES1);

  // Enable Input Capture Interrupt
  TIMSK1 |= (1 << ICIE1);

  // Enable global interrupts
  sei();
}

void loop() {
  // Do nothing in loop, all functionality is handled by interrupts
}

ISR(TIMER1_CAPT_vect) {
  if (bit_is_set(TIFR1, ICF1)) { // Check if Input Capture Flag is set
    // Capture current time in ICR1
    uint16_t recentCapture = ICR1;

    // Calculate period
    uint16_t period = recentCapture - previousCapture;

    // Print period count on serial monitor
    Serial.println(period);

    // Store recent capture for next calculation
    previousCapture = recentCapture;

    // Clear capture flag
    TIFR1 |= (1 << ICF1);
  }
}
