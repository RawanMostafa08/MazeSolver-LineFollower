#include <avr/io.h>
#include <avr/interrupt.h>
#define SET_BIT(var, pos) ((var) |= (1 << (pos)))
#define CLEAR_BIT(var, pos) ((var) &= ~(1 << (pos)))
#define TOGGLE_BIT(var, pos) ((var) ^= (1 << (pos)))
#define enA 11
#define enB 10
#define in1 8
#define in2 7
#define in3 6
#define in4 2
void initTimer0();
void initTimer1();
int timer0OVFCount = 0;
int timer1OVFCount = 0;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    pinMode(4, INPUT); // timer0 for right wheel
    pinMode(5, INPUT); // timer1 for left wheel
    int pwmOutput = 255;
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    analogWrite(enB, pwmOutput); // Send PWM signal to L298N Enable pin

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // _delay_ms(100);
    
    initTimer0();
    initTimer1();
    // enable global interrupts
    sei();
    timer0OVFCount = 0;
    timer1OVFCount = 0;
}

void loop()
{
    int synchronizedTimer0Value = (timer0OVFCount * 256) + TCNT0;
    int synchronizedTimer1Value = (timer1OVFCount * 256) + TCNT1;
    Serial.print("Counter0 Value:");
    Serial.print(TCNT0);
    Serial.print("    Counter1 Value:");
    Serial.print(TCNT1);
    Serial.print("    sync0 Value:");
    Serial.print(synchronizedTimer0Value);
    Serial.print("    sync1 Value:");
    Serial.println(synchronizedTimer1Value);
}

void initTimer0()
{
    // rising edge
    TCNT0 = 0;
    SET_BIT(TCCR0B, CS00);
    SET_BIT(TCCR0B, CS01);
    SET_BIT(TCCR0B, CS02);
    TIMSK0 |= (1 << OCIE0A); // enable CTC interrupt
    // Non PWM mode FOC2A=1
    TCCR0B |= (1 << FOC0B);
    TCCR0A |= (1 << WGM01); // CTC Mode
    OCR0A = 255;
}
void initTimer1()
{
    // rising edge
    SET_BIT(TCCR1B, CS00);
    SET_BIT(TCCR1B, CS01);
    SET_BIT(TCCR1B, CS02);
    TCCR1A = 0x00;           // enable 16 bit counter
    TIMSK1 |= (1 << OCIE1A); // enable CTC interrupt
    // Non PWM mode FOC2A=1
    TCCR1C |= (1 << FOC1A);
    TCCR1B |= (1 << WGM12); // CTC Mode
    OCR1A = 255;
    TCNT1 = 0;

}

ISR(TIMER0_COMPA_vect)
{
    timer0OVFCount++;
}

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;
    timer1OVFCount++;
}
