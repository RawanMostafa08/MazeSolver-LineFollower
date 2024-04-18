#include <avr/io.h>
#include <avr/interrupt.h>
#define SET_BIT(var, pos) ((var) |= (1 << (pos)))
#define CLEAR_BIT(var, pos) ((var) &= ~(1 << (pos)))
#define TOGGLE_BIT(var, pos) ((var) ^= (1 << (pos)))
#define enA 11 // left-->timer0
#define enB 10 // right-->timer1
#define in1 8
#define in2 7
#define in3 6
#define in4 2
void initTimer0();
void initTimer1();
void adjustMotors(int, int);
int timer0OVFCount = 0;
int timer1OVFCount = 0;
int prevTCNT0 = 0;
int pwmOutputA = 255;
int pwmOutputB = 255;

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

    pinMode(4, INPUT);            // timer0 for right wheel
    pinMode(5, INPUT);            // timer1 for left wheel
    analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
    analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin

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
    if (timer0OVFCount == 0)
    {
        if (prevTCNT0 > TCNT0)
            timer0OVFCount++;
        prevTCNT0 = TCNT0;
    }
    int synchronizedTimer0Value = (timer0OVFCount * 256) + TCNT0;
    int synchronizedTimer1Value = (timer1OVFCount * 256) + TCNT1;
    // Serial.print("Counter0 Value:");
    // Serial.print(TCNT0);
    // Serial.print("    Counter1 Value:");
    // Serial.print(TCNT1);
    // Serial.print("    sync0 Value:");
    // Serial.print(synchronizedTimer0Value);
    // Serial.print("    sync1 Value:");
    // Serial.print(synchronizedTimer1Value);
    // Serial.print("   ovf0 ");
    // Serial.println(timer0OVFCount);
    // PID gains and computation
    float kp = 2.0;
    float kd = 0.0;
    float ki = 0.0;
    int error=synchronizedTimer0Value-synchronizedTimer1Value;
    float u = pidController(error, kp, kd, ki);
    // Control motor 2 based on PID
    moveMotor(enB, u);

}

void initTimer0()
{
    TIMSK0 |= (1 << OCIE0A); // enable CTC interrupt
    // rising edge
    SET_BIT(TCCR0B, CS00);
    SET_BIT(TCCR0B, CS01);
    SET_BIT(TCCR0B, CS02);
    // Non PWM mode FOC2A=1
    TCCR0B |= (1 << FOC0A);
    TCCR0A |= (1 << WGM01); // CTC Mode
    OCR0A = 255;
    TCNT0 = 0;
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
    // TCCR1C |= (1 << FOC1A);
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
void adjustMotors(int encoderValueA, int encoderValueB)
{
    // Calculate speed error
    // Serial.println(encoderValueA);
    // Serial.println(encoderValueB);

    int speedError = encoderValueA - encoderValueB;

    // Adjust motor speeds based on speed error
    if (speedError > 0)
    {
        pwmOutputB += speedError; // Increase PWM for motor B
    }
    else if (speedError < 0)
    {
        pwmOutputA -= speedError; // Increase PWM for motor A
    }

    Serial.print("pwmA ");
    Serial.print(pwmOutputA);
    Serial.print("    pwmB ");
    Serial.print(pwmOutputB);
    Serial.print("    error ");
    Serial.println(speedError);
    // Apply PWM outputs to motors
    analogWrite(enA, pwmOutputA);
    analogWrite(enB, pwmOutputB);
}

// Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

void moveMotor( int pwmPin, float u)
{
    // Maximum motor speed
    float speed = fabs(u);
    if (speed > 255)
    {
        speed = 255;
    }
    // Stop the motor during overshoot
    // if (encoder2Count > encoder1Count)
    // {
    //     speed = 0;
    // }

    analogWrite(pwmPin, speed);
}

float pidController(int e, float kp, float kd, float ki)
{

    // Compute the error, derivative, and integral
    float eDerivative = (e - ePrevious) ;
    eIntegral = eIntegral + e;

    // Compute the PID control signal
    float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

    // Update variables for the next iteration
    ePrevious = e;
    return u;
}