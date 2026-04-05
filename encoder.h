#ifndef ENCODER_H
#define ENCODER_H

#define encoder_A       2  // Should be interrupt-capable pin
#define encoder_B       3  // Also interrupt-capable

volatile long encoderCount = 0;

void setup_encoder(){
    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);

    // Configure Timer1 for 1ms interrupt
    cli(); // Disable global interrupts

    TCCR1A = 0;              // Normal operation
    TCCR1B = 0;

    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64

    OCR1A = 249; // Compare match value for 1ms (16MHz / 64 / 1000 = 250 - 1)

    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt

    sei(); // Enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
    static bool previous_stt = 0;
    bool stt = digitalRead(encoder_B);
    if(stt != previous_stt){
        previous_stt = stt;
        encoderCount++;
    }
}

#endif
