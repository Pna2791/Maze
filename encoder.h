#ifndef ENCODER_H
#define ENCODER_H


#define encoder_A       2  // Must be an interrupt-capable pin
#define encoder_B       3  // Must also be an interrupt-capable pin

long encoderCount = 0;

void setup_encoder(){
    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);

    // Configure Timer2 for 1ms interrupt
    cli(); // Disable interrupts

    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22);   // Prescaler 64
    OCR2A = 1249;            // Compare match for 1ms (16MHz / 64 / 250 = 1kHz)
    TIMSK2 = (1 << OCIE2A); // Enable Timer2 compare match interrupt

    sei(); // Enable interrupts
}

ISR(TIMER2_COMPA_vect) {
    static bool previous_stt = 0;
    bool stt = digitalRead(encoder_B);
    if(stt != previous_stt){
        previous_stt = stt;
        encoderCount++;
    }
}


#endif
