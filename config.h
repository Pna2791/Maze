#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


#define N_COLS 22
#define N_ROWS 14


#define safe_distance 13
#define cell_size 26

#define plush_per_cm 2


#define btt_top A3
#define btt_ent A2
#define btt_bot A1

void setup_button(){
    pinMode(btt_top, INPUT_PULLUP);
    pinMode(btt_ent, INPUT_PULLUP);
    pinMode(btt_bot, INPUT_PULLUP);
}


#define EEPROM_START_X 0
#define EEPROM_START_Y 1
#define EEPROM_GOAL_X  2
#define EEPROM_GOAL_Y  3


#define n_modes 10
#define debounce_time 300



byte start_x = 11;
byte start_y = 14;
byte goal_x = 11;
byte goal_y = 0;


void load_start_goal(){
    start_x = EEPROM.read(EEPROM_START_X);
    start_y = EEPROM.read(EEPROM_START_Y);
    goal_x  = EEPROM.read(EEPROM_GOAL_X);
    goal_y  = EEPROM.read(EEPROM_GOAL_Y);
}



#define ENCODER_OFFSET  5
#define encoder_A       2  // Must be an interrupt-capable pin
#define encoder_B       3  // Must also be an interrupt-capable pin

long encoderCount = 0;
void encoderISR_A() {
    if (digitalRead(encoder_A) == digitalRead(encoder_B)) {
        encoderCount++;  // Clockwise
    } else {
        encoderCount--;  // Counterclockwise
    }
}

void encoderISR_B() {
    if (digitalRead(encoder_A) != digitalRead(encoder_B)) {
        encoderCount++;  // Clockwise
    } else {
        encoderCount--;  // Counterclockwise
    }
}

void encoderISR(){
    encoderCount++;
}

void setup_encoder(){
    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);

    // attachInterrupt(digitalPinToInterrupt(encoder_B), encoderISR, RISING);

    // attachInterrupt(digitalPinToInterrupt(encoder_A), encoderISR_A, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(encoder_B), encoderISR_B, CHANGE);

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
