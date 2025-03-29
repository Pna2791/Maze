#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


#define N_COLS 22
#define N_ROWS 14


#define ENCODER_OFFSET  8
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


#define EEPROM_START_X  0
#define EEPROM_START_Y  1
#define EEPROM_GOAL_X   2
#define EEPROM_GOAL_Y   3

#define EEPROM_LOCKED   4
#define EEPROM_LONGEST  5


#define debounce_time 300


byte start_x = 11;
byte start_y = 14;
byte goal_x = 11;
byte goal_y = 0;

bool locked_mode = 0;
int longest_step = 0;


void load_start_goal(){
    start_x = EEPROM.read(EEPROM_START_X);
    start_y = EEPROM.read(EEPROM_START_Y);
    goal_x  = EEPROM.read(EEPROM_GOAL_X);
    goal_y  = EEPROM.read(EEPROM_GOAL_Y);
    locked_mode = (EEPROM.read(EEPROM_LOCKED) != 0);
    longest_step = EEPROM.read(EEPROM_LONGEST);

    Serial.print(F("locked_mode :"));
    Serial.println(locked_mode);
    // if(longest_step > 1)    longest_step--;
}

void update_locked_mode(){
    if(locked_mode) EEPROM.update(EEPROM_LOCKED,  1);
    else            EEPROM.update(EEPROM_LOCKED,  0);
}

void update_longest_step(){
    EEPROM.update(EEPROM_LONGEST,  longest_step);
}


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


#define CHECKPOINT_1        3
#define CHECKPOINT_2        10

// MENU MODE
int menu_mode = 0;
#define N_MODES             15
#define CHECKPOINT_LENGTH   20

#define LEFT_RUN_0_MODE     0
#define LEFT_RUN_1_MODE     2
#define LEFT_RUN_2_MODE     4
#define LEFT_TRACE_MODE     6

#define RIGHT_RUN_0_MODE    1
#define RIGHT_RUN_1_MODE    3
#define RIGHT_RUN_2_MODE    5
#define RIGHT_TRACE_MODE    7

#define TEST_LINE_MODE      8
#define RESET_IMU_MODE      9

#define SET_START_X         10
#define SET_START_Y         11
#define SET_GOAL_X          12
#define SET_GOAL_Y          13

#define SET_LOCK_MODE       14


#endif
