#ifndef SENSOR_H
#define SENSOR_H

#include "hi229.h"


#define sensor_left     11
#define sensor_front    12
#define sensor_right    A0

#define rear_left       A7
#define rear_right      A6


void setup_sensor(){
    pinMode(sensor_left, INPUT_PULLUP);
    pinMode(sensor_front, INPUT_PULLUP);
    pinMode(sensor_right, INPUT_PULLUP);

    pinMode(rear_left, INPUT);
    pinMode(rear_right, INPUT);
}

// analogRead
// digitalRead
bool is_left_empty(){
    return digitalRead(sensor_left);
}
bool is_front_empty(){
    return digitalRead(sensor_front);
}
bool is_right_empty(){
    return digitalRead(sensor_right);
}
bool is_left_rear(){
    return analogRead(rear_left) < 512;
}
bool is_right_rear(){
    return analogRead(rear_right) < 512;
}


void read_sensor(){
    Serial.print(digitalRead(sensor_left));
    Serial.print(' ');
    Serial.print(digitalRead(sensor_front));
    Serial.print(' ');
    Serial.print(digitalRead(sensor_right));
    Serial.print(' ');
    Serial.print(is_left_rear());
    Serial.print(' ');
    Serial.println(is_right_rear());
}


#endif