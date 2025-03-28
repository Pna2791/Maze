#ifndef WHEEL_H
#define WHEEL_H


#define left_A 6
#define left_B 7
#define left_E 5

#define right_A 9
#define right_B 8
#define right_E 10


#define FW_SPEED    200
#define TURN_SPEED  255

void left_speed(int val){
    if(val==0){
        digitalWrite(left_E, 0);
        digitalWrite(left_A, 0);
        digitalWrite(left_B, 0);
        return;
    }
    if(val > 0){
        digitalWrite(left_A, 0);
        digitalWrite(left_B, 1);
    }
    if(val < 0){
        digitalWrite(left_A, 1);
        digitalWrite(left_B, 0);
    }
    if(abs(val) == 255)     digitalWrite(left_E, 1);
    else                    analogWrite(left_E, abs(val));
}


void right_speed(int val){
    if(val == 0){
        digitalWrite(right_E, 0);
        digitalWrite(right_A, 0);
        digitalWrite(right_B, 0);
        return;
    }
    if(val > 0){
        digitalWrite(right_A, 0);
        digitalWrite(right_B, 1);
    }
    if(val < 0){
        digitalWrite(right_A, 1);
        digitalWrite(right_B, 0);
    }
    if(abs(val) == 255)     digitalWrite(right_E, 1);
    else                    analogWrite(right_E, abs(val));
}


void move_speed(int val){
    left_speed(val);
    right_speed(val);
}

void move_forward(int val, int reduce=0){
    if(val == 0){
        left_speed(FW_SPEED);
        right_speed(FW_SPEED);
    }else if(val > 0){
        left_speed(FW_SPEED-val);
        right_speed(FW_SPEED);
    }else{
        left_speed(FW_SPEED);
        right_speed(FW_SPEED+val);
    }
}


void rotate_CCW(int val){
    left_speed(-val);
    right_speed(val);
}


void stop_move(){
    move_speed(0);
}


void setup_wheel(){
    pinMode(left_A, OUTPUT);
    pinMode(left_B, OUTPUT);
    pinMode(left_E, OUTPUT);

    pinMode(right_A, OUTPUT);
    pinMode(right_B, OUTPUT);
    pinMode(right_E, OUTPUT);

    stop_move();
}


#endif