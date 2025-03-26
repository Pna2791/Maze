#define led_blink 26

// 30128 plushs for 133cm
// 226 plush/cm => 1132 plush for 5cm
#define plush_per_cm 182
#define encoder_A 34
#define encoder_B 35

#define left_A 33
#define left_B 25
#define left_E 32

#define right_A 14
#define right_B 27
#define right_E 12

#define RXD2 16   // Example pin for UART RX
#define TXD2 17   // Example pin for UART TX
#define PACKET_LENGTH 13

#define sensor_left 26
#define sensor_front 19
#define sensor_right 22



#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

int global_dir = 0;

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
    analogWrite(left_E, abs(val));
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
    analogWrite(right_E, abs(val));
}


void move_forward(int val){
    if(val == 0){
        left_speed(255);
        right_speed(255);
    }else if(val > 0){
        left_speed(255-val);
        right_speed(255);
    }else{
        left_speed(255);
        right_speed(255+val);
    }
}


void rotate_CCW(int val){
    left_speed(-val);
    right_speed(val);
}


class PIController {
  public:
    PIController(float kp, float ki) {
        this->kp = kp;
        this->ki = ki;
        prevError = 0;
        integral = 0;
        outputMin = -127;
        outputMax = 127;
    }

    void setOutputLimits(float minVal, float maxVal) {
        outputMin = minVal;
        outputMax = maxVal;
    }

    void reset(){
        prevError = 0;
        integral = 0;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        integral += error;
        integral = constrain(integral, -90, 90);
        float output = (kp * error) + (ki * integral);

        prevError = error;
        return constrain(output, outputMin, outputMax);
    }

  private:
    float kp, ki;
    float prevError;
    float integral;
    float outputMin, outputMax;
};


PIController forward_pid = PIController(2, 0.2);
PIController turn_pid = PIController(3, 0.3);


long encoderCount = 0; 
void IRAM_ATTR encoderISR() {    // ISR (Interrupt Service Routine)
    if(!digitalRead(encoder_B))  encoderCount++;
    else                        encoderCount--;
}


void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Serial2 for UART input
    Serial2.println("AT+RST");
    
    Serial.println("ESP32 UART listening...");
    SerialBT.begin("Maze"); // Set the Bluetooth device name
    pinMode(led_blink, OUTPUT);

    pinMode(left_A, OUTPUT);
    pinMode(left_B, OUTPUT);
    pinMode(left_E, OUTPUT);

    pinMode(right_A, OUTPUT);
    pinMode(right_B, OUTPUT);
    pinMode(right_E, OUTPUT);

    pinMode(sensor_left, INPUT_PULLUP);
    pinMode(sensor_front, INPUT_PULLUP);
    pinMode(sensor_right, INPUT_PULLUP);


    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder_A), encoderISR, RISING); 

    turn_pid.setOutputLimits(-255, 255);
}

void read_sensor(){
    Serial.print(digitalRead(sensor_left));
    Serial.print(digitalRead(sensor_front));
    Serial.println(digitalRead(sensor_right));
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }

    // Check for serial commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        processSerialCommand(command);
    }
    int direction = get_direction();
    if(direction != 0xFFF){
        Serial.println(direction);
        read_sensor();
        Serial.println(encoderCount);
        Serial.println(global_dir);
    }
}


int get_direction() {
    static uint8_t previousByte = 0;
    static int count = 0;
    static int angle = 0;

    if(Serial2.available()) {
        uint8_t byteInt = Serial2.read();

        if (byteInt == 0xA5 && previousByte == 0x5A) {
            count = 2;  // Detected start sequence
        } else {
            count++;
            if (count == 13) {
                angle = byteInt * 256 + previousByte;  // Combine last two bytes
                if (angle > 0x7FFF){
                    angle = 0xFFFF - angle;
                    angle = -angle;
                }
                count = 0;  // Reset to look for the next packet
                previousByte = byteInt;
                return angle;
            }
        }
        previousByte = byteInt;
    }
    return 0xFFF;
}


void stop_move(){
    left_speed(0);
    right_speed(0);
}


void turn_forward(int value=25){
    forward_pid.reset();
    long next_checkpoint = encoderCount + plush_per_cm*value;
    while(encoderCount < next_checkpoint){
        int direction = get_direction();
        if(direction != 0xFFF){
            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int forward_value = forward_pid.compute(global_dir, direction);
            move_forward(forward_value);

            Serial.print("target: ");
            Serial.print(global_dir);
            Serial.print("direction: ");
            Serial.print(direction);
            Serial.print(", forward_value: ");
            Serial.println(forward_value);  // println here prints both values and ends the line
        }
    }
}


void turn_CCW(int delta){
    Serial.println("TURN CCW");
    global_dir = (global_dir+3600)%3600;
    global_dir += delta;
    if(global_dir > 2250)   global_dir -= 3600;

    turn_pid.reset();
    while(true){
        int direction = get_direction();
        if(direction != 0xFFF){
            if(abs(global_dir-direction)%3600 < 50) break;

            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int turn_value = turn_pid.compute(global_dir, direction);
            rotate_CCW(turn_value);

            Serial.print("target: ");
            Serial.print(global_dir);
            Serial.print("direction: ");
            Serial.print(direction);
            Serial.print(", turn_value: ");
            Serial.println(turn_value);  // println here prints both values and ends the line
        }
    }
}


void left_wall_following(){
    if(digitalRead(sensor_left)){
        turn_CCW(900);
    }else if(digitalRead(sensor_front)){
        // Forward
    }else if(digitalRead(sensor_right)){
        turn_CCW(-900);
    }else{
        turn_CCW(1800);
    }

    stop_move();
    turn_forward(30);
    stop_move();
}

void right_wall_following(){
    if(digitalRead(sensor_right)){
        turn_CCW(-900);
    }else if(digitalRead(sensor_front)){
        // Just forward
    }else if(digitalRead(sensor_left)){
        turn_CCW(900);
    }else{
        turn_CCW(1800);
    }

    stop_move();
    turn_forward(30);
    stop_move();
}

void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();
    
    if (command.startsWith("M")) {
        int direction = command.substring(1).toInt();
        if(direction == 0){
            stop_move();
        }else if(direction == 1){
            // turn_forward();
            // stop_move();
        }else if(direction == 2){
            turn_CCW(1800);
            stop_move();
        }else if(direction == 3){
            turn_CCW(900);
            stop_move();
        }else if(direction == 4){
            turn_CCW(-900);
            stop_move();
        }

        if(direction != 0){
            turn_forward(30);
            stop_move();
        }
    }


    if (command.startsWith("C")) {
        int direction = command.substring(1).toInt();
        if(direction == 1){
            for(int i=0; i<16; i++) left_wall_following();
        }
        if(direction == 2){
            for(int i=0; i<16; i++) right_wall_following();
        }
    }
}
