// #include "OTA_utils.h"

#define led_blink 26

#define left_A 33
#define left_B 25
#define left_E 32

#define right_A 14
#define right_B 27
#define right_E 12

#define RXD2 16   // Example pin for UART RX
#define TXD2 17   // Example pin for UART TX
#define PACKET_LENGTH 13

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

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


PIController forward_pid = PIController(2, 0.1);


void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Serial2 for UART input
    
    Serial.println("ESP32 UART listening...");
    SerialBT.begin("Maze"); // Set the Bluetooth device name
    pinMode(led_blink, OUTPUT);

    pinMode(left_A, OUTPUT);
    pinMode(left_B, OUTPUT);
    pinMode(left_E, OUTPUT);

    pinMode(right_A, OUTPUT);
    pinMode(right_B, OUTPUT);
    pinMode(right_E, OUTPUT);

    // START_OTA_MODE();
}

void loop() {
    // server.handleClient();
    // ElegantOTA.loop();

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


void test_forward(){
    long time_out = millis() + 10000;
    while(millis() < time_out){
        int direction = get_direction();
        if(direction != 0xFFF){
            int forward_value = forward_pid.compute(0, direction);
            move_forward(forward_value);

            Serial.print("direction: ");
            Serial.print(direction);
            Serial.print(", forward_value: ");
            Serial.println(forward_value);  // println here prints both values and ends the line
        }
    }
}


void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();
    
    if (command.startsWith("M")) {
        int direction = command.substring(1).toInt();
        if(direction == 0){
            stop_move();
        }else if(direction == 1){
            test_forward();
            stop_move();
            // left_speed(255);
            // right_speed(255);
        }else if(direction == 2){
            left_speed(-255);
            right_speed(-255);
        }else if(direction == 3){
            left_speed(-255);
            right_speed(255);
        }else if(direction == 4){
            left_speed(255);
            right_speed(-255);
        }else if(direction == 5){
            left_speed(0);
            right_speed(255);
        }else if(direction == 6){
            left_speed(255);
            right_speed(0);
        }
    }
}
