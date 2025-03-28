#ifndef PID_CONTROL_H
#define PID_CONTROL_H


#define FREQ    25

#define P_pin   A6
#define D_pin   A7


int advance_mode = 0;
void setup_pid_input(){
    pinMode(P_pin, INPUT);
    pinMode(D_pin, INPUT);
}


class PIController {
public:
    PIController(float kp, float ki, int max_output) {
        this->kp = kp;
        this->ki = ki;
        integral = 0;
        outputMin = -max_output;
        outputMax = max_output;
    }

    void reset(){
        integral = 0;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        integral += error;
        integral = constrain(integral, -255, 255);
        float output = (kp * error) + (ki * integral);

        return constrain(output, outputMin, outputMax);
    }

private:
    float kp, ki;
    float integral;
    float outputMin, outputMax;
};


class PDController {
public:
    PDController(float kp, float kd, int max_output) {
        this->kp = kp;
        this->kd = kd;
        prevError = 0;
        outputMin = -max_output;
        outputMax = max_output;
    }

    void reset(){
        prevError = 0;
    }

    void set_P_D(float p, float d){
        kp = p;
        kd = d;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        float derivative = (error - prevError) * FREQ;
        float output = (kp * error) + (kd * derivative);

        prevError = error;
        return constrain(output, outputMin, outputMax);
    }

private:
    float kp, kd;
    float prevError;
    float outputMin, outputMax;
};


#endif