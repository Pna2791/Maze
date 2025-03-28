#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#include "config.h"
#include "sensor.h"
#include "hi229.h"
#include "wheel.h"
#include "pid_control.h"
#include "mazer.h"


#define SAFE_DISTANCE_MODE  1
#define LANE_KEEPING_MODE   0
#define DEBUG_MODE          0

// Set the LCD address (commonly 0x27 or 0x3F), columns and rows:
LiquidCrystal_I2C lcd(0x27, 16, 2);  


byte menu_mode = 0;
int global_dir = 0;
int global_dir_index = 0;
PDController forward_pid    = PDController(1, 0.1, FW_SPEED/2);
PDController turn_pid       = PDController(0.65, 0.035, TURN_SPEED);
Mazer my_mazer = Mazer(N_ROWS, N_COLS);

int current_x = 0;
int current_y = 0;
int step_count = 0;

void setup() {
    Serial.begin(115200);
    setup_button();
    setup_encoder();
    setup_sensor();
    setup_wheel();
    setup_pid_input();

    load_start_goal();

    lcd.init();           // Initialize the LCD
    lcd.backlight();      // Turn on the backlight
    show_menu();
}


void test_forward(){
    forward_pid.reset();
    long time_out = millis() + 5000;
    while(millis() < time_out){
        int direction = get_direction();
        if(direction != 0xFFF){
            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int forward_value = forward_pid.compute(global_dir, direction);
            move_forward(forward_value, 0);
        }
    }
}


void test_rotate(int delta){
    turn_pid.reset();

    global_dir = (global_dir+3600)%3600;
    global_dir += delta;
    if(global_dir > 2250)   global_dir -= 3600;

    long time_out = millis() + 5000;
    while(millis() < time_out){
        int direction = get_direction();
        if(direction != 0xFFF){
            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int turn_value = turn_pid.compute(global_dir, direction);
            rotate_CCW(turn_value);
        }
    }
}


void turn_forward(int value=25){
    forward_pid.reset();
    long next_checkpoint = encoderCount + plush_per_cm*value;
    bool stt_left = !is_left_empty();   // Left have wall
    bool stt_right = !is_right_empty(); // Right have wall
    int delta_dir   = 0;

    while(long(encoderCount+ENCODER_OFFSET) < next_checkpoint){
        int direction = get_direction();
        if(direction != 0xFFF){
            // Auto count if use SAFE_DISTANCE_MODE to avoid move to far from center
            if(SAFE_DISTANCE_MODE){
                if(stt_left){
                    if(is_left_empty()){
                        stt_left = false;
                        next_checkpoint = max(next_checkpoint, encoderCount+safe_distance*plush_per_cm);
                    }
                }
                if(stt_right){
                    if(is_right_empty()){
                        stt_right = false;
                        next_checkpoint = max(next_checkpoint, encoderCount+safe_distance*plush_per_cm);
                    }
                }
            }

            if(LANE_KEEPING_MODE){
                if(is_left_rear())  delta_dir = -DELTA_DIR;
                if(is_right_rear()) delta_dir = DELTA_DIR;
            }


            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int forward_value = forward_pid.compute((global_dir+delta_dir), direction);
            move_forward(forward_value, 0);
            // if(next_checkpoint-encoderCount > 30)   move_forward(forward_value, 0);
            // else                                    move_forward(forward_value, 64);
        }
    }

    if(global_dir_index == 0)   current_y--;
    if(global_dir_index == 1)   current_x--;
    if(global_dir_index == 2)   current_y++;
    if(global_dir_index == 3 || global_dir_index == -1)   current_x++;

    // Adjust direction
    global_dir = global_dir+delta_dir;
    move_speed(-255);
    delay(25);
}


void turn_CCW(int delta){
    global_dir = (global_dir+3600)%3600;
    global_dir += delta;
    if(global_dir > 2250)   global_dir -= 3600;

    turn_pid.reset();
    float mean_error = 900;
    long time_out = millis() + 600;
    while(true){
        int direction = get_direction();
        if(direction != 0xFFF){
            mean_error = 0.6*mean_error + 0.4*(abs(global_dir-direction)%3600);
            if(mean_error < 100)        break;
            if(millis() > time_out)     break;

            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int turn_value = turn_pid.compute(global_dir, direction);
            rotate_CCW(turn_value);
        }
    }
}


void show_step_dir(){
    lcd.setCursor(0, 0);
    lcd.print(step_count);
    lcd.print(F(" "));
    lcd.print(global_dir_index);
}


void left_wall_following(){
    if(is_left_empty()){
        global_dir_index = int(global_dir_index + 4 + 1)%4;
        turn_CCW(900);
    }else if(is_front_empty()){
        // Forward
    }else if(is_right_empty()){
        global_dir_index = int(global_dir_index + 4 - 1)%4;
        turn_CCW(-900);
    }else{
        global_dir_index = int(global_dir_index + 4 + 2)%4;
        turn_CCW(1800);
    }
    stop_move();
    step_count = my_mazer.add_step(global_dir_index, current_x, current_y, step_count);
    show_step_dir();
    step_count++;

    turn_forward(cell_size);
    stop_move();
}


void right_wall_following(){
    if(is_right_empty()){
        global_dir_index = int(global_dir_index + 4 - 1)%4;
        turn_CCW(-900);
    }else if(is_front_empty()){
        // Just forward
    }else if(is_left_empty()){
        global_dir_index = int(global_dir_index + 4 + 1)%4;
        turn_CCW(900);
    }else{
        global_dir_index = int(global_dir_index + 4 + 2)%4;
        turn_CCW(1800);
    }
    stop_move();
    step_count = my_mazer.add_step(global_dir_index, current_x, current_y, step_count);
    show_step_dir();
    step_count++;

    turn_forward(cell_size);
    stop_move();
}


void show_menu(){
    lcd.clear();
    menu_mode = menu_mode % n_modes;

    switch (menu_mode) {
        case 0:
            lcd.print(F("left | trace")); 
            break;
        case 1:
            lcd.print(F("left | run")); 
            break;
        case 2:
            lcd.print(F("right | trace")); 
            break;
        case 3:
            lcd.print(F("right | run")); 
            break;
        case 4:
            lcd.print(F("test line")); 
            break;
        case 5:
            lcd.print(F("Reset direction"));
            break;
        case 6:
            lcd.print(F("=> start x: ")); lcd.print(start_x);
            lcd.setCursor(0, 1);
            lcd.print(F("   start y: ")); lcd.print(start_y);
            break;
        case 7:
            lcd.print(F("   start x: ")); lcd.print(start_x);
            lcd.setCursor(0, 1);
            lcd.print(F("=> start y: ")); lcd.print(start_y);
            break;
        case 8:
            lcd.print(F("=> goal x: ")); lcd.print(goal_x);
            lcd.setCursor(0, 1);
            lcd.print(F("   goal y: ")); lcd.print(goal_y);
            break;
        case 9:
            lcd.print(F("   goal x: ")); lcd.print(goal_x);
            lcd.setCursor(0, 1);
            lcd.print(F("=> goal y: ")); lcd.print(goal_y);
            break;
    }
    delay(debounce_time);
}


void set_start_end_point(){
    if(menu_mode == 6){
        start_x++;
        if(start_x > N_COLS)    start_x = 1;
        EEPROM.update(EEPROM_START_X, start_x);
    }  
    if(menu_mode == 7){
        start_y++;
        if(start_y > N_ROWS)    start_y = 1;
        EEPROM.update(EEPROM_START_Y, start_y);
    }  
    if(menu_mode == 8){
        goal_x++;
        if(goal_x > N_COLS)    goal_x = 1;
        EEPROM.update(EEPROM_GOAL_X,  goal_x);
    }  
    if(menu_mode == 9){
        goal_y++;
        if(goal_y > N_ROWS)    goal_y = 1;
        EEPROM.update(EEPROM_GOAL_Y,  goal_y);
    }
}


void test_menu(){
    if(menu_mode == 5){
        Serial.println("AT+RST");
    }
    if(menu_mode == 4){
        delay(2000);
        // test_forward();
        // test_rotate(1800);
        global_dir = 0;
        turn_forward(300);

        stop_move();
    }
}


void tracing_menu(){
    global_dir = 0;
    global_dir_index = 0;
    current_x = start_x;
    current_y = start_y;
    step_count = 0;

    if(menu_mode == 0){
        delay(1000);
        long time_out = millis() + 300000;

        while(true){
            left_wall_following();
            if((millis() > time_out) || (current_x==goal_x && current_y==goal_y)){
                my_mazer.save_step(0, 0, step_count);
                break;
            }
        }
        stop_move();
    }
    if(menu_mode == 2){
        delay(1000);
        long time_out = millis() + 300000;

        while(true){
            right_wall_following();
            if(DEBUG_MODE)  delay(2000);
            if((millis() > time_out) || (current_x==goal_x && current_y==goal_y)){
                my_mazer.save_step(0, 0, step_count);
                break;
            }
        }
        stop_move();
    }
}


void run_traced(int dir){
    int deltal = int(4 + dir-global_dir_index)%4;
    global_dir_index = dir;
    show_step_dir();
    if(DEBUG_MODE)  delay(2000);

    if(deltal == 1){
        turn_CCW(900);
    }else if(deltal == 0){
        // Forward
    }else if(deltal == 3 || deltal == -1){
        turn_CCW(-900);
    }else{
        turn_CCW(1800);
    }
    stop_move();
    step_count++;

    turn_forward(cell_size);
    stop_move();
}


void running_menu(){
    global_dir = 0;
    global_dir_index = 0;
    current_x = start_x;
    current_y = start_y;
    step_count = 0;

    bool read_from_eeprom = 1;
    if(menu_mode == 1){
        delay(1000);
        long time_out = millis() + 300000;

        while(true){
            if(read_from_eeprom){
                int dir = my_mazer.load_direction(step_count);
                int pos = my_mazer.load_position(step_count);
                if(dir == 0 && pos == 0 ){
                    read_from_eeprom = false;
                    left_wall_following();
                }else{
                    // run by direction.
                    run_traced(dir);
                }
            }else
                left_wall_following();
            if((millis() > time_out) || (current_x==goal_x && current_y==goal_y)){
                my_mazer.save_step(0, 0, step_count);
                break;
            }
        }
        stop_move();
    }
    if(menu_mode == 3){
        delay(1000);
        long time_out = millis() + 300000;

        while(true){
            if(read_from_eeprom){
                int dir = my_mazer.load_direction(step_count);
                int pos = my_mazer.load_position(step_count);

                if(dir == 0 && pos == 0 ){
                    read_from_eeprom = false;
                    right_wall_following();
                }else{
                    // run by direction.
                    run_traced(dir);
                }
            }else
                right_wall_following();
            if((millis() > time_out) || (current_x==goal_x && current_y==goal_y)){
                my_mazer.save_step(0, 0, step_count);
                break;
            }
            if(DEBUG_MODE)  delay(5000);
        }
        stop_move();
    }

}


void handle_button(){
    if(digitalRead(btt_top)){
        menu_mode++;
        show_menu();
    }
    if(digitalRead(btt_bot)){
        menu_mode--;
        show_menu();
    }
    if(digitalRead(btt_ent)){
        set_start_end_point();
        test_menu();
        tracing_menu();
        running_menu();
        
        show_menu();
    }
}


void loop() {
    handle_button();

    int direction = get_direction();
    if(direction != 0xFFF){
        if(menu_mode <= 5){
            lcd.setCursor(0, 1);
            lcd.print(String(direction));
            lcd.print(F(" "));
        }
        if(menu_mode == 4){
            // lcd.setCursor(0, 1);
            lcd.print(String(encoderCount));
            lcd.print(F(" "));
        }

        // if(menu_mode == 4){
        //     float kp = analogRead(P_pin) / 102.4/5;
        //     float kd = analogRead(D_pin) / 1024./10;
        //     forward_pid.set_P_D(kp, kd);

        //     lcd.print(kp);
        //     lcd.print(F(" "));
        //     lcd.print(round(kd*1000));
        //     lcd.print(F(" "));
        // }
    }
    read_sensor();
}
