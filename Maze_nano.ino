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


void turn_forward(int value=25, int n_step=1){
    forward_pid.reset();
    long next_checkpoint = encoderCount + plush_per_cm*value;
    bool stt_left = 0;   // Left have wall
    bool stt_right = 0; // Right have wall
    int delta_dir   = 0;
    bool set_xxx = true;

    while(long(encoderCount+ENCODER_OFFSET) < next_checkpoint){
        int direction = get_direction();
        if(direction != 0xFFF){
            // Auto count if use SAFE_DISTANCE_MODE to avoid move to far from center
            if(SAFE_DISTANCE_MODE){
                if(set_xxx && encoderCount > long(next_checkpoint - cell_size*plush_per_cm)){
                    stt_left = !is_left_empty();   // Left have wall
                    stt_right = !is_right_empty(); // Right have wall
                    set_xxx = false;
                }
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

    if(global_dir_index == 0)   current_y -= n_step;
    if(global_dir_index == 1)   current_x -= n_step;
    if(global_dir_index == 2)   current_y += n_step;
    if(global_dir_index == 3 || global_dir_index == -1)   current_x += n_step;

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


void show_checkpoint(){
    if(is_left_mode())  lcd.print(F("LEFT | from ")); 
    else                lcd.print(F("RIGHT | from ")); 

    if(menu_mode == LEFT_RUN_0_MODE || menu_mode == RIGHT_RUN_0_MODE){
        lcd.print(F("0"));

        lcd.setCursor(11, 1);
        lcd.print(start_x);
        lcd.print(F(" "));
        lcd.print(start_y);
    }
    if(menu_mode == LEFT_RUN_1_MODE || menu_mode == RIGHT_RUN_1_MODE){
        Serial.print(F("LONGEST STEP :"));
        Serial.println(longest_step);
        Serial.print(F("locked_mode :"));
        Serial.println(locked_mode);

        lcd.print(-CHECKPOINT_1);

        int longest_pos = 0;
        if(locked_mode){
            longest_pos = my_mazer.load_position(max(longest_step-CHECKPOINT_1, 0));
        }else{
            int longest_traced_step = my_mazer.get_longest_step();
            Serial.print(F("longest_traced_step :"));
            Serial.println(longest_traced_step);
            if(longest_traced_step < CHECKPOINT_1){
                lcd.setCursor(11, 1);
                lcd.print(start_x);
                lcd.print(F(" "));
                lcd.print(start_y);
                return;
            }
            longest_pos = my_mazer.load_position(longest_traced_step-CHECKPOINT_1);
        }

        lcd.setCursor(11, 1);
        lcd.print(longest_pos%N_COLS);
        lcd.print(F(" "));
        lcd.print(longest_pos/N_COLS);
    }
    if(menu_mode == LEFT_RUN_2_MODE || menu_mode == RIGHT_RUN_2_MODE){
        lcd.print(-CHECKPOINT_2);

        int longest_pos = 0;
        if(locked_mode){
            longest_pos = my_mazer.load_position(max(longest_step-CHECKPOINT_2, 0));
        }else{
            int longest_traced_step = my_mazer.get_longest_step();
            if(longest_traced_step < CHECKPOINT_2){
                lcd.setCursor(11, 1);
                lcd.print(start_x);
                lcd.print(F(" "));
                lcd.print(start_y);
                return;
            }
            longest_pos = my_mazer.load_position(longest_traced_step-CHECKPOINT_2);
        }

        lcd.setCursor(11, 1);
        lcd.print(longest_pos%N_COLS);
        lcd.print(F(" "));
        lcd.print(longest_pos/N_COLS);
    }
}


void show_menu(){
    lcd.clear();
    menu_mode = menu_mode % N_MODES;

    switch (menu_mode) {
        case LEFT_TRACE_MODE:
            lcd.print(F("left | trace")); 
            break;
        case RIGHT_TRACE_MODE:
            lcd.print(F("right | trace")); 
            break;
        case TEST_LINE_MODE:
            lcd.print(F("test line")); 
            break;
        case RESET_IMU_MODE:
            lcd.print(F("Reset direction"));
            break;

        case SET_START_X:
            lcd.print(F("=> START X: ")); lcd.print(start_x);
            lcd.setCursor(0, 1);
            lcd.print(F("   START Y: ")); lcd.print(start_y);
            break;
        case SET_START_Y:
            lcd.print(F("   START X: ")); lcd.print(start_x);
            lcd.setCursor(0, 1);
            lcd.print(F("=> START Y: ")); lcd.print(start_y);
            break;
        case SET_GOAL_X:
            lcd.print(F("=> GOAL X: ")); lcd.print(goal_x);
            lcd.setCursor(0, 1);
            lcd.print(F("   GOAL Y: ")); lcd.print(goal_y);
            break;
        case SET_GOAL_Y:
            lcd.print(F("   GOAL X: ")); lcd.print(goal_x);
            lcd.setCursor(0, 1);
            lcd.print(F("=> GOAL Y: ")); lcd.print(goal_y);
            break;
        case SET_LOCK_MODE:
            if(locked_mode) lcd.print(F("READ MODE"));
            else            lcd.print(F("WRITE MODE"));
            break;

        default:
            show_checkpoint();
            break;
    }
    delay(debounce_time);
}


void set_start_end_point(){
    if(menu_mode == SET_START_X){
        start_x++;
        if(start_x > N_COLS)    start_x = 1;
        EEPROM.update(EEPROM_START_X, start_x);
    }  
    if(menu_mode == SET_START_Y){
        start_y++;
        if(start_y > N_ROWS)    start_y = 1;
        EEPROM.update(EEPROM_START_Y, start_y);
    }  

    if(menu_mode == SET_GOAL_X){
        goal_x++;
        if(goal_x > N_COLS)    goal_x = 1;
        EEPROM.update(EEPROM_GOAL_X,  goal_x);
    }  
    if(menu_mode == SET_GOAL_Y){
        goal_y++;
        if(goal_y > N_ROWS)    goal_y = 1;
        EEPROM.update(EEPROM_GOAL_Y,  goal_y);
    }
}


void test_menu(){
    if(menu_mode == RESET_IMU_MODE){
        Serial.println("AT+RST");
        delay(1000);
        Serial.println("AT+RST");
    }

    if(menu_mode == TEST_LINE_MODE){
        delay(2000);
        // test_forward();
        // test_rotate(1800);
        global_dir = 0;
        turn_forward(300);

        stop_move();
    }
}


void tracing_menu(){
    if(locked_mode){
        lcd.setCursor(0, 1);
        lcd.print(F("READ ONLY "));
        delay(1000);
        return;
    }

    global_dir = 0;
    global_dir_index = 0;
    current_x = start_x;
    current_y = start_y;
    step_count = 0;

    delay(1000);
    long time_out = millis() + 300000;

    while(true){
        if(is_left_mode())  left_wall_following();
        else                right_wall_following();
        if(DEBUG_MODE)  delay(2000);

        if(current_x==goal_x && current_y==goal_y){
            // my_mazer.add_step(global_dir_index, current_x, current_y, step_count);
            longest_step = 0;
            update_longest_step();

            locked_mode = true;
            update_locked_mode();
            break;
        }

        if(millis() > time_out){
            break;
        }
    }
    stop_move();
}


void run_traced(int dir, int n_step=1){
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

    turn_forward(cell_size*n_step, n_step);
    step_count += n_step;
    stop_move();
}


bool is_left_mode(){
    if(menu_mode == LEFT_RUN_0_MODE)    return true;
    if(menu_mode == LEFT_RUN_1_MODE)    return true;
    if(menu_mode == LEFT_RUN_2_MODE)    return true;
    if(menu_mode == LEFT_TRACE_MODE)    return true;

    return false;
}


void running_menu(){
    global_dir = 0;
    global_dir_index = 0;

    current_x = start_x;
    current_y = start_y;
    step_count = 0;

    if(menu_mode == LEFT_RUN_1_MODE || menu_mode == RIGHT_RUN_1_MODE){
        lcd.print(-CHECKPOINT_1);
        step_count = max(longest_step-CHECKPOINT_1, 0);
        int longest_pos = my_mazer.load_position(step_count);
        if(longest_step == my_mazer.get_longest_step()){
            my_mazer.save_step(0, 0, step_count+1);
        }
        current_x = longest_pos%N_COLS;
        current_y = longest_pos/N_COLS;

        lcd.setCursor(11, 1);
        lcd.print(current_x);
        lcd.print(F(" "));
        lcd.print(current_y);
    }

    if(menu_mode == LEFT_RUN_2_MODE || menu_mode == RIGHT_RUN_2_MODE){
        lcd.print(-CHECKPOINT_2);
        step_count = max(longest_step-CHECKPOINT_2, 0);
        int longest_pos = my_mazer.load_position(step_count);
        if(longest_step == my_mazer.get_longest_step()){
            my_mazer.save_step(0, 0, step_count+1);
        }
        current_x = longest_pos%N_COLS;
        current_y = longest_pos/N_COLS;

        lcd.setCursor(11, 1);
        lcd.print(current_x);
        lcd.print(F(" "));
        lcd.print(current_y);
    }


    bool read_from_eeprom = 1;
    delay(1000);
    long time_out = millis() + 300000;

    while(true){
        if((millis() > time_out) || (current_x==goal_x && current_y==goal_y)){
            // my_mazer.save_step(0, 0, step_count);
            break;
        }
        if(read_from_eeprom){
            int dir = my_mazer.load_direction(step_count);
            int pos = my_mazer.load_position(step_count);
            int n_step = 1;
            if(pos == 0 && dir == 0){
                read_from_eeprom = false;
                if(is_left_mode() ) left_wall_following();
                else                right_wall_following();
            }else{
                for(int i=1; i<20; i++){
                    if(dir == my_mazer.load_direction(step_count+i))    n_step = i+1;
                    else                                                break;
                }
                // run by direction.
                run_traced(dir, n_step);
            }
        }else{
            if(is_left_mode())  left_wall_following();
            else                right_wall_following();
        }

        longest_step = step_count;
        update_longest_step();
    }
    stop_move();
}


void handle_button(){
    if(digitalRead(btt_top)){
        menu_mode++;
        show_menu();
    }
    if(digitalRead(btt_bot)){
        menu_mode = (menu_mode == 0) ? RESET_IMU_MODE : menu_mode - 1;
        show_menu();
    }
    if(digitalRead(btt_ent)){
        if(SET_START_X <= menu_mode && menu_mode <= SET_GOAL_Y)
            set_start_end_point();

        if(menu_mode == RESET_IMU_MODE || menu_mode == TEST_LINE_MODE)
            test_menu();

        if(menu_mode == LEFT_TRACE_MODE || menu_mode == RIGHT_TRACE_MODE)
            tracing_menu();

        if(LEFT_RUN_0_MODE <= menu_mode && menu_mode < LEFT_TRACE_MODE)
            running_menu();
        
        if(menu_mode == SET_LOCK_MODE){
            locked_mode = !locked_mode;
            update_locked_mode();
        }
        
        show_menu();
    }
}


void loop() {
    handle_button();

    int direction = get_direction();
    if(direction != 0xFFF){
        if(menu_mode <= RESET_IMU_MODE){
            lcd.setCursor(0, 1);
            lcd.print(String(direction));
            lcd.print(F(" "));
        }
        if(menu_mode == TEST_LINE_MODE){
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
    // read_sensor();
}
