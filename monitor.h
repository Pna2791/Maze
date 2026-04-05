#ifndef MONITOR_H
#define MONITOR_H


#ifdef nano
    const float max_analog = 1023;
#else
    const float max_analog = 4097;
#endif


class Monitor{
    private:
        int button_pin, n_keys;


    public:
        Monitor(int button_pin, int n_keys){
            this->button_pin    = button_pin;
            this->n_keys        = n_keys;
            
            pinMode(button_pin, INPUT);
        }

        int read_button(){
            int value = analogRead(button_pin)*n_keys;
            return round(value/max_analog);
        }
}










#endif