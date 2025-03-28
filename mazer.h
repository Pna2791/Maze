#ifndef MAZER_H
#define MAZER_H

#include <EEPROM.h>
// #include "config.h"

#define eeprom_offset 10


class Mazer {
    public:
        Mazer(int n_rows = 14, int n_cols = 22) {
            this->n_rows = n_rows;
            this->n_cols = n_cols;
        }
        
        int load_direction(int step){
            int eepromIndex = eeprom_offset + step * 2;
            return EEPROM.read(eepromIndex+1) % 4;
        }

        int load_position(int step){
            int eepromIndex = eeprom_offset + step * 2;
            byte highByte = EEPROM.read(eepromIndex);
            byte lowByte = EEPROM.read(eepromIndex + 1);

            int value = (highByte << 8) + lowByte;
            return value / 16;
        }

        void save_step(int dir, int pos, int step){
            dir %= 4;
            int eepromIndex = eeprom_offset + step * 2;

            int value = pos * 16 + dir;
            EEPROM.update(eepromIndex, byte(value/256));
            EEPROM.update(eepromIndex+1, byte(value%256));

            EEPROM.update(eepromIndex+2, 0);
            EEPROM.update(eepromIndex+3, 0);
        }

        int add_step(int dir, int xx, int yy, int step){
            int pos = yy*n_cols + xx;
            dir %= 4;
            for(int i=0; i<step; i++){
                if(pos == load_position(i)){
                    save_step(dir, pos, i);
                    return i;
                }
            }
            save_step(dir, pos, step);
            return step;
        }
    
    private:
        int n_rows;
        int n_cols;

};



#endif
