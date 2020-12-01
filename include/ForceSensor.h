#ifndef ForceSensor_h
#define ForceSensor_h

#include <Arduino.h>

class ForceSensor {
public:
    uint8_t heel_pin, toe_pin, thigh_pin;
    uint16_t heel, toe, thigh;

    void readData();
private:
    /* data */
};


#endif