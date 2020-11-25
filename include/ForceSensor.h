#ifndef ForceSensor_h
#define ForceSensor_h

#include <Arduino.h>

class ForceSensor {
public:
    int8_t heel_pin, toe_pin, thigh_pin;
    uint16_t heel, toe, thigh;

    void readData();
private:
    /* data */
};


#endif