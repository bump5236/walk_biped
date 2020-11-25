#include "ControlCylinder.h"

void ControlCylinder::init() {
    pinMode(push_pin, OUTPUT);
    pinMode(pull_pin, OUTPUT);
}

void ControlCylinder::setPush() {
    hi_speed_digitalWrite(push_pin, HIGH);
    hi_speed_digitalWrite(pull_pin, LOW);
}

void ControlCylinder::setPull() {
    hi_speed_digitalWrite(push_pin, LOW);
    hi_speed_digitalWrite(pull_pin, HIGH);
}

void ControlCylinder::setNeutral() {
    hi_speed_digitalWrite(push_pin, LOW);
    hi_speed_digitalWrite(pull_pin, LOW);
}

// Private
void ControlCylinder::hi_speed_digitalWrite(int pin, char _io) {
    stl_write_reg*  reg = &(stl_io_regs[pin]);
    if (_io == HIGH) {
        *(reg->Io_register) |= reg->mask;
    }
    else {
        *(reg->Io_register) &= ~(reg->mask);
    }   
}