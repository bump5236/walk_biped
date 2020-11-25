#ifndef ControlCylinder_h
#define ControlCylinder_h

#include <Arduino.h>

class ControlCylinder {
public:
    int8_t push_pin, pull_pin;

    typedef struct {
        volatile uint8_t* Io_register;
        unsigned char     mask;
    } stl_write_reg;

    stl_write_reg stl_io_regs[14] = {
        {&PORTD,0b00000001}, /* pin0   PD0 */
        {&PORTD,0b00000010}, /* pin1   PD1 */
        {&PORTD,0b00000100}, /* pin2   PD2 */
        {&PORTD,0b00001000}, /* pin3   PD3 */
        {&PORTD,0b00010000}, /* pin4   PD4 */
        {&PORTD,0b00100000}, /* pin5   PD5 */
        {&PORTD,0b01000000}, /* pin6   PD6 */
        {&PORTD,0b10000000}, /* pin7   PD7 */
        {&PORTB,0b00000001}, /* pin8   PB0 */
        {&PORTB,0b00000010}, /* pin9   PB1 */
        {&PORTB,0b00000100}, /* pin10  PB2 */
        {&PORTB,0b00001000}, /* pin11  PB3 */
        {&PORTB,0b00010000}, /* pin12  PB4 */
        {&PORTB,0b00100000}  /* pin13  PB5 */
    };

    void init();
    void setPush();
    void setPull();
    void setNeutral();

private:
    void hi_speed_digitalWrite(int pin, char _io);
};


#endif