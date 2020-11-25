#include "ForceSensor.h"

void ForceSensor::readData() {
    heel = analogRead(heel_pin);
    toe = analogRead(toe_pin);
    thigh = analogRead(thigh_pin);
}