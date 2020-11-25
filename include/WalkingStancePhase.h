#ifndef WalkingStancePhase_h
#define WalkingStancePhase_h

#include <Arduino.h>

#include "ForceSensor.h"

class WalkingStancePhase {
public:
    ForceSensor sensor;

    uint8_t phase;
    int16_t thresh;
    uint32_t start_stance_time, last_stance_time;
    uint32_t nost_time;
    uint32_t ic_time;
    uint32_t lr_time;
    uint32_t mst_time;
    uint32_t tst_time;
    uint32_t psw_time;

    enum StancePhase {
        NoSt,
        IC,
        LR,
        MSt,
        TSt,
        PSw
    };

    WalkingStancePhase(ForceSensor &ForceSensor);

    int toggleStancePhase();

private:
    uint32_t nost_start_time;
    uint32_t ic_start_time;
    uint32_t lr_start_time;
    uint32_t mst_start_time;
    uint32_t tst_start_time;
    uint32_t psw_start_time;
};

#endif