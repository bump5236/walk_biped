#include "WalkingStancePhase.h"

WalkingStancePhase::WalkingStancePhase(ForceSensor &ForceSensor)
    :sensor(ForceSensor){
    phase = NoSt;
}

int WalkingStancePhase::toggleStancePhase() {
    if (phase == NoSt) {
        start_stance_time = millis();
    }

    if ((sensor.heel > thresh) && (phase == NoSt)) {
        ic_start_time = millis();
        return IC;
    }

    if ((millis() - ic_start_time > ic_time) && (phase == IC)) {
        lr_start_time = millis();
        return LR;
    }

    if ((millis() - lr_start_time > lr_time) && (phase == LR) && (sensor.heel <= thresh)) {
        mst_start_time = millis();
        return MSt;
    }

    if ((millis() - mst_start_time > mst_time) && (phase == MSt)) {
        tst_start_time = millis();
        return TSt;
    }

    if ((millis() - tst_start_time > tst_time) && (phase == TSt)) {
        psw_start_time = millis();
        return PSw;
    }

    if ((millis() - psw_start_time > psw_time) && (phase == PSw)) {
        last_stance_time = millis() - start_stance_time;
        return NoSt;
    }

    return phase;
}