#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <mcp_can.h>
#include <RMDx8Arduino.h>

#include "ForceSensor.h"
#include "ControlCylinder.h"
#include "WalkingStancePhase.h"

#define isDebug 0
#define isMotor 1

uint8_t sync_pin, sync_flag, is_error;
int16_t right_add_cur, left_add_cur;
int16_t right_tgt_cur, left_tgt_cur;
uint32_t timer[3];

int16_t base;
uint16_t T;
float freq, omega;

#if (isMotor)
    MCP_CAN CAN(9); //set CS PIN
    RMDx8Arduino right_motor(CAN, 0x141);
    RMDx8Arduino left_motor(CAN, 0x142);
#endif

ForceSensor right_sensor;
ForceSensor left_sensor;

ControlCylinder right_knee;
ControlCylinder left_knee;

WalkingStancePhase right(right_sensor);
WalkingStancePhase left(left_sensor);

void setup() {
    /* Arduino Settings */
    Serial.begin(115200);
    while (!Serial);
    Serial.println("--- start ---");
    

    /* Sensor Io Settings */
    sync_pin              = 2;
    right.sensor.heel_pin = A0;
    right.sensor.toe_pin  = A1;
    left.sensor.heel_pin  = A2;
    left.sensor.toe_pin   = A3;
    pinMode(sync_pin, INPUT);


    /* Cylinder Io Settings */
    right_knee.pull_pin   = 3;
    right_knee.push_pin   = 4;
    left_knee.pull_pin    = 5;
    left_knee.push_pin    = 6;
    right_knee.init();
    left_knee.init();


    /* Switching Phase Settings */
    right.thresh          = 300;
    left.thresh           = 300;


    /* Walking Phase Settings [ms] */
    T                     = 1500;

    right.ic_time         = T*0.02;  //   0-2 %
    right.lr_time         = T*0.10;  //  2-12 %
    right.mst_time        = T*0.19;  // 12-31 %
    right.tst_time        = T*0.19;  // 31-50 %
    right.psw_time        = T*0.12;  // 50-62 %

    left.ic_time          = T*0.02;  //   0-2 %
    left.lr_time          = T*0.10;  //  2-12 %
    left.mst_time         = T*0.19;  // 12-31 %
    left.tst_time         = T*0.19;  // 31-50 %
    left.psw_time         = T*0.12;  // 50-62 %


    /* Motor Settings */
    #if (isMotor)
        right_motor.canSetup();
        right_motor.clearState();
        right_motor.writePID(40, 40, 50, 40, 20, 250);

        left_motor.canSetup();
        left_motor.clearState();
        left_motor.writePID(40, 40, 50, 40, 20, 250);

        freq  = 1 / T * 1000;
        base  = 4 * 2000 / 12.5 / 3.3;  // [Nm] -> [-]
        omega = 2 * 3.14 * freq * 0.001;
    #endif
}

void loop() {
    #if (isMotor)
        Serial.println("time, sync, heelR, toeR, phaseR, tgt_curR, curR, posR, heelL, toeL, phaseL, tgt_curL, curL, posL");
    #elif (isDebug)
        Serial.println("time, sync, heelR, toeR, phaseR, tempR, add_curR, tgt_curR, curR, posR, heelL, toeL, phaseL, tempL, add_curL, tgt_curL, curL, posL");
    #else
        Serial.println("time, sync, heelR, toeR, phaseR, heelL, toeL, phaseL");
    #endif

    timer[0] = millis();

    while (is_error == 0) {
        /* Sensing Data */
        timer[1] = millis() - timer[0];
        sync_flag = PIND & _BV(2);
        right.sensor.readData();
        left.sensor.readData();
        #if (isMotor)
            right_motor.readPosition();
            left_motor.readPosition();
        #endif

        /* Sequence Control */
        right.phase = right.toggleStancePhase();
        left.phase = left.toggleStancePhase();

        switch (right.phase) {
            case right.NoSt:
                right_knee.setNeutral();
                right_add_cur = 0;
                break;

            case right.IC:
                right_knee.setPull();
                break;
            
            case right.LR:
                right_knee.setPull();
                right_add_cur = 100;
                break;

            case right.MSt:
                right_knee.setPush();
                right_add_cur = 100;
                break;

            case right.TSt:
                right_knee.setPush();
                right_add_cur = 0;
                break;
    
            case right.PSw:
                right_knee.setNeutral();
                break;

            default:
                right_knee.setNeutral();
                break;
        }

        switch (left.phase) {
            case left.NoSt:
                left_knee.setNeutral();
                left_add_cur = 0;
                break;

            case left.IC:
                left_knee.setPull();
                break;
            
            case left.LR:
                left_knee.setPull();
                left_add_cur = - 100;
                break;

            case left.MSt:
                left_knee.setPush();
                left_add_cur = - 100;
                break;

            case left.TSt:
                left_knee.setPush();
                left_add_cur = 0;
                break;
    
            case left.PSw:
                left_knee.setNeutral();
                break;

            default:
                left_knee.setNeutral();
                break;
        }

        #if (isMotor)
            /* CPG */
            int16_t base_cur = base * cos(omega * timer[1]);
            right_tgt_cur = base_cur + right_add_cur;
            left_tgt_cur = base_cur + left_add_cur;

            if (base_cur > 0) {
                left_tgt_cur = left_tgt_cur * 0.02;
            }
            else if (base_cur < 0) {
                right_tgt_cur = right_tgt_cur * 0.02;
            }


            /* Limit Angle */
            if (right_motor.present_position > 42000) {
                right_tgt_cur = 0;
            }
            else if (right_motor.present_position < -25200) {
                right_tgt_cur = 0;
            }

            if (left_motor.present_position < -42000) {
                left_tgt_cur = 0;
            }
            else if (left_motor.present_position > 25200) {
                left_tgt_cur = 0;
            }


            /* Error Handling */
            if (right_motor.present_position > 48000 || right_motor.present_position < -31200) {
                is_error = 1;
                right_tgt_cur = 0;
            }

            if (left_motor.present_position < -48000 || left_motor.present_position > 31200) {
                is_error = 1;
                left_tgt_cur = 0;
            }


            /* Output Current */
            right_motor.writeCurrent(right_tgt_cur);
            left_motor.writeCurrent(left_tgt_cur);
        #endif


        /* Serial Communication */
        Serial.print(timer[1]);
        Serial.print(",");
        Serial.print(sync_flag);
        Serial.print(",");
        Serial.print(right.sensor.heel);
        Serial.print(",");
        Serial.print(right.sensor.toe);
        Serial.print(",");
        Serial.print(right.phase);
        
        #if (isDebug)
            Serial.print(",");
            Serial.print(right_motor.temperature);
            Serial.print(",");
            Serial.print(right_add_cur);
        #endif
        
        #if (isMotor)
            Serial.print(",");
            Serial.print(right_tgt_cur);
            Serial.print(",");
            Serial.print(right_motor.present_current);
            Serial.print(",");
            Serial.print(right_motor.present_position);
        #endif

        Serial.print(",");
        Serial.print(left.sensor.heel);
        Serial.print(",");
        Serial.print(left.sensor.toe);
        Serial.print(",");
        Serial.print(left.phase);

        #if (isDebug)
            Serial.print(",");
            Serial.print(left_motor.temperature);
            Serial.print(",");
            Serial.print(left_add_cur);
        #endif

        #if (isMotor)
            Serial.print(",");
            Serial.print(left_tgt_cur);
            Serial.print(",");
            Serial.print(left_motor.present_current);
            Serial.print(",");
            Serial.print(left_motor.present_position);
        #endif

        Serial.println();
    }

    right_motor.clearState();
    left_motor.clearState();
    Serial.println("--- Catching Error ---");

    while (1) {
      /* 無限ループ */
    }
}