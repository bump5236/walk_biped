#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <mcp_can.h>
#include <RMDx8Arduino.h>

#include "ForceSensor.h"
#include "ControlCylinder.h"
#include "WalkingStancePhase.h"

/* Config */
#define isDebug 0
#define isMotor 1


/* Angle Settings */
#define NORMAL_MAX_ANGLE 75600
#define NORMAL_MIN_ANGLE 8400
#define LIMIT_MAX_ANGLE 120000
#define LIMIT_MIN_ANGLE 24000

uint8_t sync_pin, sync_flag, is_error;
int16_t right_add_cur, left_add_cur;
int16_t right_tgt_cur, left_tgt_cur;
int32_t right_pos[2], left_pos[2];

int16_t base;
uint16_t T;
uint32_t timer[3];

float freq, omega;

#if (isMotor)
    MCP_CAN CAN(9); //set CS PIN
    RMDx8Arduino right_motor(CAN, 0x142);
    RMDx8Arduino left_motor(CAN, 0x141);
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

        freq  = 1000.0 / T;
        base  = 3.5 * 2000 / 12.5 / 3.3;  // [Nm] -> [-]
        omega = 2.0 * 3.14 * freq * 0.001;
    #endif
}

void loop() {
    #if (isMotor)
    #else
        Serial.println("time, sync, heelR, toeR, phaseR, heelL, toeL, phaseL");
    #endif

    timer[0] = millis();
    #if (isMotor)
        right_motor.readPosition();
        left_motor.readPosition();
        right_pos[0] = right_motor.present_position;
        left_pos[0] = left_motor.present_position;
    #endif

    while (is_error == 0) {
        /* Sensing Data */
        timer[1] = millis() - timer[0];
        sync_flag = PIND & _BV(2);
        right.sensor.readData();
        left.sensor.readData();
        #if (isMotor)
            right_motor.readPosition();
            left_motor.readPosition();
            right_pos[1] = right_motor.present_position - right_pos[0];
            left_pos[1] = left_motor.present_position - left_pos[0];
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
                right_add_cur = 50;
                break;

            case right.MSt:
                right_knee.setPull();
                right_add_cur = 0;
                break;

            case right.TSt:
                // right_knee.setPush();
                right_knee.setPull();
                right_add_cur = 0;
                break;
    
            case right.PSw:
                // right_knee.setNeutral();
                right_knee.setPush();
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
                left_add_cur = - 50;
                break;

            case left.MSt:
                left_knee.setPull();
                left_add_cur = 0;
                break;

            case left.TSt:
                // left_knee.setPush();
                left_knee.setPull();
                left_add_cur = 0;
                break;
    
            case left.PSw:
                // left_knee.setNeutral();
                left_knee.setPush();
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
                left_tgt_cur = left_tgt_cur * 0.05;
            }
            else if (base_cur < 0) {
                right_tgt_cur = right_tgt_cur * 0.05;
            }


            /* Limit Angle */
            if (right_pos[1] > NORMAL_MAX_ANGLE && right_tgt_cur > 0) {
                right_tgt_cur = right_tgt_cur * 0.7;
            }
            else if (right_pos[1] < - NORMAL_MIN_ANGLE && right_tgt_cur < 0) {
                right_tgt_cur = 80;
            }

            if (left_pos[1] < - NORMAL_MAX_ANGLE && left_tgt_cur < 0) {
                left_tgt_cur = left_tgt_cur * 0.7;
            }
            else if (left_pos[1] > NORMAL_MIN_ANGLE && left_tgt_cur > 0) {
                left_tgt_cur = -80;
            }


            /* Error Handling */
            if (right_pos[1] > LIMIT_MAX_ANGLE || right_pos[1] < - LIMIT_MIN_ANGLE) {
                is_error = 1;
                right_tgt_cur = 0;
            }

            if (left_pos[1] < - LIMIT_MAX_ANGLE || left_pos[1] > LIMIT_MIN_ANGLE) {
                is_error = 1;
                left_tgt_cur = 0;
            }


            /* Output Current */
            right_motor.writeCurrent(right_tgt_cur);
            left_motor.writeCurrent(left_tgt_cur);
        #endif


        /* Serial Communication */
        #if (isDebug)
            Serial.print("TIME:");
            Serial.print(timer[1]);
            Serial.print("\tR_TGT:");
            Serial.print(right_tgt_cur);
            Serial.print("\tR_POS:");
            Serial.print(right_pos[1]);
            Serial.print("\tL_TGT:");
            Serial.print(left_tgt_cur);
            Serial.print("\tL_POS:");
            Serial.print(left_pos[1]);

        #else
            Serial.print(timer[1]);
            Serial.print(",");
            Serial.print(sync_flag);
            Serial.print(",");
            Serial.print(right.sensor.heel);
            Serial.print(",");
            Serial.print(right.sensor.toe);
            Serial.print(",");
            Serial.print(right.phase);
            
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

            #if (isMotor)
                Serial.print(",");
                Serial.print(left_tgt_cur);
                Serial.print(",");
                Serial.print(left_motor.present_current);
                Serial.print(",");
                Serial.print(left_motor.present_position);
            #endif
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