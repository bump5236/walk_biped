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
#define isMotor 0


/* Angle Settings */
#define NORMAL_MAX_ANGLE 75600
#define NORMAL_MIN_ANGLE 8400
#define LIMIT_MAX_ANGLE 120000
#define LIMIT_MIN_ANGLE 24000

/* Variable Definition */
uint8_t right_bend_sensor_pin;
uint16_t right_bend;

uint8_t sync_pin, sync_flag, is_error;
int16_t right_add_cur;
int16_t right_tgt_cur;
int32_t right_pos[2];

int16_t base;
uint16_t T;
uint32_t timer[3];

float freq, omega;


/* Init Class */
#if (isMotor)
    MCP_CAN CAN(9); //set CS PIN
    RMDx8Arduino right_motor(CAN, 0x142);
#endif
ForceSensor right_sensor;
ControlCylinder right_ankle;
WalkingStancePhase right(right_sensor);

void setup() {
    /* Arduino Settings */
    Serial.begin(115200);
    while (!Serial);
    Serial.println("--- start ---");
    

    /* Sensor Io Settings */
    sync_pin              = 2;
    right.sensor.heel_pin = A0;
    right.sensor.toe_pin  = A1;
    right_bend_sensor_pin = A2;
    pinMode(sync_pin, INPUT);


    /* Cylinder Io Settings */
    right_ankle.pull_pin   = 3;
    right_ankle.push_pin   = 4;
    right_ankle.init();


    /* Switching Phase Settings */
    right.thresh          = 300;


    /* Walking Phase Settings [ms] */
    T                     = 1500;

    right.ic_time         = T*0.02;  //   0-2 %
    right.lr_time         = T*0.10;  //  2-12 %
    right.mst_time        = T*0.19;  // 12-31 %
    right.tst_time        = T*0.19;  // 31-50 %
    right.psw_time        = T*0.12;  // 50-62 %


    /* Motor Settings */
    #if (isMotor)
        right_motor.canSetup();
        right_motor.clearState();
        right_motor.writePID(40, 40, 50, 40, 20, 250);

        freq  = 1000.0 / T;
        base  = 3.5 * 2000 / 12.5 / 3.3;  // [Nm] -> [-]
        omega = 2.0 * 3.14 * freq * 0.001;
    #endif
}

void loop() {
    #if (isMotor)
    #else
        Serial.println("time, sync, heelR, toeR, vendR, phaseR");
    #endif

    #if (isMotor)
        right_motor.readPosition();
        right_pos[0] = right_motor.present_position;
    #endif

    timer[0] = millis();

    while (is_error == 0) {
        /* Sensing Data */
        timer[1] = millis() - timer[0];
        sync_flag = PIND & _BV(2);
        right.sensor.readData();
        right_bend = analogRead(right_bend_sensor_pin);

        #if (isMotor)
            right_motor.readPosition();
            right_pos[1] = right_motor.present_position - right_pos[0];
        #endif

        /* Sequence Control */
        right.phase = right.toggleStancePhase();

        switch (right.phase) {
            case right.NoSt:
                right_ankle.setNeutral();
                right_add_cur = 0;
                break;

            case right.IC:
                right_ankle.setNeutral();
                break;
            
            case right.LR:
                right_ankle.setNeutral();
                right_add_cur = 50;
                break;

            case right.MSt:
                right_ankle.setNeutral();
                right_add_cur = 0;
                break;

            case right.TSt:
                right_ankle.setPull();
                right_add_cur = 0;
                break;
    
            case right.PSw:
                right_ankle.setPush();
                break;

            default:
                right_ankle.setNeutral();
                break;
        }

        #if (isMotor)
            /* CPG */
            int16_t base_cur = base * cos(omega * timer[1]);
            right_tgt_cur = base_cur + right_add_cur;

            if (base_cur < 0) {
                right_tgt_cur = right_tgt_cur * 0.05;
            }


            /* Limit Angle */
            if (right_pos[1] > NORMAL_MAX_ANGLE && right_tgt_cur > 0) {
                right_tgt_cur = right_tgt_cur * 0.7;
            }
            else if (right_pos[1] < - NORMAL_MIN_ANGLE && right_tgt_cur < 0) {
                right_tgt_cur = 80;
            }


            /* Error Handling */
            if (right_pos[1] > LIMIT_MAX_ANGLE || right_pos[1] < - LIMIT_MIN_ANGLE) {
                is_error = 1;
                right_tgt_cur = 0;
            }


            /* Output Current */
            right_motor.writeCurrent(right_tgt_cur);
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
            Serial.print(right_bend);
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
        #endif

        Serial.println();
    }

    #if (isMotor)
        right_motor.clearState();
    #endif
    
    Serial.println("--- Catching Error ---");

    while (1) {
      /* 無限ループ */
    }
}