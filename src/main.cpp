#include <Arduino.h>
#include <FastGPIO.h>

#include <stepper.h>

//#define SIMAVR

#define BUTTON_START IO_B0
#define BUTTON_STOP  IO_B1
#define SWITCH_MODE  IO_B2
#define SWITCH_LEN   IO_B3

#define ANALOG_SPEED A7

#define MOTOR1_STEP IO_D2
#define MOTOR2_STEP IO_D3
#define MOTOR3_STEP IO_D4

#define MOTOR_ENABLE IO_D5

struct Machine {
    static constexpr float MAX_SPEED_S1 = 7200.0f/1024.0f;
    static constexpr float MAX_SPEED_S2 = 7200.0f/1024.0f;
    static constexpr float MAX_SPEED_S3 = 7200.0f/1024.0f;

    static constexpr unsigned long S1_LENGTH1 = 261UL;
    static constexpr unsigned long S1_LENGTH2 = 440UL;

    static constexpr unsigned long S2_LENGTH = 5*800UL;
    static constexpr unsigned long S3_LENGTH = 5*1600UL;

    Stepper<MOTOR1_STEP> s;
    Stepper<MOTOR2_STEP> s2;
    Stepper<MOTOR3_STEP> s3;
    enum State {
        M_IDLE,
        M_SETUP, // advance wire while button #1 is pressed
        M_STOPPING,
        M_SETUP2, // advance motor #2 while button #2 is pressed
        M_STOPPING2,
        M_SETUP3, // advance motor #3 while button #2 is pressed
        M_STOPPING3,
        M_STEP1, // advance wire 20mm (1/4 turn)
        M_STEP2, // half turn #2 motor
        M_STEP3, // full turn #3 motor
        M_STEP4, // half turn #2 motor
    } st;

    bool cycle_stop_req;

    float variSpeed(float max) {
#ifndef SIMAVR
        int a = analogRead(ANALOG_SPEED);
        float s = max * a + 10.0;
        //Serial.println(s);
#else
        float s = max * 1024;
#endif
        return s;
    }

    void start_cycle() {
        //Serial.println("start cycle, speed:");
        cycle_stop_req = false;
        s.setCurrentPosition(0);
        s2.setCurrentPosition(0);
        s3.setCurrentPosition(0);
        s.setMaxSpeed(variSpeed(MAX_SPEED_S1));
        st = M_STEP1;
        s.move((FastGPIO::Pin<SWITCH_LEN>::isInputHigh()) ? S1_LENGTH1 : S1_LENGTH2);
        //Serial.println("start cycle, enable motors 2 & 3");
        FastGPIO::Pin<MOTOR_ENABLE>::setOutput(false); // enable motors #2 and #3
    }

    void start_setup() {
        //Serial.println("start setup");
        st = M_SETUP;
        s.setMaxSpeed(variSpeed(MAX_SPEED_S1));
        s.move(400000);
    }

    void start_setup2() {
        //Serial.println("start setup");
        st = M_SETUP2;
        s2.setMaxSpeed(variSpeed(MAX_SPEED_S2));
        s2.move(400000);
    }
    
    void start_setup3() {
        //Serial.println("start setup");
        st = M_SETUP3;
        s3.setMaxSpeed(variSpeed(MAX_SPEED_S3));
        s3.move(400000);
    }

    Machine() {
        FastGPIO::Pin<BUTTON_START>::setInput(); // #button #1 (start)
        FastGPIO::Pin<BUTTON_STOP>::setInput(); // #button #2 (stop)
        FastGPIO::Pin<SWITCH_MODE>::setInput(); // #switch #1 (cycle/setup)
        FastGPIO::Pin<SWITCH_LEN>::setInput(); // #switch #2 (len1/len2)

        FastGPIO::Pin<MOTOR_ENABLE>::setOutput(true); // disable motors #2 and #3
        pinMode(ANALOG_SPEED, INPUT);
        st = M_IDLE;
    }
    void state_machine() {
        switch(st) {
        case M_IDLE: {
            if (!FastGPIO::Pin<BUTTON_START>::isInputHigh()) { // button #1 is pressed
                if (FastGPIO::Pin<SWITCH_MODE>::isInputHigh()) { // SETUP mode
                    start_setup();
                } else {
                    start_cycle();
                }
            }

            if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) {
                if (FastGPIO::Pin<SWITCH_MODE>::isInputHigh()) {
                    if (FastGPIO::Pin<SWITCH_LEN>::isInputHigh()) {
                        start_setup2();
                    } else {
                        start_setup3();
                    }
                }
            }
            break;
        }
        case M_SETUP: {
            if (FastGPIO::Pin<BUTTON_START>::isInputHigh()) { // button #1 is released
                //Serial.println("START released");
                s.stop();
                st = M_STOPPING;
            } else {
                s._currentPos = 0;
            }
            // fall through
        }
        case M_STOPPING: {
            if (!s.run()) {// stopped
                //Serial.println("go IDLE");
                st = M_IDLE;
            }
            break;
        }
        case M_SETUP2: {
            if (FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is released
                //Serial.println("START released");
                s2.stop();
                st = M_STOPPING2;
            } else {
                s2._currentPos = 0;
            }
            // fall through
        }
        case M_STOPPING2: {
            if (!s2.run()) {// stopped
                //Serial.println("go IDLE");
                st = M_IDLE;
            }
            break;
        }
        case M_SETUP3: {
            if (FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is released
                //Serial.println("START released");
                s3.stop();
                st = M_STOPPING3;
            } else {
                s3._currentPos = 0;
            }
        }
        case M_STOPPING3: {
            if (!s3.run()) {// stopped
                //Serial.println("go IDLE");
                st = M_IDLE;
            }
            break;
        }
        case M_STEP1: {
            if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is pressed
                cycle_stop_req = true;
            }
            if (!s.run()) {
                //Serial.println("go STEP2");
                s2.setMaxSpeed(variSpeed(MAX_SPEED_S2));
                s2.move(S2_LENGTH); // half-turn
                st = M_STEP2;
            }
            break;
        }
        case M_STEP2: {
            if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is pressed
                cycle_stop_req = true;
            }
            if (!s2.run()) {
                //Serial.println("go STEP3");
                s3.setMaxSpeed(variSpeed(MAX_SPEED_S3));
                s3.move(S3_LENGTH); // full-turn
                st = M_STEP3;
            }
            break;
        }
        case M_STEP3: {
            if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is pressed
                cycle_stop_req = true;
            }
            if (!s3.run()) {
                //Serial.println("go STEP4");
                s2.move(S2_LENGTH); //half-turn
                st = M_STEP4;
            }
            break;
        }
        case M_STEP4: {
            if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is pressed
                cycle_stop_req = true;
            }
            if (!s2.run()) {
                if (cycle_stop_req) {
                    FastGPIO::Pin<MOTOR_ENABLE>::setOutput(true); // disable motors #2 and #3
                    //Serial.println("go IDLE");
                    st = M_IDLE;
                } else {
                    //Serial.println("new CYCLE");
                    start_cycle();
                }
            }
            break;
        }
        }
    }
} m;

void setup() {
    //Serial.begin(115200);
    //s.setSpeed(8000);
    //s.moveTo(3200);
    //m.start_setup();
#ifdef SIMAVR
    m.start_cycle();
#endif
}

void loop() {
    //while(s.run());
    m.state_machine();
}
