#include <Arduino.h>
#include <FastGPIO.h>

#include <stepper.h>

struct Machine {
    static constexpr float MAX_SPEED_S1 = 400.0f;
    static constexpr float MAX_SPEED_S2 = 400.0f;
    static constexpr float MAX_SPEED_S3 = 400.0f;

    static constexpr unsigned long S1_LENGTH1 = 400UL;
    static constexpr unsigned long S1_LENGTH2 = 200UL;

    static constexpr unsigned long S2_LENGTH = 800UL;
    static constexpr unsigned long S3_LENGTH = 1600UL;

    Stepper<IO_D2> s;
    Stepper<IO_D3> s2;
    Stepper<IO_D4> s3;
    enum State {
        M_IDLE,
        M_SETUP, // advance wire while button #1 is pressed
        M_STOPPING,
        M_STEP1, // advance wire 20mm (1/4 turn)
        M_STEP2, // half turn #2 motor
        M_STEP3, // full turn #3 motor
        M_STEP4, // half turn #2 motor
    } st;

    bool stop_request;

    void start_cycle() {
        int a = analogRead(A0);
        float speed = MAX_SPEED_S1 * a / 1024.0;
        s.setSpeed(speed);
        st = M_STEP1;
        s.move((FastGPIO::Pin<IO_B3>::isInputHigh()) ? S1_LENGTH1 : S1_LENGTH2);
        FastGPIO::Pin<IO_D5>::setOutput(false); // enable motors #2 and #3
    }

    void start_setup() {
        st = M_SETUP;
        int a = analogRead(A0);
        float speed = MAX_SPEED_S1 * a / 1024.0;
        s.setSpeed(speed);
        s.move(400000);
    }

    Machine() {
        FastGPIO::Pin<IO_B0>::setInputPulledUp(); // #button #1 (start)
        FastGPIO::Pin<IO_B1>::setInputPulledUp(); // #button #2 (stop)
        FastGPIO::Pin<IO_B2>::setInputPulledUp(); // #switch #1 (cycle/setup)
        FastGPIO::Pin<IO_B3>::setInputPulledUp(); // #switch #2 (len1/len2)

        FastGPIO::Pin<IO_D5>::setOutput(true); // enable for motors #2 and #3

        pinMode(A0, INPUT);
        st = M_IDLE;
    }
    void state_machine() {
        if (!FastGPIO::Pin<IO_B1>::isInputHigh()) { // button #2 is pressed
            stop_request = true;
        }
        switch(st) {
        case M_IDLE: {
            if (!FastGPIO::Pin<IO_B0>::isInputHigh()) { // button #1 is pressed
                stop_request = false;
                if (FastGPIO::Pin<IO_B2>::isInputHigh()) { // switch #1 is high
                    start_setup();
                } else {
                    start_cycle();
                }
            }
            break;
        }
        case M_SETUP: {
            if (FastGPIO::Pin<IO_B0>::isInputHigh()) { // button #1 is released
                s.stop();
                st = M_STOPPING;
            } else {
                s._currentPos = 0;
            }
            s.run();
            break;
        }
        case M_STOPPING: {
            if (!s.run()) {// stopped
                st = M_IDLE;
            }
            break;
        }
        case M_STEP1: {
            if (!s.run()) {
                int a = analogRead(A0);
                float speed = MAX_SPEED_S2 * a / 1024.0;
                s2.setSpeed(speed);
                s2.move(S2_LENGTH); // half-turn
                st = M_STEP2;
            }
            break;
        }
        case M_STEP2: {
            if (!s2.run()) {
                int a = analogRead(A0);
                float speed = MAX_SPEED_S3 * a / 1024.0;
                s3.setSpeed(speed);
                s3.move(S3_LENGTH); // full-turn
                st = M_STEP3;
            }
            break;
        }
        case M_STEP3: {
            if (!s3.run()) {
                s2.move(S2_LENGTH); //half-turn
                st = M_STEP4;
            }
            break;
        }
        case M_STEP4: {
            if (!s2.run()) {
                if (stop_request) {
                    FastGPIO::Pin<IO_D5>::setOutput(true); // disable motors #2 and #3
                    st = M_IDLE;
                } else {
                    start_cycle();
                }
            }
            break;
        }
        }
    }
} m;

void setup() {
    //m.start_setup();
    //m.start_cycle();
}

void loop() {
    m.state_machine();
}
