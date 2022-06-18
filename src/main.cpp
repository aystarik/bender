#include <Arduino.h>
#include <FastGPIO.h>

#include <stepper.h>

struct Machine {
    static constexpr float MAX_SPEED_S1 = 400.0f/1024.0f;
    static constexpr float MAX_SPEED_S2 = 400.0f/1024.0f;
    static constexpr float MAX_SPEED_S3 = 400.0f/1024.0f;

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
        Serial.println("start cycle");
        int a = analogRead(A0);
        float speed = MAX_SPEED_S1 * a;
        s.setSpeed(speed);
        st = M_STEP1;
        s._currentPos = s2._currentPos = s3._currentPos = 0;
        s.move((FastGPIO::Pin<IO_B3>::isInputHigh()) ? S1_LENGTH1 : S1_LENGTH2);
        FastGPIO::Pin<IO_D5>::setOutput(false); // enable motors #2 and #3
    }

    void start_setup() {
        Serial.println("start setup");
        st = M_SETUP;
        int a = analogRead(A7);
        float speed = MAX_SPEED_S1 * a;
        s.setSpeed(speed);
        s.move(400000);
    }

    Machine() {
        FastGPIO::Pin<IO_B0>::setInput(); // #button #1 (start)
        FastGPIO::Pin<IO_B1>::setInput(); // #button #2 (stop)
        FastGPIO::Pin<IO_B2>::setInput(); // #switch #1 (cycle/setup)
        FastGPIO::Pin<IO_B3>::setInput(); // #switch #2 (len1/len2)

        FastGPIO::Pin<IO_D5>::setOutput(true); // disable motors #2 and #3
        pinMode(A7, INPUT);
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
                int a = analogRead(A7);
                float speed = MAX_SPEED_S2 * a;
                s2.setSpeed(speed);
                s2.move(S2_LENGTH); // half-turn
                st = M_STEP2;
            }
            break;
        }
        case M_STEP2: {
            if (!s2.run()) {
                int a = analogRead(A7);
                float speed = MAX_SPEED_S3 * a;
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
#if 0
        FastGPIO::Pin<IO_B0>::setInput(); // #button #1 (start)
        FastGPIO::Pin<IO_B1>::setInput(); // #button #2 (stop)
        FastGPIO::Pin<IO_B2>::setInput(); // #switch #1 (cycle/setup)
        FastGPIO::Pin<IO_B3>::setInput(); // #switch #2 (len1/len2)
#endif
    Serial.begin(115200);
    //m.start_setup();
    //m.start_cycle();
}

void loop() {
    bool k1 = FastGPIO::Pin<IO_B0>::isInputHigh();
    bool k2 = FastGPIO::Pin<IO_B1>::isInputHigh();
    bool p1 = FastGPIO::Pin<IO_B2>::isInputHigh();
    bool p2 = FastGPIO::Pin<IO_B3>::isInputHigh();
    int speed = analogRead(A7);
    if (!k1) {
        Serial.print("START: ");
    }
    if (!k2) {
        Serial.print("STOP: ");
    }
    if (p1) {
        Serial.print("SETUP: ");
    } else {
        Serial.print("CYCLE: ");
    }
    if (p2) {
        Serial.print("P #1: ");
    } else {
        Serial.print("P #2: ");
    }
    Serial.println(speed);
    delay(500);
    //m.state_machine();
}
