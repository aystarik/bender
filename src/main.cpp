#include <Arduino.h>
#include <FastGPIO.h>

#include <stepper.h>


#define BUTTON_START IO_B0
#define BUTTON_STOP  IO_B1
#define SWITCH_MODE  IO_B2
#define SWITCH_LEN   IO_B3

#define ANALOG_SPEED A7

#define MOTOR1_STEP IO_D2
#define MOTOR2_STEP IO_D3
#define MOTOR3_STEP IO_D4

#define MOTOR_ENABLE IO_D5

#if 0 // almost direct import of famous Motor.cc code, allow for 65us step updates
template <uint16_t accel, uint8_t step_pin = IO_NONE, uint8_t dir_pin = IO_NONE> struct Stepper2 {
    enum Direction {
        DIRECTION_CCW = -1,  ///< Counter-Clockwise
        DIRECTION_CW  = 1   ///< Clockwise
    } _dir;
    enum RampState {
        RAMP_IDLE = 0,
        RAMP_UP,
        RAMP_MAX,
        RAMP_DOWN,
        RAMP_LAST,
    } ramp_sts = RAMP_IDLE;
    uint16_t _c0, _cmin;
    uint16_t c;          // integer delay count
    uint32_t step_no;    // progress of move
    uint32_t step_down;  // start of down-ramp
    int16_t denom; // 4.n+1 in ramp algo
    int32_t motor_pos = 0; // absolute step number
    uint32_t move;       // total steps to move
    uint32_t midpt;      // midpoint of move
    uint32_t c32;        // 24.8 fixed point delay count
    unsigned long _prev;
    bool run_flg;    // true while motor is running

    Stepper2() {
        FastGPIO::Pin<step_pin>::setOutput(LOW);
        FastGPIO::Pin<dir_pin>::setOutput(LOW);
        _c0 = 956000 / sqrt(accel);
    }

    void setSpeed(uint16_t speed) {
        _cmin = 1000000/speed;
    }

    void stepHigh() {FastGPIO::Pin<step_pin>::setOutputValueHigh();}
    void stepLow() {FastGPIO::Pin<step_pin>::setOutputValueLow();}
    void dir(Direction d) {FastGPIO::Pin<dir_pin>::setOutputValue(d == DIRECTION_CW);}

    void moveTo(int32_t pos_new) { // set up to drive motor to pos_new (absolute step#)
        if (pos_new == motor_pos)
            return;
        _prev = micros();
        if (pos_new < motor_pos) { // get direction & #steps
            move = motor_pos - pos_new;
            _dir = DIRECTION_CCW;
        } else {
            move = pos_new - motor_pos;
            _dir = DIRECTION_CW;
        }
        dir(_dir);
        stepHigh();
        midpt = (move - 1) >> 1;
        c = _c0;
        c32 = (uint32_t)c << 10; // keep c in 24.8 fixed-point format for ramp calcs
        step_no  = 0; // step counter
        denom    = 1; // 4.n+1, n=0
        ramp_sts = RAMP_UP; // start ramp state-machine
        run_flg  = true;
        stepLow();
    }

    bool run() {
        if (!run_flg)
            return run_flg;
        unsigned long time = micros();
        if (time - _prev < c)
            return false;
        stepHigh();
        _prev = time;
        switch (ramp_sts) {
        case RAMP_UP:   // accel
            if (step_no == midpt) { // midpoint: decel
                ramp_sts = RAMP_DOWN;
                denom = ((step_no - move) << 2) + 1;
                if (!(move & 1)) { // even move: repeat last delay before decel
                    denom += 4;
                    break;
                }
            }
            // no break: share code for ramp algo
        case RAMP_DOWN: // decel
            if (step_no == move - 1) { // next irq is cleanup (no step)
                ramp_sts = RAMP_LAST;
                break;
            }
            denom += 4;
            c32 -= (c32 << 1) / denom; // ramp algorithm
            // beware confict with foreground code if long div not reentrant
            c = (c32 + 512) >> 10; // round 24.8format->int16
            if (c <= _cmin) { // go to constant speed
                ramp_sts = RAMP_MAX;
                step_down = move - step_no;
                c = _cmin;
            }
            break;
        case RAMP_MAX: // constant speed
            if (step_no == step_down) { // start decel
                ramp_sts = RAMP_DOWN;
                denom = ((step_no - move) << 2) + 5;
            }
            break;
        default: // last step: cleanup
            ramp_sts = RAMP_IDLE;
            run_flg = false; // move complete
            break;
        }

        if (ramp_sts != RAMP_IDLE) {
            motor_pos += _dir;
            ++step_no;
        }
        stepLow();
        return run_flg;
    }
};

Stepper2<25600, MOTOR1_STEP> s;

#endif

#if 1
struct Machine {
    static constexpr float MAX_SPEED_S1 = 7200.0f/1024.0f;
    static constexpr float MAX_SPEED_S2 = 7200.0f/1024.0f;
    static constexpr float MAX_SPEED_S3 = 7200.0f/1024.0f;

    static constexpr unsigned long S1_LENGTH1 = 392UL;
    static constexpr unsigned long S1_LENGTH2 = 660UL;

    static constexpr unsigned long S2_LENGTH = 5*800UL;
    static constexpr unsigned long S3_LENGTH = 5*1600UL;

    Stepper<MOTOR1_STEP> s;
    Stepper<MOTOR2_STEP> s2;
    Stepper<MOTOR3_STEP> s3;
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

    float variSpeed(float max) {
        int a = analogRead(ANALOG_SPEED);
        float s = max * a + 10.0;
        //Serial.println(s);
        //float s = max * 1024;
        return s;
    }

    void start_cycle() {
        //Serial.println("start cycle, speed:");
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
        if (!FastGPIO::Pin<BUTTON_STOP>::isInputHigh()) { // button #2 is pressed
            stop_request = true;
        }
        switch(st) {
        case M_IDLE: {
            if (!FastGPIO::Pin<BUTTON_START>::isInputHigh()) { // button #1 is pressed
                //Serial.println("START pressed");
                stop_request = false;
                if (FastGPIO::Pin<SWITCH_MODE>::isInputHigh()) { // switch #1 is high
                    //Serial.println("CYCLE selected");
                    start_setup();
                } else {
                    //Serial.println("SETUP selected");
                    start_cycle();
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
            if (!s.run()) {
                //Serial.println("go IDLE");
                st = M_IDLE;
            }
            break;
        }
        case M_STOPPING: {
            if (!s.run()) {// stopped
                //Serial.println("go IDLE");
                st = M_IDLE;
            }
            break;
        }
        case M_STEP1: {
            if (!s.run()) {
                //Serial.println("go STEP2");
                s2.setMaxSpeed(variSpeed(MAX_SPEED_S2));
                s2.move(S2_LENGTH); // half-turn
                st = M_STEP2;
            }
            break;
        }
        case M_STEP2: {
            if (!s2.run()) {
                //Serial.println("go STEP3");
                s3.setMaxSpeed(variSpeed(MAX_SPEED_S3));
                s3.move(S3_LENGTH); // full-turn
                st = M_STEP3;
            }
            break;
        }
        case M_STEP3: {
            if (!s3.run()) {
                //Serial.println("go STEP4");
                s2.move(S2_LENGTH); //half-turn
                st = M_STEP4;
            }
            break;
        }
        case M_STEP4: {
            if (!s2.run()) {
                if (stop_request) {
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
#endif

void setup() {
    //Serial.begin(115200);
    //s.setSpeed(8000);
    //s.moveTo(3200);
    //m.start_setup();
    //m.start_cycle();
}

void loop() {
    //while(s.run());
    m.state_machine();
}
