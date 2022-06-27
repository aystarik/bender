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
