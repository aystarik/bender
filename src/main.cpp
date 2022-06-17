#include <Arduino.h>
#include <FastGPIO.h>

template <uint8_t step_pin, uint8_t dir_pin = IO_NONE> struct Stepper {
    typedef enum {
        DIRECTION_CCW = -1,  ///< Counter-Clockwise
        DIRECTION_CW  = 1   ///< Clockwise
    } Direction;

    Stepper() : _lastStepTime(0) {
        FastGPIO::Pin<step_pin>::setOutput(LOW);
        FastGPIO::Pin<dir_pin>::setOutput(LOW);
        setCurrentPosition(0);
        setAcceleration(25000.0);
        setMaxSpeed(8000.0);
    }

    void stepHigh() {FastGPIO::Pin<step_pin>::setOutputValueHigh();}
    void stepLow() {FastGPIO::Pin<step_pin>::setOutputValueLow();}
    void dir(Direction d) {FastGPIO::Pin<dir_pin>::setOutputValue(d == DIRECTION_CW);}

    float _maxSpeed;
    long _speed;
    float _acceleration, _inverse_accel;
    unsigned long _stepInterval;
    long _n;
    unsigned long _cn, _c0, _cmin;
    long _cn10;
    long _targetPos, _currentPos;
    Direction _direction;

    void setCurrentPosition(long position) {
        _targetPos = _currentPos = position;
        _n = 0;
        _stepInterval = 0;
        _speed = 0.0;
    }

    void setMaxSpeed(float speed) {
        if (speed < 0.0)
            speed = -speed;
        if (_maxSpeed != speed) {
            _maxSpeed = speed;
            _cmin = 1000000.0 / speed;
            // Recompute _n from current speed and adjust speed if accelerating or cruising
            if (_n > 0) {
                _n = (long)(float(_speed * _speed) * _inverse_accel); // Equation 16
                computeNewSpeed();
            }
        }
    }

    void setAcceleration(float acceleration) {
        if (acceleration == 0.0)
            return;
        if (acceleration < 0.0)
            acceleration = -acceleration;
        if (_acceleration != acceleration) {
            // Recompute _n per Equation 17
            _n = _n * (_acceleration / acceleration);
            // New c0 per Equation 7, with correction per Equation 15
             _acceleration = acceleration;
            _inverse_accel = 0.5 / acceleration;
            _c0 = 2.0 * 0.676 * sqrt(_inverse_accel) * 1000000.0; // Equation 15
           computeNewSpeed();
        }
    }

    void setSpeed(float speed) {
        if (speed == _speed)
            return;
        speed = constrain(speed, -_maxSpeed, _maxSpeed);
        if (speed == 0.0)
            _stepInterval = 0;
        else {
            _stepInterval = fabs(1000000.0 / speed);
            _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
        }
        _speed = speed;
    }

    void moveTo(long absolute) {
        if (_targetPos != absolute) {
            _targetPos = absolute;
            computeNewSpeed();
            // compute new n?
        }
    }

    void move(long relative) {
        moveTo(_currentPos + relative);
    }

    long distanceToGo() {
        return _targetPos - _currentPos;
    }
    unsigned long _lastStepTime;
    bool runSpeed() {
        // Dont do anything unless we actually have a step interval
        if (!_stepInterval)
            return false;

        unsigned long time = micros();
        if (time - _lastStepTime < _stepInterval)
            return false;
        _currentPos += _direction;

        stepHigh();

        _lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }

    void computeNewSpeed() {
        long distanceTo = distanceToGo(); // +ve is clockwise from curent location
        //FastGPIO::Pin<IO_B0>::setOutputHigh();
        long stepsToStop = (long)(_speed * _speed * _inverse_accel); // Equation 16
        //FastGPIO::Pin<IO_B0>::setOutputLow();

        if (distanceTo == 0 && stepsToStop <= 1) {
            // We are at the target and its time to stop
            _stepInterval = 0;
            _speed = 0.0;
            _n = 0;
            return;
        }
        Direction new_direction = DIRECTION_CW;
        if (distanceTo < 0) {
            distanceTo = -distanceTo;
            new_direction = DIRECTION_CCW;
        }
        // Need to accelerate or decelerate
        if (_n == 0) {
            // First step from stopped
            _cn = _c0;
            _cn10 = _cn << 10;
            _direction = new_direction;
            dir(_direction);
        } else {
            // Need to go clockwise from here, maybe decelerate now
            if (_n > 0) {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) || _direction != new_direction)
                    _n = -stepsToStop; // Start deceleration
            } else {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) && _direction == new_direction)
                    _n = -_n; // Start accceleration
            }
            //FastGPIO::Pin<IO_B1>::setOutputHigh();
            // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            _cn10 -= (2 * _cn10) / (4 * _n + 1); // Equation 13
            _cn = _cn10 >> 10;
            if (_cn < _cmin) {
                _cn = _cmin;
            }
            //FastGPIO::Pin<IO_B1>::setOutputLow();
        }

        ++_n;
        _stepInterval = _cn;
        _speed = 1000000UL / _cn;
        if (_direction == DIRECTION_CCW)
            _speed = -_speed;
    }

    void stop() {
        if (_speed == 0.0)
            return;
        long stepsToStop = (long)((_speed * _speed * _inverse_accel)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }

    bool run() {
        if (runSpeed()) {
            computeNewSpeed();
            stepLow();
        }
        return _speed != 0.0 || distanceToGo() != 0;
    }
};

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
                    st = M_SETUP;
                    int a = analogRead(A0);
                    float speed = MAX_SPEED_S1 * a / 1024.0;
                    s.setSpeed(speed);
                    s.move(400000);

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
    m.start_cycle();
}

void loop() {
    m.state_machine();
}
