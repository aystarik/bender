#pragma once

#include <Arduino.h>
#include <FastGPIO.h>

template <uint8_t step_pin, uint8_t dir_pin = IO_NONE> struct Stepper {
    typedef enum {
        DIRECTION_CCW = -1,  ///< Counter-Clockwise
        DIRECTION_CW  = 1   ///< Clockwise
    } Direction;

    Stepper() {
        _lastStepTime = 0;
        FastGPIO::Pin<step_pin>::setOutput(LOW);
        FastGPIO::Pin<dir_pin>::setOutput(LOW);
        setCurrentPosition(0);
        setAcceleration(12800.0);
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
            _c0 = 0.676 * sqrt(2 / acceleration) * 1000000.0; // Equation 15
           computeNewSpeed();
        }
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
        long stepsToStop = (long)(_speed * _speed * _inverse_accel); // Equation 16

        Direction new_direction = DIRECTION_CW;
        if (distanceTo < 0) {
            if (dir_pin != IO_NONE) {
                distanceTo = -distanceTo;
                new_direction = DIRECTION_CCW;
            } else {
                distanceTo = 0;
            }
        }

        if (distanceTo == 0) {
            // We are at the target and its time to stop
            _stepInterval = 0;
            _speed = 0.0;
            _n = 0;
            return;
        }

        if (distanceTo > 0) {
            // We are anticlockwise from the target
            // Need to go clockwise from here, maybe decelerate now
            if (_n > 0) {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) || _direction != new_direction)
                    _n = -stepsToStop; // Start deceleration
            } else if (_n < 0) {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) && _direction == new_direction)
                    _n = -_n; // Start accceleration
            }
        }

        // Need to accelerate or decelerate
        if (_n == 0) {
            // First step from stopped
            _cn = _c0;
            _cn10 = _cn << 10;
            _direction = new_direction;
            dir(_direction);
        } else {
            // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            _cn10 -= (2 * _cn10) / (4 * _n + 1); // Equation 13
            _cn = (_cn10 + 512) >> 10;
            if (_cn < _cmin) {
                _cn = _cmin;
            }
        }
        ++_n;
        _stepInterval = _cn;
        _speed = 1000000L / _cn;
        if (_direction == DIRECTION_CCW)
            _speed = -_speed;
    }

    void stop() {
        if (_speed == 0)
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
