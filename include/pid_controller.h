#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float compute(float setpoint, float measured, float dt);

private:
    float _kp, _ki, _kd;
    float _prevError;
    float _integral;
};

#endif // PID_CONTROLLER_H
