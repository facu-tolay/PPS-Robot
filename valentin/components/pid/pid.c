
#include "pid.h"

void PID_Compute(PID_params_t *params_in)
{
    float _integral = params_in -> _integral;
    float _pre_error = params_in -> _pre_error;
    float rpm_destino = params_in -> rpm_destino;
    float rpm_actual = params_in -> rpm_actual;
    signed int output = params_in -> output;

    // Calculate error
    float error = rpm_destino - rpm_actual;

    if((error > 0 && error < 1.5) || (error < 0 && error > -1.5))
    {
        //_pre_error = 0;
        //_integral = 0;
    }
    else
    {
        float Pout = _Kp * error; // Proportional term

        _integral += error * _dt; // Integral term
        float Iout = _Ki * _integral;

        float derivative = (error - _pre_error) / _dt; // Derivative term
        float Dout = _Kd * derivative;

        // Calculate total output
        output = Pout + Iout + Dout;

        // Restrict to max/min
        if(rpm_destino > 0)
        {
            output += MIN_PWM_VALUE;

            if(output >= MAX_PWM_VALUE)
            {
                output = MAX_PWM_VALUE;
            }
            else if(output <= MIN_PWM_VALUE)
            {
                output = MIN_PWM_VALUE;
            }
        }
        else if(rpm_destino < 0)
        {
            output -= MIN_PWM_VALUE;

            if(output <= -MAX_PWM_VALUE)
            {
                output = -MAX_PWM_VALUE;
            }
            else if(output >= -MIN_PWM_VALUE)
            {
                output = -MIN_PWM_VALUE;
            }
        }

        //printf("err=%4.2f/ pre=%4.2f/ Pout=%4.2f/ Iout=%4.2f/ Dout=%4.2f/ OUT=%d\n", error, _pre_error, Pout, Iout, Dout, output);

        // Save error to previous error
        _pre_error = error;
    }

    params_in -> _integral = _integral;
    params_in -> _pre_error = _pre_error;
    params_in -> output = output;
}

void reset_pid_state(PID_params_t *pid_params)
{
    pid_params->_integral = 0;
    pid_params->_pre_error = 0;
}
