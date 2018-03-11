/*
BSD 2-Clause License

Copyright (c) 2017, linorobot
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "../include/PID.h"

PID::PID(float min_val, float max_val, float band, float kp, float ki, float kd)
{
    min_val_ = min_val;
    max_val_ = max_val;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    band_ = band;
}

double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return /*constrain(pid, min_val_, max_val_);*/ pid;
}

void PID::updateConstants(float max_val, float min_val, float band, float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    band_ = band;
    min_val_ = min_val;
    max_val_ = max_val;
}

int PID::toPWM(float output)
{
  float max_output = 100;
    float min_output = -100;
    if(output > max_output)
      output = max_output;
    if(output < min_output)
      output = min_output;
    float temp = output * 255 / max_output;
    int pwm = static_cast<int>(temp);
    if(pwm > 255)
      pwm = 255;
    if(pwm < -255)
      pwm = -255;
    return pwm;
}

float PID::get_band()
{
  return band_;
}

float PID::get_prev_error()
{
  return prev_error_;
}
