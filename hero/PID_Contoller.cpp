#include "PID_Contoller.hpp"

PID_Contoller::PID_Contoller(float p, float i, float d, float period)
{
    this->P_gain = p;
    this->I_gain = i;
    this->D_gain = d;

    this->controlPeriod = period;
}

float PID_Contoller::CalculateOutput(float error, float integralError, float differentialError)
{
    float out = 0.0;
    out = this->P_gain * error + this->I_gain * integralError + this->D_gain * differentialError;
    return out;
}

float PID_Contoller::CalculateOutput(float error)
{
    float out = 0.0;
    float integralError = this->CalculateIntegralError(error);
    float differentialError = this->CalculateDifferentialError(error);
    out = this->P_gain * error + this->I_gain * integralError + this->D_gain * differentialError;
    return out;
}

float PID_Contoller::CalculateIntegralError(float error)
{
    this->integralError += error / this->controlPeriod;
    return this->integralError;
}

float PID_Contoller::CalculateDifferentialError(float error)
{
    this->differentialError = (error - this->pastError) * this->controlPeriod;
    this->pastError = error;
    return this->differentialError;
}

void PID_Contoller::SetP_gain(float setValue)
{
    this->P_gain = setValue;
}

void PID_Contoller::SetI_gain(float setValue)
{
    this->I_gain = setValue;
}

void PID_Contoller::SetD_gain(float setValue)
{
    this->D_gain = setValue;
}

void PID_Contoller::SetIntegralError(float setValue)
{
    this->integralError = setValue;
}

void PID_Contoller::SetDifferentialError(float setValue)
{
    this->differentialError = setValue;
}

void PID_Contoller::SetPastError(float setValue)
{
    this->pastError = setValue;
}

void PID_Contoller::SetContolPeriod(float setValue)
{
    this->controlPeriod = setValue;
}

float PID_Contoller::GetP_gain()
{
    return this->P_gain;
}

float PID_Contoller::GetI_gain()
{
    return this->I_gain;
}

float PID_Contoller::GetD_gain()
{
    return this->D_gain;
}

float PID_Contoller::GetIntegralError()
{
    return this->integralError;
}

float PID_Contoller::GetDifferentialError()
{
    return this->differentialError;
}

float PID_Contoller::GetpastError()
{
    return this->pastError;
}

float PID_Contoller::GetControlPeriod()
{
    return this->controlPeriod;
}
