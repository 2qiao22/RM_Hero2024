#pragma once
#include <cmath>

class PID_Contoller
{
public:
    PID_Contoller(float p,float i, float d, float period);

    float CalculateOutput(float error);
    float CalculateOutput(float error,float integralError,float differentialError);
    float CalculateIntegralError(float error);
    float CalculateDifferentialError(float error);

    void SetP_gain(float setValue);
    void SetI_gain(float setValue);
    void SetD_gain(float setValue);
    void SetIntegralError(float setValue);
    void SetDifferentialError(float setValue);
    void SetPastError(float setValue);
    void SetContolPeriod(float setValue);
    
    float GetP_gain();
    float GetI_gain();
    float GetD_gain();
    float GetIntegralError();
    float GetDifferentialError();
    float GetpastError();
    float GetControlPeriod();

private:
    float P_gain = 0.0;//Pゲイン
    float D_gain = 0.0;//Dゲイン
    float I_gain = 0.0;//Iゲイン
    float integralError = 0.0;//偏差の時間積分
    float differentialError = 0.0;//偏差の時間微分
    float pastError = 0.0;//1制御周期前の偏差
    float controlPeriod = 10.0;//制御ループの周波数[Hz]
};