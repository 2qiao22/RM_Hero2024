#pragma once
#include "AutoReloadRegisterConverterF.hpp"

AutoReloadRegisterConverterF::AutoReloadRegisterConverterF(float maxVal, bool revers, float threshold)
{
    this->registerMaxVal = maxVal;
    this->overflowThreshold = threshold;
    this->isRevers = revers ? -1 : 1;
}

float AutoReloadRegisterConverterF::convert(float registerVal)
{
    if ((registerVal - this->pastRegisterVal) <= -(this->overflowThreshold * this->registerMaxVal)) // オーバーフローした
    {
        this->overflowCount++;
    }
    else if ((registerVal - this->pastRegisterVal) > (this->overflowThreshold * this->registerMaxVal)) // アンダーフローした
    {
        this->overflowCount--;
    }
    this->convertedValFloat = this->isRevers * (this->overflowCount * this->registerMaxVal + registerVal) - this->valOrigin;
    this->pastRegisterVal = registerVal;
    return this->convertedValFloat;
}

float AutoReloadRegisterConverterF::getConvertedVal()
{
    return this->convertedValFloat;
}

void AutoReloadRegisterConverterF::reset(float origin)
{
    this->valOrigin = origin;
}