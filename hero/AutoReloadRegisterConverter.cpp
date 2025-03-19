#pragma once
#include "AutoReloadRegisterConverter.hpp"

AutoReloadRegisterConverter::AutoReloadRegisterConverter(convertTypeDef convertTo, int maxVal, bool revers, float threshold)
{
    this->convertType = convertTo;
    this->registerMaxVal = maxVal;
    this->overflowThreshold = threshold;
    this->isRevers = revers ? -1 : 1;
}

int AutoReloadRegisterConverter::convert(int registerVal)
{

    if ((registerVal - this->pastRegisterVal) <= -(this->overflowThreshold * this->registerMaxVal)) // オーバーフローした
    {
        this->overflowCount++;
    }
    else if ((registerVal - this->pastRegisterVal) > (this->overflowThreshold * this->registerMaxVal)) // アンダーフローした
    {
        this->overflowCount--;
    }
    this->convertedVal = this->isRevers * (this->overflowCount * this->registerMaxVal + registerVal) - this->valOrigin;
    this->pastRegisterVal = registerVal;
    return this->convertedVal;
}

int AutoReloadRegisterConverter::getConvertedVal()
{
    return this->convertedVal;
}

void AutoReloadRegisterConverter::reset(int origin)
{
    this->valOrigin = origin;
}