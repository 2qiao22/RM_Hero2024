class AutoReloadRegisterConverterF
{
public:
    AutoReloadRegisterConverterF(float registerMaxVal, bool revers, float threshold = 0.4);
    float convert(float registerVal);
    float getConvertedVal();
    void reset(float origin);

private:
    float isRevers = 1;
    float valOrigin = 0;
    float registerMaxVal = 0;
    float pastRegisterVal = 0;
    float convertedValFloat = 0;
    float convertedVal = 0;
    float overflowCount = 0;
    float overflowThreshold = 0.4;
};