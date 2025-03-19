
enum convertTypeDef
{
    signedInt32bit, // 符号付き32bit整数
};

class AutoReloadRegisterConverter
{
public:
    AutoReloadRegisterConverter(convertTypeDef convertTo, int registerMaxVal, bool revers, float threshold = 0.4);
    int convert(int registerVal);
    int getConvertedVal();
    void reset(int origin);

private:
    convertTypeDef convertType;
    int isRevers = 1;
    int valOrigin = 0;
    int registerMaxVal = 0;
    int pastRegisterVal = 0;
    int convertedVal = 0;
    int overflowCount = 0;
    float overflowThreshold = 0.4;
};