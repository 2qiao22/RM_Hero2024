#include <FlexCAN_T4.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "const.h"
#include "operate.hpp"
#include "Mecanum2.hpp"
#include "PID_Contoller.hpp"
#include "AutoReloadRegisterConverter.hpp"
#include "AutoReloadRegisterConverterF.hpp"

unsigned long controlTime = 0;
int controlCount = 0; // 制御ループが実行された回数

int chassisPower = POWER_FLAG;

int inputRawVoltage = 0; // キャパシタユニットからの電源電圧の生データ
int capacitorRawVoltage = 0; // キャパシタユニットからのキャパシタ電圧の生データ

Mecanum2 mecanum(WHEEL_WIDTH_DISTANCE, WHEEL_VERTICAL_DISTANCE);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire); // IMU sensor
imu::Vector<3> euler;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; // can2 port

static CAN_message_t can2_0x1ff; // roller右左,ピッチ
static CAN_message_t can1_0x200; // 足回り
static CAN_message_t can1_0x1ff; // Yaw GM6020,loader C620
static CAN_message_t rxmsg;

typedef struct djiEscDataSt
{
    uint16_t angle;
    int16_t rotation;
    int16_t torque;
    short temp;
};
/*受信用配列
 *0~3:chassis 0:右前　1:左前　2:右後　3;左後
 *4,5:gimbal 4:ヨー 5:ピッチ
 *6  :fire　
 *7,8:roller 7:右　8:左
 */
djiEscDataSt escData[9];

float chassisReferenceCurrent[4] = {0.0, 0.0, 0.0, 0.0}; // 足回りへの指令電流[A] 0:右前　1:左前　2:右後　3;左後
float gimbalReferenceCurrent[2] = {0.0, 0.0};            // ジンバルへの指令電圧[V] (ただしバッテリー電圧は24Vとする) 0:ヨー　1:ピッチ
float fireReferenceCurrent = 0.0;                        // 弾送りへの電流指令値[A]
float rollerReferenceCurrent[2] = {0.0, 0.0};            // 発射ローラーへの電流指令値[A] 0:右　1:左

float wheelSpeed[4] = {0.0, 0.0, 0.0, 0.0};          // 足回りのタイヤの速度[rps] 0:右前　1:左前　2:右後　3;左後
float gimbalSpeed[2] = {0.0, 0.0};                   // ジンバルの速度[rps] 0:ヨー　1:ピッチ
float fireSpeed = 0.0;                               // 弾送りの回転速度[rps]
float rollerSpeed[2] = {0.0, 0.0};                   // 発射ローラーの速度 0:右　1:左
float RobotReferenceSpeed[3] = {0.0, 0.0, 0.0};      // 足回りを基準としたロボットの目標速度　0:X右が正[mm/s] 1:Y前が正[mm/s] 2:θ:反時計回りが正[rad/s]
float wheelReferenceSpeed[4] = {0.0, 0.0, 0.0, 0.0}; // 足回りのタイヤの目標速度[rps] 0:右前　1:左前　2:右後　3;左後
float fireReferenceSpeed = 0.0;                      // 弾送りの目標回転速度[rps]

float gimbalAngle[2] = {0.0, 0.0};          // 足回りに対するジンバルの角度[deg] 0:ヨー　1:ピッチ
float gimbalReferenceAngle[2] = {0.0, 0.0}; // 足回りに対するジンバルの目標角度[deg] 0:ヨー　1:ピッチ

typedef enum robot_state
{                  // ロボットの状態表示
    Launch = 0,    // 立ち上げ処理
    Normal = 1,    // 通常モード
    Beyblade = 2,  // 無限回転モード
    SpeedStar = 3, // 速度制限解除モード
    Stop = 4       // 停止モード、死んだときとかはここ
};
robot_state robotState = Stop;

PID_Contoller pidWheelSpeed[4] = {
    PID_Contoller(0.0405, 3.35, 0.000006, 1000),
    PID_Contoller(0.0375, 3.07, 0.000011, 1000),
    PID_Contoller(0.0410, 3.75, 0.000006, 1000),
    PID_Contoller(0.0353, 3.14, 0.000008, 1000)};

PID_Contoller pidGimbalAngle[2] = {
    PID_Contoller(0.8, 0.3, 0.0, 1000),
    PID_Contoller(1.5, 0.0, 0.0, 1000)};

PID_Contoller pidRollerSpeed[2] = {
    PID_Contoller(5.0, 0.0, 0.0, 1000),
    PID_Contoller(5.0, 0.0, 0.0, 1000)};

PID_Contoller pidFireSpeed(27, 380.0, 0.0, 1000);

AutoReloadRegisterConverter gimbalYawAngle(signedInt32bit, 8191, true);
AutoReloadRegisterConverterF gimbalImuAngle(360.0, true);
void setup()
{
    Serial.begin(912600);

    pinMode(11, OUTPUT);
    pinMode(13, INPUT);

    operate.begin();

    can1.begin();
    can1.setBaudRate(1000000);
    can2.begin();
    can2.setBaudRate(1000000);

    if (!bno.begin())
    {
        digitalWrite(11, HIGH); // ブザー鳴らす
        while (1)
        {
        }
    }
    can2_0x1ff.id = 0x1FF;
    can2_0x1ff.len = 8;
    can1_0x200.id = 0x200;
    can1_0x200.len = 8;
    can1_0x1ff.id = 0x1FF;
    can1_0x1ff.len = 8;
    robotState = Stop;
    controlTime = 0;
}

void loop()
{
    controlCount++;

    operate.update();
    //euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    if (chassisPower == 0 || operate.LS() < 2)
    { // 死んだorコントローラOFF
        robotState = Stop;
    }
    else if (robotState != Launch && robotState != Stop)
    {
        switch (operate.RS())
        {
        case 3:
            robotState = Normal;
            break;
        case 1:
            robotState = Beyblade;
            break;
        case 2:
            robotState = SpeedStar;
            break;
        }
    }

    // CANから情報を取得
    if (chassisPower > 0)
    {
        chassisPower--;
    }
    while (can1.read(rxmsg))
    {
        chassisPower = POWER_FLAG;
        switch (rxmsg.id)
        {
        case 0x201: // Wheel右前 C620 ID1
        {
            escData[0].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[0].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[0].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[0].temp = rxmsg.buf[6];
            break;
        }
        case 0x202: // Wheel左前 C620 ID2
        {
            escData[1].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[1].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[1].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[1].temp = rxmsg.buf[6];
            break;
        }
        case 0x203: // Wheel左後 C620 ID3
        {
            escData[2].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[2].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[2].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[2].temp = rxmsg.buf[6];
            break;
        }
        case 0x204: // Wheel右後 C620 ID4
        {
            escData[3].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[3].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[3].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[3].temp = rxmsg.buf[6];
            break;
        }
        case 0x205: // yaw GM6020 ID1
        {
            escData[4].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[4].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[4].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[4].temp = rxmsg.buf[6];
            break;
        }
        case 0x207: // loader C620 ID7
        {
            escData[6].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[6].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[6].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[6].temp = rxmsg.buf[6];
            break;
        }
        case 0x211:
        {
            inputRawVoltage = rxmsg.buf[1] * 256 + rxmsg.buf[0];     // 入力電圧
            capacitorRawVoltage = rxmsg.buf[3] * 256 + rxmsg.buf[2]; // コンデンサ電圧
            break;
        }
        }
    }
    while (can2.read(rxmsg))
    {
        switch (rxmsg.id)
        {
        case 0x207: // roller右 C620 ID7
        {
            escData[7].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[7].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[7].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[7].temp = rxmsg.buf[6];
            break;
        }
        case 0x208: // roller左 C620 ID8
        {
            escData[8].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[8].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[8].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[8].temp = rxmsg.buf[6];
            break;
        }
        case 0x206: // pitch GM6020 ID2
        {
            escData[5].angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
            escData[5].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
            escData[5].torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
            escData[5].temp = rxmsg.buf[6];
            break;
        }
        }
    }

    // wheelSpeedを計算
    for (int i = 0; i < 4; i++)
    {
        wheelSpeed[i] = escData[i].rotation / 60.0 / M3508_GEAR_RATIO;
    }

    // gimbalSpeedを計算
    gimbalSpeed[0] = escData[4].rotation / 60.0;
    gimbalSpeed[0] = escData[5].rotation / 60.0;

    // fireSpeedを計算
    fireSpeed = escData[6].rotation / 60.0 / M3508_GEAR_RATIO;

    // rollerSpeedを計算
    rollerSpeed[0] = escData[7].rotation / 60.0;
    rollerSpeed[1] = escData[8].rotation / 60.0;

    // ジンバルのヨーを計算
    gimbalAngle[0] = gimbalYawAngle.convert(escData[4].angle) / 8192.0 * 360.0;
    gimbalImuAngle.convert(euler.x());

    // ジンバルのピッチを計算
    gimbalAngle[1] = (float)(escData[5].angle - CENTER_PITCH_RAW) / (float)(MAX_PITCH_RAW - MIN_PITCH_RAW) * 15.0;

    switch (robotState)
    {
    case Launch:
    {
        gimbalReferenceAngle[0] = gimbalImuAngle.getConvertedVal();
        robotState = Normal;
        break;
    }
    case Stop:
    {
        for (int i = 0; i < 4; i++)
        {
            chassisReferenceCurrent[i] = 0.0;
        }
        for (int i = 0; i < 2; i++)
        {
            gimbalReferenceCurrent[i] = 0.0;
        }
        fireReferenceCurrent = 0.0;
        for (int i = 0; i < 2; i++)
        {
            rollerReferenceCurrent[i] = -0.02 * rollerSpeed[i];
        }
        if (chassisPower > 0 && operate.LS() != 1)
        {
            robotState = Launch;
        }
        break;
    }
    case Normal:
    {
        // 弾送りの目標速度を変更
        static int fireReferenceSpeedCounter = 0;
        fireReferenceSpeedCounter++;
        fireReferenceSpeedCounter = fireReferenceSpeedCounter > 500 ? 0 : fireReferenceSpeedCounter;
        if(fireReferenceSpeedCounter < 300)
        {
            fireReferenceSpeed = 1.40 * (float)operate.SR() / 660.0;
        }
        else if(fireReferenceSpeedCounter < 350)
        {
            fireReferenceSpeed = -1.0 * (float)operate.SR() / 660.0;
        }
        else
        {
            fireReferenceSpeed = 0.0;
        }

        // if (controlCount % 20000 < 5000)
        // {
        //    fireReferenceSpeed = 1.0;
        // }
        // else if (controlCount % 20000 < 10000)
        // {
        //     fireReferenceSpeed = 0.0;
        // }
        // else if (controlCount % 20000 < 15000)
        // {
        //     fireReferenceSpeed = -1.0;
        // }
        // else
        // {
        //     fireReferenceSpeed = 0.0;
        // }

        // ジンバルの目標角度を計算
        gimbalReferenceAngle[0] += -0.1 * (float)operate.RX() / 660.0;
        gimbalReferenceAngle[1] += 0.05 * (float)operate.RY() / 660.0;
        gimbalReferenceAngle[1] = gimbalReferenceAngle[1] > MAX_PITCH_ANGLE ? MAX_PITCH_ANGLE : gimbalReferenceAngle[1];
        gimbalReferenceAngle[1] = gimbalReferenceAngle[1] < MIN_PITCH_ANGLE ? MIN_PITCH_ANGLE : gimbalReferenceAngle[1];

        // ロボットの目標速度を計算
        RobotReferenceSpeed[0] = 2000 * (float)operate.LX() / 660.0;
        RobotReferenceSpeed[1] = 2000 * (float)operate.LY() / 660.0;
        RobotReferenceSpeed[2] = 0.008 * gimbalAngle[0];

        float out[4] = {0.0, 0.0, 0.0, 0.0};  //                                      　　各タイヤが出すべき周速度を格納[mm/s]
        mecanum.calculate(RobotReferenceSpeed, out, -2 * M_PI * gimbalAngle[0] / 360); // 各タイヤが出すべき周速度を計算
        out[0] = -out[0];
        out[1] = out[1];
        out[2] = out[2];
        out[3] = -out[3];
        // max({out[0],out[1],out[2],out[3]});

        // out[0] = 0.0;
        // out[1] = 0.0;
        // out[2] = 0.0;
        // out[3] = 0.0;

        // if (controlCount % 20000 < 5000)
        // {
        //     out[3] = 2200.0;
        // }
        // else if (controlCount % 20000 < 10000)
        // {
        //     out[3] = 0.0;
        // }
        // else if (controlCount % 20000 < 15000)
        // {
        //     out[3] = -2200.0;
        // }
        // else
        // {
        //     out[3] = 0.0;
        // }

        // out[0] = 2500 * sin(4.5 * controlCount/1000.0 * 2.0 * M_PI);

        // タイヤの電流指令値を計算
        for (int i = 0; i < 4; i++)
        {
            float wheelPeripheralSpeed = wheelSpeed[i] * M_PI * WHEEL_DIAMETER; // タイヤの周速度[mm/s]
            float error = out[i] - wheelPeripheralSpeed;
            float integralError = pidWheelSpeed[i].CalculateIntegralError(error);
            if (integralError * pidWheelSpeed[i].GetI_gain() > 6.0 )
            {
                pidWheelSpeed[i].SetIntegralError(6.0 / pidWheelSpeed[i].GetI_gain());
            }
            if (integralError * pidWheelSpeed[i].GetI_gain() < -6.0)
            {
                pidWheelSpeed[i].SetIntegralError(-6.0 / pidWheelSpeed[i].GetI_gain());
            }
            chassisReferenceCurrent[i] = pidWheelSpeed[i].CalculateOutput(error, pidWheelSpeed[i].GetIntegralError(), pidWheelSpeed[i].CalculateDifferentialError(error));
            chassisReferenceCurrent[i] = min(max(chassisReferenceCurrent[i],-6.0),6.0);
            // chassisReferenceCurrent[0] = 1.0;
            // chassisReferenceCurrent[1] = 1.0;
            // chassisReferenceCurrent[2] = 1.0;
            // chassisReferenceCurrent[3] = 1.0;
        }

        // 弾送りの電流指令値を計算
        float fireError = fireReferenceSpeed - fireSpeed;
        float fireIntegralError = pidFireSpeed.CalculateIntegralError(fireError);
        if (fireIntegralError * pidFireSpeed.GetI_gain() > 15.0 )
        {
            pidFireSpeed.SetIntegralError(15.0 / pidFireSpeed.GetI_gain());
        }
        if (fireIntegralError * pidFireSpeed.GetI_gain() < -15.0)
        {
            pidFireSpeed.SetIntegralError(-15.0 / pidFireSpeed.GetI_gain());
        }
        fireReferenceCurrent = pidFireSpeed.CalculateOutput(fireError,pidFireSpeed.GetIntegralError(),pidFireSpeed.CalculateDifferentialError(fireError));
        fireReferenceCurrent = min(max(fireReferenceCurrent,-15.0),15.0);

        // ジンバルの電流指令値を計算
        gimbalReferenceCurrent[0] = -pidGimbalAngle[0].CalculateOutput(gimbalReferenceAngle[0] - gimbalImuAngle.getConvertedVal());
        gimbalReferenceCurrent[1] = -pidGimbalAngle[1].CalculateOutput(gimbalReferenceAngle[1] - gimbalAngle[1]);

        // 発射ローラーの電流指令値を計算
        rollerReferenceCurrent[0] = pidRollerSpeed[0].CalculateOutput(-ROLLER_RPS - rollerSpeed[0]);
        rollerReferenceCurrent[1] = pidRollerSpeed[1].CalculateOutput(ROLLER_RPS - rollerSpeed[1]);

        //Serial.printf(">n:%f\n>t:%f\n>o:%f\n", wheelSpeed[3] * M_PI * WHEEL_DIAMETER, out[3], chassisReferenceCurrent[3]);
        Serial.printf(">n:%f\n>t:%f\n>o:%f\n", fireSpeed, fireReferenceSpeed, fireReferenceCurrent);        

        break;
    }
    }
    if (digitalRead(13))
    {
        rollerReferenceCurrent[0] = 0.0;
        rollerReferenceCurrent[1] = 0.0;
        fireReferenceCurrent = 0.0;
    }

    // 足回りの指令値をCANメッセージに格納
    for (int i = 0; i < 4; i++)
    {
        int16_t referenceCurrentBin = min(16384, max(-16384, (chassisReferenceCurrent[i] / 20.0 * 16384))); // バイナリーに変換
        can1_0x200.buf[i * 2] = referenceCurrentBin >> 8;
        can1_0x200.buf[i * 2 + 1] = referenceCurrentBin & 0xFF;
    }

    // 弾送りの指令値をCANメッセージに格納
    int16_t fierReferenceCurrentBin = min(16384, max(-16384, (fireReferenceCurrent / 20.0 * 16384))); // バイナリーに変換
    can1_0x1ff.buf[4] = fierReferenceCurrentBin >> 8;
    can1_0x1ff.buf[5] = fierReferenceCurrentBin & 0xFF;

    // ヨーの指令値をCANメッセージに格納
    int16_t yawReferenceCurrentBin = min(25000, max(-25000, (gimbalReferenceCurrent[0] / 20.0 * 25000))); // バイナリーに変換
    can1_0x1ff.buf[0] = yawReferenceCurrentBin >> 8;
    can1_0x1ff.buf[1] = yawReferenceCurrentBin & 0xFF;

    // ピッチの指令値をCANメッセージに格納
    int16_t pitchReferenceCurrentBin = min(25000, max(-25000, (gimbalReferenceCurrent[1] / 20.0 * 25000))); // バイナリーに変換
    can2_0x1ff.buf[2] = pitchReferenceCurrentBin >> 8;
    can2_0x1ff.buf[3] = pitchReferenceCurrentBin & 0xFF;

    // ローラー右左の指令値をCANメッセージ格納
    int16_t rollerRightReferenceCurrentBin = min(16384, max(-16384, (rollerReferenceCurrent[0] / 20.0 * 16384))); // バイナリーに変換
    can2_0x1ff.buf[4] = rollerRightReferenceCurrentBin >> 8;
    can2_0x1ff.buf[5] = rollerRightReferenceCurrentBin & 0xFF;
    int16_t rollerLeftReferenceCurrentBin = min(16384, max(-16384, (rollerReferenceCurrent[1] / 20.0 * 16384))); // バイナリーに変換
    can2_0x1ff.buf[6] = rollerLeftReferenceCurrentBin >> 8;
    can2_0x1ff.buf[7] = rollerLeftReferenceCurrentBin & 0xFF;

    can1.write(can1_0x200);
    can1.write(can1_0x1ff);
    can2.write(can2_0x1ff);

    // Serial.printf("%f,%f,%f\r\n",fireSpeed,fireReferenceSpeed,fireReferenceCurrent);
    // Serial.printf("%f,%f,%f,%d\r\n", rollerSpeed[0], ROLLER_RPS, rollerReferenceCurrent[0], rollerRightReferenceCurrentBin);
    // Serial.printf("%f,%f,%f,%d\r\n", gimbalAngle[0], gimbalAngle[1], gimbalImuAngle.getConvertedVal(),escData[5].angle);
    // Serial.printf("%f,%f,%f,%f\r\n", wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    // Serial.printf("%f,%f,%f\r\n", RobotReferenceSpeed[0], RobotReferenceSpeed[1], RobotReferenceSpeed[2]);
    // Serial.printf("%f,%f,%f,%f\r\n", chassisReferenceCurrent[0], chassisReferenceCurrent[1], chassisReferenceCurrent[2], chassisReferenceCurrent[3]);
    // Serial.printf("%d,%d,%d,%d,%d,%d,%d\r\n", operate.RX(), operate.RY(), operate.LX(), operate.LY(), operate.RS(), operate.LS(), operate.SR());
    Serial.printf("%d\r\n",micros() - controlTime);
    while ((micros() - controlTime) < 1000)
    {
    }
    controlTime = micros();
}
