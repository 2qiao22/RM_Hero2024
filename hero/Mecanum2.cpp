#include "Mecanum2.hpp"

/**
 ****************************************************************************************************************************************************
 * @brief これは、Mecanum2クラスのコンストラクタ。
 * メカナムホイール間の横幅と縦幅から逆運動学モデルを作成
 * @param wheelWidthDistance メカナムホイール間の横幅
 * @param wheelVerticalDistance メカナムホイール間の縦幅
 ****************************************************************************************************************************************************
 */
Mecanum2::Mecanum2(float wheelWidthDistance, float wheelVerticalDistance)
{
    this->inverseKinematicsMatrix[0][2] = wheelWidthDistance / 2.0 + wheelVerticalDistance / 2.0;
    this->inverseKinematicsMatrix[1][2] = -(wheelWidthDistance / 2.0 + wheelVerticalDistance / 2.0);
    this->inverseKinematicsMatrix[2][2] = -(wheelWidthDistance / 2.0 + wheelVerticalDistance / 2.0);
    this->inverseKinematicsMatrix[3][2] = wheelWidthDistance / 2.0 + wheelVerticalDistance / 2.0;
};

/**
 ****************************************************************************************************************************************************
 *  @fn void calculate
 *  @brief フィールド座標系での速度ベクトルを各メカナムホイールの周速度に変換
 *  @param velocityVector[3] 変換するロボットの速度ベクトル
 *  @param _out[4] 出力された各メカナムホイールの周速度を格納する配列
 *  @param currentAngle 現在のロボットのZ軸の角度 省略時はロボット座標系と等価
 ****************************************************************************************************************************************************
 */
void Mecanum2::calculate(float velocityVector[3], float _out[4], float currentAngle)
{
    // フィールド座標系での速度ベクトルをロボット座標系に回転行列を用いて変換
    float velocityVectorOnRobot[3];
    velocityVectorOnRobot[0] = velocityVector[0] * cos(-currentAngle) - velocityVector[1] * sin(-currentAngle);
    velocityVectorOnRobot[1] = velocityVector[0] * sin(-currentAngle) + velocityVector[1] * cos(-currentAngle);
    velocityVectorOnRobot[2] = velocityVector[2];

    for (size_t motorNum = 0; motorNum < 4; motorNum++)
    {
        _out[motorNum] = 0.0;
        for (size_t element = 0; element < 3; element++)
        {
            _out[motorNum] += this->inverseKinematicsMatrix[motorNum][element] * velocityVectorOnRobot[element];
        }
    }
};
