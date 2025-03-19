const int POWER_FLAG = 50;

const float WHEEL_DIAMETER = 100.0;//タイヤの直径[mm]

const float WHEEL_WIDTH_DISTANCE = 600;
const float WHEEL_VERTICAL_DISTANCE = 700;
const float WHEEL_MAX_PERIPHERAL_SPEED = 1000;//タイヤの最大周速度[mm/s]

const int MAX_PITCH_RAW = 3275;//仰角が一番上の時の生データ
const int MIN_PITCH_RAW = 6755;//仰角が一番下の時の生データ
const int CENTER_PITCH_RAW = 4950;//仰角が平行の時の生データ
const float PITCH_ANGLE_RANGE = 15.0;//仰角の稼動角度[deg.]

const float MAX_PITCH_ANGLE = (float)(MAX_PITCH_RAW - CENTER_PITCH_RAW) / (float)(MAX_PITCH_RAW - MIN_PITCH_RAW) * 15.0;//ピッチが一番上を向いた時の角度[deg.]
const float MIN_PITCH_ANGLE = (float)(MIN_PITCH_RAW - CENTER_PITCH_RAW) / (float)(MAX_PITCH_RAW - MIN_PITCH_RAW) * 15.0;//ピッチが一番下を向いた時の角度[deg.]


const float ROLLER_RPS = 160.0;

const float M3508_GEAR_RATIO = 3591.0 / 187.0;  //M3508P19のギヤヘッドのギヤ比

