#include "Motor.h"
#include "ColorSensor.h"
#include "util.h"
#include "GyroSensor.h" //朋子追加
#include "SonarSensor.h" //朋子追加
#include "Clock.h"  // 朋子追加
// #include "PrmReq.h" //朋子追加

using namespace ev3api;

class Tracer {
public:
  Tracer();
  void run();
  void init();
  void terminate();
  //朋子変数
  int hoge=0;
  int pwm_slow = 10;
  
private:
  Motor leftWheel;
  Motor rightWheel;
  ColorSensor colorSensor;
  GyroSensor gyro_a; //朋子追加
  SonarSensor sonar_a; //朋子追加
  Motor sippoWheel; //朋子追加 尻尾用
  Motor armWheel; //朋子追加　アーム用
  uint16_t r_c; //朋子追加　色研究
  uint16_t g_c; //朋子追加　色研究
  uint16_t b_c; //朋子追加　色研究
  int32_t b_m_r;  //朋子追加　色研究
  int8_t light; //朋子追加　色研究
  int b1 = 0; //朋子追加　１つめのブルー検知
  int b2 = 0; //朋子追加　２つめのブルー検知
  int b3_0 = 0; //朋子追加　３つめのブルー検知の準備
  int b3 = 0; //朋子追加　３つめのブルー検知
  bool slalom_flg = false;
  int a_dis=99;
  int b_dis=99;

//  PrmReq prmadd; //朋子追加
  const int8_t mThreshold = 20;
//  const int8_t pwm = (Motor::PWM_MAX) / 3;
  const int8_t pwm = 52;
  float judge_c =0;

  float calc_prop_value(float Kp, int target,int bias);      // <1>
  void turn_r(Motor r_leftWheel,Motor l_leftWheel,Clock c_clock);
};
