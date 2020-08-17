#include "Tracer.h"
#include "Clock.h"  // 朋子追加
//#include "PrmReq.h" //朋子追加

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B),
  colorSensor(PORT_3), gyro_a(PORT_2),sonar_a(PORT_4),sippoWheel(PORT_D),armWheel(PORT_A){
}

void Tracer::init() {
  init_f("Tracer");
  judge_c = 0;
  slalom_flg = false;
}

void Tracer::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();
  rightWheel.stop();
}

float Tracer::calc_prop_value(float Kp, int target,int bias) {
//  const float Kp = 0.83;        // <1>
//  const int target = 10;        // <2>
  int diff = colorSensor.getBrightness() - target; // <3>
  return (Kp * diff + bias);                       // <4>
}

void Tracer::turn_r(Motor r_leftWheel,Motor l_leftWheel,Clock c_clock) {
    r_leftWheel.setPWM(0);
    l_leftWheel.setPWM(50);
    c_clock.sleep(910); 
}

void Tracer::run() {
  msg_f("running...", 1);
 
  Clock clock;    // 朋子追加

  
  light = colorSensor.getBrightness();
  colorid_t color_no = colorSensor.getColorNumber();
  rgb_raw_t rgb; //朋子追加　色研究

  colorSensor.getRawColor(rgb);
  r_c=rgb.r;
  g_c=rgb.g;
  b_c=rgb.b;
  b_m_r=b_c-r_c;
  int pwm_s=0;
  int s_dis = sonar_a.getDistance();
  printf(",b-r=%d,r=%d,g=%d,b=%d,color_no=%d,light=%d,s_dis=%d,",b_m_r,r_c,g_c,b_c,color_no,light,s_dis);

  //１つめのブルー検出  青-赤＞60、ソナーセンサーに検知なし
  if(b_m_r > 60 && s_dis > 250 && b1 == 0){
     b1 = 1; //１つ目のブルー検知フラグを立てる
  }else if(b1 ==1 && b_m_r < 40){
     b1 = 0; //１つめのブルー検知フラグを落とす
  
  //２つめのブルー検知  青-赤＞60、ソナーセンサー40?
  }else if(b_m_r > 60 &&  s_dis <40 && s_dis >= 10 && b3_0==0){
     b2 = 1;
     b3_0=1;//直角ターンで立てる方が良いかな？
     gyro_a.reset();

  }else if(b2 ==1 && b_m_r < 40){
     //b2 = 0;
  //３つめのブルー検知
  }
  if(b_m_r > 60 &&  s_dis >=40 && s_dis < 60){
     b3 = 1;
  }
  
  //スラロームに上ったことを把握する 
  //急に目の前の障害物が遠くなったらスラロームＯＮ
  if(b2==1){
    b_dis = a_dis;
    a_dis = s_dis;
  
    if(a_dis > b_dis){
      slalom_flg = true;  
    }
  }

  printf("a_dis=%d,b_dis=%d,b1=%d,b2=%d,b3=%d,",a_dis,b_dis,b1,b2,b3);

  if(b1==1){
    //armWheel.setPWM(100); //手を上げる
  }

  if(b2==1){
    //1000*1000/(4*1000)=250 本タスクは4msごとに繰り返す。１秒に250回繰り返しているため、250で商算
    judge_c = judge_c + (gyro_a.getAnglerVelocity()/250);
  }

  if(b3==1){
    //armWheel.setPWM(-100); //手を降ろす
  }

  //段差をソナーセンサーで検知、駆動力上げ、ジャンプorアーム上げで突破  
  if(s_dis> 0 && s_dis < 12 && b2 ==1 &&  !slalom_flg){

    //尻尾上げる
    pwm_s=70;

    //腕上げる
    armWheel.setPWM(100);
  }

  //朋子変更
  float turn=0;
  int pwm_l=0;
  int pwm_r=0;
 
  
  //通常モード
  if (!slalom_flg){
    float Kp_u = 1.0;        // <1> const int8_t pwm = 2*(Motor::PWM_MAX) / 3;
    int target_u = 20;        // <2> const int8_t pwm = 2*(Motor::PWM_MAX) / 3;
    int bias_u = 0;
    turn = calc_prop_value(Kp_u,target_u,bias_u); // <1>
    pwm_l = pwm - turn;      // <2>
    pwm_r = pwm + turn;      // <2>

  //スラロームに登ったら、それぞれ処理
  } else if(slalom_flg){
    turn = calc_prop_value(0.83,15,5); // <1>
    pwm_l = 40 - turn;      // <2>
    pwm_r = 40 + turn;      // <2>
    //腕下げる
    armWheel.setPWM(-20);
    //尻尾しまう
    pwm_s=-100;
    b2=0;

    if(s_dis < 13){
      printf("回転開始\n");
      pwm_s=0;
      pwm_l = 30- turn;
      pwm_r = 30+ turn;
    }

  }
  //printf("駆動力ＵＰ開始、ジャイロは%d\n",judge_c);

  //turn_r(rightWheel,leftWheel,clock);

  //回転後、とりあえず直進
  //rightWheel.setPWM(55);
  //leftWheel.setPWM(55);
  //clock.sleep(7500);
  
  
  printf("pwm_l=%d, pwm_r=%d, turn=%5.2f, slalom_flg=%d\n", pwm_l,pwm_r,turn,slalom_flg);
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
  sippoWheel.setPWM(pwm_s);
}

