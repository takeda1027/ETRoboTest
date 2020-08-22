//
//  crew.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"

// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless;
// global variables to gyro sensor output from Observer to  Navigator and its sub-classes
int16_t g_angle, g_anglerVelocity;

Radioman::Radioman() {
    _debug(syslog(LOG_NOTICE, "%08u, Radioman constructor", clock->now()));
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
}

void Radioman::operate() {
    uint8_t c = fgetc(bt); /* 受信 */
    fputc(c, bt); /* エコーバック */
    switch(c)
    {
        case CMD_START_R:
        case CMD_START_r:
            syslog(LOG_NOTICE, "%08u, StartCMD R-mode received", clock->now());
            captain->decide(EVT_cmdStart_R);
            break;
        case CMD_START_L:
        case CMD_START_l:
            syslog(LOG_NOTICE, "%08u, StartCMD L-mode received", clock->now());
            captain->decide(EVT_cmdStart_L);
            break;
        case CMD_DANCE_D:
        case CMD_DANCE_d:
            syslog(LOG_NOTICE, "%08u, LimboDancer forced by command", clock->now());
            captain->decide(EVT_cmdDance);
            break;
        case CMD_CRIMB_C:
        case CMD_CRIMB_c:
            syslog(LOG_NOTICE, "%08u, SeesawCrimber forced by command", clock->now());
            captain->decide(EVT_cmdCrimb);
            break;
        case CMD_STOP_S:
        case CMD_STOP_s:
            syslog(LOG_NOTICE, "%08u, stop forced by command", clock->now());
            captain->decide(EVT_cmdStop);
            break;
        default:
            break;
    }
}

Radioman::~Radioman() {
    _debug(syslog(LOG_NOTICE, "%08u, Radioman destructor", clock->now()));
    fclose(bt);
}

Observer::Observer(Motor* lm, Motor* rm, Motor* am, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;
    touchSensor = ts;
    sonarSensor = ss;
    gyroSensor  = gs;
    colorSensor = cs;
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = 0;
    prevAngR = 0;
    notifyDistance = 0;
    traceCnt = 0;
    prevGS = INT16_MAX;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
    lost_flag = false;
    blue_flag = false;
    //ot_r = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_g = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_b = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

void Observer::goOnDuty() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
}

void Observer::reset() {
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
}

void Observer::notifyOfDistance(int32_t delta) {
    notifyDistance = delta + distance;
}

int32_t Observer::getDistance() {
    return (int32_t)distance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

void Observer::operate() {
    colorSensor->getRawColor(cur_rgb);
    // process RGB by the Low Pass Filter
    cur_rgb.r = fir_r->Execute(cur_rgb.r);
    cur_rgb.g = fir_g->Execute(cur_rgb.g);
    cur_rgb.b = fir_b->Execute(cur_rgb.b);
    rgb_to_hsv(cur_rgb, cur_hsv);
    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;
    // calculate gray scale and save them to the global area
    //g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 120 + cur_rgb.b * 29) / 226;
    //g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 120 + (cur_rgb.b - cur_rgb.g) * 29) / 226; // B - G cuts off blue
 
    //スラローム上では、茶色地面なので青、赤を逆転っぽく sano
    if(!slalom_flg){
        g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 120 + cur_rgb.b * 29) / 226;
        g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 120 + (cur_rgb.b - cur_rgb.g) * 29) / 226; // B - G cuts off blue

    }else{
        g_grayScale = ((cur_rgb.r-4) * 77 + (cur_rgb.g-1) * 120 + (cur_rgb.b + 17) * 29) / 226;
        g_grayScaleBlueless = ((cur_rgb.r-4) * 77 + (cur_rgb.g-1) * 120 +  (cur_rgb.b - cur_rgb.g + 16)  * 29) / 226; // B - G cuts off blue
    }
    
//   g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 120 + cur_rgb.b * 29) / 226;
//   g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 120 + (cur_rgb.b - cur_rgb.g) * 29) / 226; // B - G cuts off blue

    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    // accumulate distance
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    // calculate azimuth
    double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
    azimuth += deltaAzi;
    if (azimuth > M_2PI) {
        azimuth -= M_2PI;
    } else if (azimuth < 0.0) {
        azimuth += M_2PI;
    }
    // estimate location
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance) && !slalom_flg) { //sano なぜか直角ターン後、この処理が走るため停止回避
        syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        captain->decide(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
    //    syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        captain->decide(EVT_touch_On);
    } else if (!result && touch_flag) {
    //    syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        captain->decide(EVT_touch_Off);
    }
    
    // monitor sonar sensor
    result = check_sonar();
    if (result && !sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        captain->decide(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        captain->decide(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        captain->decide(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        captain->decide(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        result = check_lost();
        if (result && !lost_flag) {
            syslog(LOG_NOTICE, "%08u, line lost", clock->now());
            lost_flag = true;
            captain->decide(EVT_line_lost);
        } else if (!result && lost_flag) {
            syslog(LOG_NOTICE, "%08u, line found", clock->now());
            lost_flag = false;
            captain->decide(EVT_line_found);
        }

        // temporary dirty logic to detect the second black to blue change
        int32_t ma_gs;
        if (prevGS == INT16_MAX) {
            prevTime = clock->now();
            prevGS = g_grayScale;
            ma_gs = ma->add(0);
        } else {
            curTime = clock->now();
            gsDiff = g_grayScale - prevGS;
            timeDiff = curTime - prevTime;
            ma_gs = ma->add(gsDiff * 1000000 / timeDiff);
            prevTime = curTime;
            prevGS = g_grayScale;
        }
        int32_t x = getLocX();
// sano：開始
 //       syslog(LOG_NOTICE, "gs = %d, MA = %d, x = %d", g_grayScale, ma_gs, x);
        // if ( (ma_gs > 150) || (ma_gs < -150) ){
        //     //syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
        //     if ( !blue_flag && (ma_gs > 150) && ((x > 4300) || (
        //         x < -4300)) ) {
        //         blue_flag = true;
        //         syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
        //         captain->decide(EVT_bk2bl);
        //     }
        // }
 // sano:終了
        //         if ( (ma_gs > 150) || (ma_gs < -150) ){LOG_NOTICE, "gs = %d
        //     syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
        //     if ( !blue_flag && (ma_gs > 150) && ((x > 4300) || (x < -4300)) ) {
        //         blue_flag = true;
        //         syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
        //         captain->decide(EVT_bk2bl);
        //     }
        // }

        // determine if tilt
        if ( check_tilt() ) {
            //captain->decide(EVT_cmdStop);
        }
    }
    
    // display trace message in every PERIOD_TRACE_MSG ms */
    int32_t d = getDistance();
    
    //sano：開始
    //printf(",d=%d",d);
    //障害物との距離=>アーム上げのタイミング判定に利用
     int32_t dis = sonarSensor->getDistance();
    //printf(",dis=%d",dis);
 
    //ブルー１個目の判断 前方何もなし、ラインブルーの場合(スタート地点でなぜか処理に入ってしまうためcur_rgb.b <=255を追加)
    if( cur_rgb.b - cur_rgb.r > 60 && !b1 && dis > 250 && cur_rgb.b <=255 && cur_rgb.r<=255){
        b1 =true;
            //captain->decide(EVT_turnCnr); 
    }else if(b1 && cur_rgb.b - cur_rgb.r < 40){
        b1 = false; //１つめのブルー検知フラグを落とす
    }

    //b-r青判定、スラロームが近い（前方障害あり）、ブルー２個目b2フラグ立てる
    if(cur_rgb.b - cur_rgb.r > 60 && dis < 50  && cur_rgb.b <=255 && cur_rgb.r<=255){
        b2 =true;
        armMotor->setPWM(20);
        //}else if(b2==1 &&  cur_rgb.b - cur_rgb.r < 40){
       //    b2 =0;  //黒に戻ったらＢ２落とす
    }
    //スラローム判定（スラロームに近づくとdisが徐々に小さくなるが、のった後はdisが大きくなるためそこでフラグオン）
    if(b2 && !slalom_flg && !right_angle){
        b_dis=a_dis;
        a_dis=dis;
        if(a_dis>b_dis){
            slalom_flg=true;
            a_dis = 0;//初期化
            b_dis = 0;//初期化
        }
    }
    
    //スラローム専用処理（駆動をゼロにするだけで現状上がりっぱなし。。。）
    if(slalom_flg){
        //アーム停止
        armMotor->setPWM(0);
        //if(!obj_flg){
          //  b_dis=a_dis;
          //  a_dis=dis;
          //  if(a_dis<b_dis){
            //    captain->decide(EVT_bk2bl); // 止めてアーム下ろす // sano;
          //      obj_flg = true;
          //  }
            //printf("a_dis=%d,b_dis=%d,dis=%d,\n",a_dis,b_dis,dis);
        //}

        //printf(",r+g+b=%d,r=%d,g=%d,b=%d,right_angle=%d\n",cur_rgb.r + cur_rgb.g + cur_rgb.b,cur_rgb.r,cur_rgb.g,cur_rgb.b,right_angle);
    
        // 左下の直角カーブ対応
        if(cur_rgb.r + cur_rgb.g + cur_rgb.b <= 100 && !right_angle){
            right_angle=true;
            //captain->decide(EVT_bk2bl); // sano使えるぞこれは
            captain->decide(EVT_turnCnr); // ここで直角ターン
        }
       //車体の傾きでスラローム終了を検知
        printf("g_angle=%d,slalom_flg=%d\n",g_angle,slalom_flg);
         if(g_angle > 10){
            slalom_flg = false;
        }
 
    }

    //sano　青判定３回目
    if( cur_rgb.b - cur_rgb.r > 60 && right_angle && !b3){
        printf(",b-r=%d,r=%d,g=%d,b=%d,right_angle=%d\n", cur_rgb.b - cur_rgb.r,cur_rgb.r,cur_rgb.g,cur_rgb.b,right_angle);
        b3 = true;
        captain->decide(EVT_turnb3); // ソナー稼働回転、物体を見つけに行く
    }



    //printf(",b-r=%d,r=%03u, g=%03u, b=%03u, b1=%d,b2=%d,slalom_flg=%d\n", g_rgb.b-g_rgb.r,g_rgb.r, g_rgb.g, g_rgb.b,b1,b2,slalom_flg);
    //sano：終了

    if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
    //if ((++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) && (d < 11000)) {
        traceCnt = 0;
     //   _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), dis, getAzimuth(), getLocX(), getLocY()));
     //   _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), d, getAzimuth(), getLocX(), getLocY()));
     //   _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
     //   _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
     //   _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
    //} else if (d >= 11000) {
    //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), d, getAzimuth(), getLocX(), getLocY()));
    //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
    //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
    }
}

void Observer::goOffDuty() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
   // _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

bool Observer::check_touch(void) {
//    if (touchSensor->isPressed()) {
    if (touchSensor->isPressed() && !b2 ){  //sano_t
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
    /*
    int8_t otRes_r, otRes_g, otRes_b;
    otRes_r = ot_r->test(cur_rgb.r);
    otRes_g = ot_g->test(cur_rgb.g);
    otRes_b = ot_b->test(cur_rgb.b);
    if ((otRes_r == POS_OUTLIER && otRes_g == POS_OUTLIER) ||
        (otRes_g == POS_OUTLIER && otRes_b == POS_OUTLIER) ||
        (otRes_b == POS_OUTLIER && otRes_r == POS_OUTLIER)) {
        return true;
    } else {
        return false;
    }
    */
}

bool Observer::check_tilt(void) {
    int16_t anglerVelocity = gyroSensor->getAnglerVelocity();
    if (anglerVelocity < ANG_V_TILT && anglerVelocity > (-1) * ANG_V_TILT) {
        return false;
    } else {
       // _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): TILT anglerVelocity = %d", clock->now(), anglerVelocity));
        return true;
    }
}

void Observer::freeze() {
    frozen = true;
}

void Observer::unfreeze() {
    frozen = false;
}


Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}

Navigator::Navigator() {
    _debug(syslog(LOG_NOTICE, "%08u, Navigator default constructor", clock->now()));
    ltPid = new PIDcalculator(P_CONST, I_CONST, D_CONST, PERIOD_NAV_TSK, -16, 16); 
}

void Navigator::goOnDuty() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_NAV_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_NAV_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Navigator handler set", clock->now()));
}

void Navigator::goOffDuty() {
    activeNavigator = NULL;
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_NAV_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_NAV_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Navigator handler unset", clock->now()));
}

Navigator::~Navigator() {
    _debug(syslog(LOG_NOTICE, "%08u, Navigator destructor", clock->now()));
}

AnchorWatch::AnchorWatch(Motor* tm) {
    _debug(syslog(LOG_NOTICE, "%08u, AnchorWatch constructor", clock->now()));
}

void AnchorWatch::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, AnchorWatch has control", clock->now());
}

void AnchorWatch::operate() {
    //controlTail(TAIL_ANGLE_STAND_UP,10); /* 完全停止用角度に制御 */
}

AnchorWatch::~AnchorWatch() {
    _debug(syslog(LOG_NOTICE, "%08u, AnchorWatch destructor", clock->now()));
}

LineTracer::LineTracer(Motor* lm, Motor* rm, Motor* tm) {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    trace_pwmLR = 0;
    speed       = SPEED_NORM;
    frozen      = false;
}

void LineTracer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, LineTracer has control", clock->now());
}

void LineTracer::operate() {
    //controlTail(TAIL_ANGLE_DRIVE,10); /* バランス走行用角度に制御 */

    if(turn_flg){//直角ターン命令のため。駆動力を自由に設定可 sano

        pwm_L = pwm_p_L;
        pwm_R = pwm_p_R;
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
        //printf("turn_flg経由,");

        
    }else{  //sano

        if (frozen) {
            forward = turn = 0; /* 障害物を検知したら停止 */
           // printf("frozen経由,");

        } else{
            forward = speed; //前進命令
            /*
            // on-off control
            if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2) {
                turn =  20; // 左旋回命令
            } else {
                turn = -20; // 右旋回命令
            }
            */
            /*
            // PID control by brightness
            int16_t sensor = colorSensor->getBrightness();
            int16_t target = (LIGHT_WHITE + LIGHT_BLACK)/2;
            */
            // PID control by Gray Scale with blue cut
            int16_t sensor = g_grayScaleBlueless;
            int16_t target = GS_TARGET;

            if (state == ST_tracing_L || state == ST_stopping_L || state == ST_crimbing) {
                turn = ltPid->compute(sensor, target);
                
            } else {
                // state == ST_tracing_R || state == ST_stopping_R || state == ST_dancing
                turn = (-1) * ltPid->compute(sensor, target);
            }
            //printf("その他経由,");

        }

        /* 左右モータでロボットのステアリング操作を行う */
        pwm_L = forward - turn;
        pwm_R = forward + turn;

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

    } //sano

    //printf(",pwm_L = %d, pwm_R = %d,turn=%d,", pwm_L, pwm_R,turn); //sano

    // display pwm in every PERIOD_TRACE_MSG ms */
    if (++trace_pwmLR * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
        trace_pwmLR = 0;
        //_debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): pwm_L = %d, pwm_R = %d", clock->now(), pwm_L, pwm_R));
        /*
        _debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
        */
    }
}

void LineTracer::setSpeed(int8_t s) {
    speed = s;
}

void LineTracer::freeze() {
    frozen = true;
}

void LineTracer::unfreeze() {
    frozen = false;
}

//　直角ターン用　左右の車輪に駆動にそれぞれ値を指定する sano
void LineTracer::turnC(bool t,int p_L,int p_R) { //sano
    turn_flg = t;
    pwm_p_L = p_L;
    pwm_p_R = p_R;
}


LineTracer::~LineTracer() {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer destructor", clock->now()));
}

Captain::Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain default constructor", clock->now()));
}

void Captain::takeoff() {
    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor   = new Motor(PORT_A); //sano
    steering    = new Steering(*leftMotor, *rightMotor);
    
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET aflac2020", 0, CALIB_FONT_HEIGHT*1);
    
    observer = new Observer(leftMotor, rightMotor, armMotor, touchSensor, sonarSensor, gyroSensor, colorSensor);
    observer->freeze(); // Do NOT attempt to collect sensor data until unfreeze() is invoked
    observer->goOnDuty();
    limboDancer = new LimboDancer(leftMotor, rightMotor, tailMotor);
    seesawCrimber = new SeesawCrimber(leftMotor, rightMotor, tailMotor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor);
    
    /* 尻尾モーターのリセット */
    //tailMotor->reset();
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    state = ST_takingOff;
    anchorWatch = new AnchorWatch(tailMotor);
    anchorWatch->goOnDuty();
    anchorWatch->haveControl();

    act_tsk(RADIO_TASK);
}

void Captain::decide(uint8_t event) {
    syslog(LOG_NOTICE, "%08u, Captain::decide(): event %s received by state %s", clock->now(), eventName[event], stateName[state]);
    switch (state) {
        case ST_takingOff:
            switch (event) {
                case EVT_cmdStart_R:
                case EVT_cmdStart_L:
                case EVT_touch_On:
                    if (event == EVT_cmdStart_L || (event == EVT_touch_On && _LEFT)) {
                        state = ST_tracing_L;
                    } else {  // event == EVT_cmdStart_R || (event == EVT_touch_On && !_LEFT)
                        state = ST_tracing_R;
                    }
                    syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                    
                    /* 走行モーターエンコーダーリセット */
                    leftMotor->reset();
                    rightMotor->reset();
                    
                    //balance_init(); /* 倒立振子API初期化 */
                    observer->reset();
                    
                    /* ジャイロセンサーリセット */
                    gyroSensor->reset();
                    ev3_led_set_color(LED_GREEN); /* スタート通知 */
                    
                    observer->freeze();
                    lineTracer->freeze();
                    lineTracer->haveControl();
                    //clock->sleep() seems to be still taking milisec parm
                    //clock->sleep(PERIOD_NAV_TSK*FIR_ORDER/1000); // wait until FIR array is filled
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    syslog(LOG_NOTICE, "%08u, Departed", clock->now());
                   break;
                default:
                    break;
            }
            break;
        case ST_tracing_R:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_sonar_On:
		    //lineTracer->freeze();
		    // During line trancing,
		    // if sonar is on (limbo sign is near by matchine),
		    // limbo dance starts.
                    //state = ST_dancing;
                    //limboDancer->haveControl();
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_cmdDance:
                case EVT_bl2bk:
                    //state = ST_dancing;
                    //limboDancer->haveControl();
                    break;
                case EVT_bk2bl:
                // sanoコメントアウト
                    // observer->freeze();
                    // lineTracer->freeze();
                    // //lineTracer->setSpeed(Motor::PWM_MAX);
                    // //clock->sleep() seems to be still taking milisec parm
                    // clock->sleep(5000); // wait a little
                    // lineTracer->unfreeze();
                    // observer->unfreeze();
                    break;
                case EVT_cmdStop:
                    state = ST_stopping_R;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_tracing_L:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_sonar_On:
                    //lineTracer->freeze();
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_cmdCrimb:
                case EVT_bl2bk:
                    //state = ST_crimbing;
                    //seesawCrimber->haveControl();
                    break;
                case EVT_bk2bl:
                // sano
                    // observer->freeze();
                    lineTracer->freeze();
                    // //lineTracer->setSpeed(Motor::PWM_MAX);
                    // //clock->sleep() seems to be still taking milisec parm
                     armMotor->setPWM(-100); // アーム降ろす
                     clock->sleep(5000); // wait a little
                     lineTracer->unfreeze();
                    // observer->unfreeze();
                    break;
                case EVT_turnCnr:
                // sano
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(true,100,0);
                    printf("ターンしています\n");
                    clock->sleep(735); // wait a little
                    printf("スリープ終了した\n");
                    lineTracer->turnC(false,0,0);
                    printf("ターンを０に\n");
                    lineTracer->freeze();
                    printf("回転後の停止\n");
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    printf("動き出します\n");
                    break;
                case EVT_turnb3:
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(true,20,0);
                    clock->sleep(1000); // wait a little
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(true,10,0);
                    // b3_aa = 0; //念のため角度累積を初期化
                    // b_av = 0; //念のため前角速度を初期化
                    // n_av = 0; //念のため前角速度を初期化
                    //ソナーを回転しを見つける
                    for (int i = 0; i < 300000; i++){
                        printf("dis_obj=%d,",sonarSensor->getDistance());
                        // if(sonarSensor->getDistance() > 180 && sonarSensor->getDistance() <250){
                        //     printf("物体を見つけた\n");
                        //     //ソナーセンサーの放射角度20度を補正
                        //     while(b3_aa < 20){
                                 n_av = g_anglerVelocity;
                                 now_tim = clock->now();
                                 b3_aa += calc_angle(n_av,b_av,now_tim - before_tim);
                                 //printf("b3_aa=%lf,n_av=%d,now_tim=%d,",b3_aa,n_av,now_tim);
                                 b_av = n_av;
                                 before_tim = now_tim;
                                //printf("b_av=%d,g_anglerVelocity=%d,before_tim=%d\n",b_av,g_anglerVelocity,before_tim);
                                clock->sleep(10); // wait a little
                        //     }
                        //     lineTracer->freeze();
                        //     clock->sleep(500); // wait a little
                        //     lineTracer->unfreeze();
                        //     break;
                        // }
                    }
                    lineTracer->turnC(true,30,30);
                    clock->sleep(10000); // wait a little
                    break;
                case EVT_cmdStop:
                    state = ST_stopping_L;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_dancing:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_bk2bl:
		    // Don't use "black line to blue line" event.
  		    /*
                    state = ST_stopping_R;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
		    */
                    break;
                default:
                    break;
            }
            break;
        case ST_crimbing:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_bk2bl:
                    state = ST_stopping_L;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_stopping_R:
        case ST_stopping_L:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_dist_reached:
                    state = ST_landing;
                    anchorWatch->haveControl(); // does robot stand still?
                    triggerLanding();
                    break;
                default:
                    break;
            }
            break;
        case ST_landing:
            break;
        default:
            break;
    }
}

void Captain::triggerLanding() {
    syslog(LOG_NOTICE, "%08u, Landing...", clock->now());
    ER ercd = wup_tsk(MAIN_TASK); // wake up the main task
    assert(ercd == E_OK);
}

void Captain::land() {
    ter_tsk(RADIO_TASK);

    if (activeNavigator != NULL) {
        activeNavigator->goOffDuty();
    }
    leftMotor->reset();
    rightMotor->reset();
    
    delete anchorWatch;
    delete lineTracer;
    delete seesawCrimber;
    delete limboDancer;
    observer->goOffDuty();
    delete observer;
    
    delete tailMotor;
    delete armMotor; //sano
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
}

Captain::~Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain destructor", clock->now()));
}
