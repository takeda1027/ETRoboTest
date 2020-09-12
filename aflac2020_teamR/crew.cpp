//
//  crew.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//hugehuge

#include "app.h"
#include "crew.hpp"


// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless, challenge_stepNo;
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
        default:
            break;
    }
}

Radioman::~Radioman() {
    _debug(syslog(LOG_NOTICE, "%08u, Radioman destructor", clock->now()));
    fclose(bt);
}

Observer::Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;
    tailMotor = tm;
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
    lost_flag = false;
    line_over_flg = false; // sano
    adjust_flg = false; // sano

    curAngle = 0; //sano
    prevAngle = 0; //sano
    challenge_stepNo = 0; // sano

    blue2 = true; // sano テスト用
    slalom_flg = false; // sano
    moveBack_flg= false; // sano
    gyroSensor->setOffset(0);//sano

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();

    process_count = 1; // sano
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

int32_t Observer::getSonarDistance() {
    return (int32_t)sonarDistance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int16_t Observer::getDegree() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    if (degree > 180){
        degree -= 360;
    }
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
    curRgbSum = cur_rgb.r + cur_rgb.g + cur_rgb.b;
    rgb_to_hsv(cur_rgb, cur_hsv);

    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;

    // calculate gray scale and save them to the global area
    //スラローム上では、茶色地面なので青、赤を逆転っぽく sano
    if(!slalom_flg){
        g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 120 + cur_rgb.b * 29) / 226;
        g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 120 + (cur_rgb.b - cur_rgb.g) * 29) / 226; // B - G cuts off blue
    }else{
        g_grayScale = ((cur_rgb.r-4) * 77 + (cur_rgb.g-1) * 120 + (cur_rgb.b + 17) * 29) / 226;
        g_grayScaleBlueless = ((cur_rgb.r-4) * 77 + (cur_rgb.g-1) * 120 +  (cur_rgb.b - cur_rgb.g + 16)  * 29) / 226; // B - G cuts off blue
    }
    
    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    //printf("g_angle=%d, g_anglerVelocity=%d\n", g_angle, g_anglerVelocity);

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

    //sano テスト
    if(touchSensor->isPressed() && blue2){
        //azimuth =0;
        float hoge = 360.0 * azimuth/ M_2PI;
        if(hoge < 180){
            hoge =  hoge * 22 /30;
        }else{
            hoge = (hoge - 360) * 22 /30 + 360;
        }
        azimuth = hoge * M_2PI / 360.0;
    }
    float r = 0.0;
    r = 360.0 * azimuth/ M_2PI - 13 ;
    if(r < 0){
        r+=360;
    }
    //printf("deg=%lf,touchSensor=%d\n",r,touchSensor->isPressed());

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance) && !slalom_flg) {
        syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        captain->decide(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        captain->decide(EVT_touch_On);
    } else if (!result && touch_flag) { // sanoテスト用
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        captain->decide(EVT_touch_Off);
     }
    
    // monitor sonar sensor
    // sanoコメントアウト
    result = check_sonar();
    // if (result && !sonar_flag) {
    //     syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
    //     sonar_flag = true;
    //     //captain->decide(EVT_sonar_On);
    // } else if (!result && sonar_flag) {
    //     syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
    //     sonar_flag = false;
    //     //captain->decide(EVT_sonar_Off);
    // }

    // if (!frozen) { // these checks are meaningless thus bypassed when frozen
    //     // determine if still tracing the line
    //     result = check_lost();
    //     if (result && !lost_flag) {
    //         syslog(LOG_NOTICE, "%08u, line lost", clock->now());
    //         lost_flag = true;
    //         captain->decide(EVT_line_lost);
    //     } else if (!result && lost_flag) {
    //         syslog(LOG_NOTICE, "%08u, line found", clock->now());
    //         lost_flag = false;
    //         captain->decide(EVT_line_found);
    //     }

    // }
    
    // sano
    curDegree = getDegree();
    sonarDistance = sonarSensor->getDistance();

    //b-r青判定、スラロームが近い（前方障害あり）、ブルー２個目b2フラグ立てる
    if(cur_rgb.b - cur_rgb.r > 60 && sonarDistance < 50  && cur_rgb.b <=255 && cur_rgb.r<=255){
        blue2 =true;
    }

    //スラローム判定
    if(blue2 && !slalom_flg){
 
        if (sonarDistance >= 1 && sonarDistance <= 10 && !moveBack_flg){
            captain->decide(EVT_slalom_reached);
            armMotor->setPWM(30);
            moveBack_flg = true;
        }

        curAngle = g_angle;
        if(curAngle < -9){
            prevAngle = curAngle;
        }
        if (prevAngle < -9 && curAngle >= 0){
            printf("スラロームオン\n");
            slalom_flg = true;
            curAngle = 0;//初期化
            prevAngle = 0;//初期化
            distance = 0;//初期化
            armMotor->setPWM(-100);
        }
    }
    
    //スラローム専用処理
    if(slalom_flg){

        // if (challenge_stepNo >= 9 && challenge_stepNo <= 14){
        //      if (++traceCnt && traceCnt > 1) {
        //          printf(",curDegree=%d, sonarDistance=%d, challenge_stepNo=%d,r+g+b=%d\n",curDegree, sonarDistance,challenge_stepNo,curRgbSum);
        //          traceCnt = 0;
        //      }
        // }
        
        //スラロームオン後、１つ目の障害物を見つける
        if (challenge_stepNo == 0 && distance > 150){
            printf(",スラロームオン後、１つ目の障害物を見つける\n");
            captain->decide(EVT_obstcl_reached);
         // １つ目の障害物に接近する
        }else if(challenge_stepNo == 1 && check_sonar(40,255)){
            printf(",１つ目の障害物を回避する\n");
            captain->decide(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (challenge_stepNo == 2 && !line_over_flg){
             if (curRgbSum < 120) {
                 prevRgbSum = curRgbSum;
             }
             if(prevRgbSum < 120 && curRgbSum > 200){
                 printf(",黒ラインを超えるまで前進し、超えたら向きを調整する\n");
                 line_over_flg = true;
                 prevRgbSum = 0;
                 captain->decide(EVT_obstcl_angle);
             }
             prevDegree = curDegree;
        // ２つ目の障害物に向かって前進する
        }else if (challenge_stepNo == 3 && check_sonar(25,35) && own_abs(curDegree - prevDegree) > 60){
            printf(",２つ目の障害物に向かって前進する\n");
            captain->decide(EVT_obstcl_infront);
        // ２つ目の障害物に接近したら向きを変える
        }else if (challenge_stepNo == 4 && check_sonar(0,5)){
            printf(",２つ目の障害物に接近したら向きを変える\n");
            captain->decide(EVT_obstcl_reached);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 視界が晴れたら左上に前進する
        }else if(challenge_stepNo == 5 && check_sonar(20,255)){
            printf(",視界が晴れたところで前進する");
            captain->decide(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整し３つ目の障害物に接近する
        }else if (challenge_stepNo == 6 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 150){
                printf(",黒ラインを超えたら向きを調整し障害物に接近する\n");
                captain->decide(EVT_obstcl_angle);
                line_over_flg = true;
            }
        // ３つ目の障害物に接近したら後退して調整する
        }else if (challenge_stepNo == 7 && check_sonar(0,5)){
                printf(",３つ目の障害物に接近したら後退して調整する\n");
                captain->decide(EVT_obstcl_reached);
                line_over_flg = true;
        // 視界が晴れたら左下に前進する
        }else if (challenge_stepNo == 8 && check_sonar(20,255)){
                printf(",視界が晴れたら左下に前進する\n");
                captain->decide(EVT_obstcl_avoidable);
                prevRgbSum = curRgbSum;
                line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (challenge_stepNo == 9 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 195){
                printf(",黒ラインを超えたら向きを調整する\n");
                captain->decide(EVT_obstcl_angle);
                line_over_flg = true;
            }
            prevDegree = curDegree;
        // ４つ目の障害物に接近する
        }else if(challenge_stepNo == 10 && check_sonar(0,30) && own_abs(curDegree - prevDegree) > 60){
            printf(",４つ目の障害物に接近する\n");
            captain->decide(EVT_obstcl_infront);
        // ４つ目の障害物に接近したら向きを変える
        }else if (challenge_stepNo == 11 && check_sonar(0,5)){
            printf(",４つ目の障害物に接近したら向きを変える\n");
            captain->decide(EVT_obstcl_reached);
            prevDegree = curDegree;
        // 視界が晴れたら左上に前進する
        }else if (challenge_stepNo == 12 && check_sonar(21,255) && own_abs(curDegree - prevDegree) > 73){
            printf(",視界が晴れたら左上に前進する\n");
            captain->decide(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する
        }else if (challenge_stepNo == 13){
            if (!line_over_flg){
                if (curRgbSum < 100) {
                    prevRgbSum = curRgbSum;
                }
                if(prevRgbSum < 100 && curRgbSum > 150){
                    line_over_flg = true;
                }
            }else if (curRgbSum < 60 && line_over_flg) {
                printf(",黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する\n");
                captain->decide(EVT_obstcl_angle);
            }
            prevDegree = curDegree;
        // 直進しスラロームを降りる
        }else if(challenge_stepNo == 14 && own_abs(curDegree - prevDegree) > 70 ){
            printf(",直進しスラロームを降りる\n");
            captain->decide(EVT_obstcl_avoidable);
        }

        //printf("g_angle=%d\n");
         if(g_angle > 10 && challenge_stepNo > 2){
            printf("スラロームオフ\n");
            slalom_flg = false;
        }
    }

    // display trace message in every PERIOD_TRACE_MSG ms */
    //if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
        // if (d < 11000) {
        //     traceCnt = 0;
        //     //_debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), getDistance(), getAzimuth(), getLocX(), getLocY()));
        // //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), d, getAzimuth(), getLocX(), getLocY()));
        // //  _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        // //  _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        // //  _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
        // } else if (d >= 11000) {
        // //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), d, getAzimuth(), getLocX(), getLocY()));
        // //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        // //    _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        // }
    //}
}

void Observer::goOffDuty() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    //_debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

// bool Observer::check_touch(void) {
//     if (touchSensor->isPressed() && !b2 ){  //sano_t
//         return true;
//     } else {
//         return false;
//     }
// }

bool Observer::check_touch(void) {
    if (touchSensor->isPressed()){  //sano_t
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if (distance <= SONAR_ALERT_DISTANCE && distance >= 0) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to) {
    int32_t distance = sonarSensor->getDistance();
    //printf(",distance2=%d, sonar_alert_dist_from=%d, sonar_alert_dist_to=%d\n",distance, sonar_alert_dist_from, sonar_alert_dist_to );
    if (distance >= sonar_alert_dist_from && distance <= sonar_alert_dist_to) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
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

        if (frozen) {
            forward = turn = 0; /* 障害物を検知したら停止 */
        } else{
            forward = speed; //前進命令
 
            // PID control by Gray Scale with blue cut
            int16_t sensor = g_grayScaleBlueless;
            int16_t target = GS_TARGET;

            if (state == ST_tracing_L || state == ST_stopping_L) {
                turn = ltPid->compute(sensor, target);
            } else {
                turn = (-1) * ltPid->compute(sensor, target);
            }
        }

        /* 左右モータでロボットのステアリング操作を行う */
        pwm_L = forward - turn;
        pwm_R = forward + turn;

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
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


LineTracer::~LineTracer() {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer destructor", clock->now()));
}

ChallengeRunner::ChallengeRunner(Motor* lm, Motor* rm, Motor* tm) : LineTracer(lm, rm, tm){
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    pwm_L = 20;
    pwm_R = 20;
    pwmMode = 1;
    count = 0;
    procCount = 1;
    traceCnt = 0;
    frozen = false;
}

void ChallengeRunner::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, ChallengeRunner has control", clock->now());
}

void ChallengeRunner::operate() {

    if (frozen) {
        //printf("Stop");
        pwm_L = 0;
        pwm_R = 0;
    }else{
        if (pwmMode != Mode_speed_constant){
            if (++count && count == procCount){
                switch (pwmMode) {
                    case Mode_speed_increaseL:
                        ++pwm_L;
                        break;
                    case Mode_speed_decreaseL:
                        --pwm_L;
                        break;
                    case Mode_speed_increaseR:
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseR:
                        --pwm_R;
                        break;
                    case Mode_speed_increaseLR:
                        ++pwm_L;
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseLR:
                        --pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsLdcrsR:
                        ++pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsRdcrsL:
                        --pwm_L;
                        ++pwm_R;
                        break;
                    default:
                        break;
                }
                count = 0; //初期化
            }
        }
    }
    
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // if (++traceCnt && traceCnt > 50) {
    //     printf(",pwm_L=%d, pwm_R=%d, count=%d, procCount=%d\n", pwm_L,pwm_R,count,procCount);
    //     traceCnt = 0;
    // }
}

//　左右の車輪に駆動にそれぞれ値を指定する sano
void ChallengeRunner::setPwmLR(int p_L,int p_R,int mode,int proc_count) {
    pwm_L = p_L;
    pwm_R = p_R;
    pwmMode = mode;
    procCount = proc_count;
    count = 0;
}

int8_t ChallengeRunner::getPwmL() {
    return pwm_L;
}

int8_t ChallengeRunner::getPwmR() {
    return pwm_R;
}

ChallengeRunner::~ChallengeRunner() {
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner destructor", clock->now()));
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
    tailMotor   = new Motor(PORT_D); // sano
    armMotor   = new Motor(PORT_A); //sano
    
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET aflac2020", 0, CALIB_FONT_HEIGHT*1);
    
    observer = new Observer(leftMotor, rightMotor, armMotor, tailMotor, touchSensor, sonarSensor, gyroSensor, colorSensor);
    observer->freeze(); // Do NOT attempt to collect sensor data until unfreeze() is invoked
    observer->goOnDuty();
    limboDancer = new LimboDancer(leftMotor, rightMotor, tailMotor);
    seesawCrimber = new SeesawCrimber(leftMotor, rightMotor, tailMotor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor);
    challengeRuuner = new ChallengeRunner(leftMotor, rightMotor, tailMotor);
    
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
    //syslog(LOG_NOTICE, "%08u, Captain::decide(): event %s received by state %s", clock->now(), eventName[event], stateName[state]);
    switch (state) {
        case ST_takingOff:
            switch (event) {
                case EVT_cmdStart_R:
                case EVT_cmdStart_L:
                case EVT_touch_On:
                    if (event == EVT_cmdStart_L || (event == EVT_touch_On && _LEFT)) {
                        state = ST_tracing_L;
                    } else {
                        state = ST_tracing_R;
                    }
                    syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                    
                    /* 走行モーターエンコーダーリセット */
                    leftMotor->reset();
                    rightMotor->reset();
                    
                    observer->reset();
                    
                    /* ジャイロセンサーリセット */
                    gyroSensor->reset();
                    ev3_led_set_color(LED_GREEN); /* スタート通知 */
                    
                    observer->freeze();
                    lineTracer->freeze();
                    lineTracer->haveControl();
                    
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
                case EVT_sonar_On:
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_slalom_reached:
                    state = ST_challenge_R;
                    challengeRuuner->haveControl();
                    challengeRuuner->setPwmLR(20,20,Mode_speed_decreaseLR,10);
                    break;
                default:
                    break;
            }
            break;
        case ST_tracing_L:
            switch (event) {
                case EVT_sonar_On:
                    //lineTracer->freeze();
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_slalom_reached:
                    printf("ぶつかり\n");
                    state = ST_challenge_L;
                    armMotor->setPWM(-50);
                    challengeRuuner->haveControl();
                    challengeRuuner->setPwmLR(20,20,Mode_speed_constant,1);
                    clock->sleep(1000);
                    challengeRuuner->setPwmLR(10,10,Mode_speed_constant,1);
                    clock->sleep(1000);
                    challengeRuuner->freeze();
                    clock->sleep(300);
                    challengeRuuner->unfreeze();
                    challengeRuuner->setPwmLR(-20,-20,Mode_speed_constant,1);
                    clock->sleep(500);
                    challengeRuuner->freeze();
                    armMotor->setPWM(80);
                    clock->sleep(300);
                    challengeRuuner->unfreeze();
                    challengeRuuner->setPwmLR(43,40,Mode_speed_decreaseLR,40);
                    break;
                default:
                    break;
            }
            break;
        case ST_stopping_R:
        case ST_stopping_L:
            switch (event) {
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
        case ST_challenge_L:
            switch (event) {
                case EVT_sonar_On:
                    printf("ソナーオン\n");
                    challengeRuuner->setPwmLR(10,-9,Mode_speed_constant,1);
                    challenge_stepNo += 1;
                    break;
                case EVT_obstcl_angle:
                    if (challenge_stepNo == 2){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 6){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(10,15,Mode_speed_decreaseL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 9){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        // challengeRuuner->setPwmLR(27,-27,Mode_speed_constant,1);
                        // clock->sleep(450);
                        challengeRuuner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 13){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,150);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_infront:
                    if (challenge_stepNo == 3){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(32,27,Mode_speed_decreaseL,70);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 10){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        //challengeRuuner->setPwmLR(30,20,Mode_speed_decreaseL,120);
                        challengeRuuner->setPwmLR(32,25,Mode_speed_decreaseL,100);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_reached:
                    if (challenge_stepNo == 0){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(-15,17,Mode_speed_incrsLdcrsR,100);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 4){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(-15,-15,Mode_speed_constant,1);
                        clock->sleep(300);
                        challengeRuuner->setPwmLR(15,-13,Mode_speed_constant,1);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 7){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(-15,10,Mode_speed_constant,1);
                        clock->sleep(500);
                        //challengeRuuner->setPwmLR(-12,15,Mode_speed_constant,1);
                        challengeRuuner->setPwmLR(-10,15,Mode_speed_constant,1);
                        clock->sleep(500);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 11){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,110);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_avoidable:
                    if (challenge_stepNo == 1){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(25,20,Mode_speed_increaseL,150);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 5){
                        challengeRuuner->freeze();
                        clock->sleep(300);
                        challengeRuuner->unfreeze();
                        challengeRuuner->setPwmLR(14,16,Mode_speed_increaseR,80);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 8){
                        challengeRuuner->setPwmLR(20,28,Mode_speed_constant,1);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 12){
                        challengeRuuner->setPwmLR(25,25,Mode_speed_increaseR,33);
                        challenge_stepNo += 1;
                    }else if (challenge_stepNo == 14){
                        challengeRuuner->setPwmLR(25,30,Mode_speed_incrsLdcrsR,100);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_lost_on_the_line:
                    challengeRuuner->setPwmLR(10,-10,Mode_speed_constant,1);
                    break;
                default:
                    break;
            }
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
    delete challengeRuuner; // sano
    delete seesawCrimber;
    delete limboDancer;
    observer->goOffDuty();
    delete observer;
    
    delete tailMotor; // sano
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
