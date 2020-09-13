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
int16_t g_color_brightness=0;//sano_t
int16_t g_b3_turn_flg = 0; //sano_t
int16_t g_turn_cnt=0;//sano_t



Radioman::Radioman() {
    //_debug(syslog(LOG_NOTICE, "%08u, Radioman constructor", clock->now()));
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
            //syslog(LOG_NOTICE, "%08u, StartCMD R-mode received", clock->now());
            captain->decide(EVT_cmdStart_R);
            break;
        case CMD_START_L:
        case CMD_START_l:
            //syslog(LOG_NOTICE, "%08u, StartCMD L-mode received", clock->now());
            captain->decide(EVT_cmdStart_L);
            break;
        case CMD_DANCE_D:
        case CMD_DANCE_d:
            //syslog(LOG_NOTICE, "%08u, LimboDancer forced by command", clock->now());
            captain->decide(EVT_cmdDance);
            break;
        // case CMD_CRIMB_C:
        // case CMD_CRIMB_c:
        //     syslog(LOG_NOTICE, "%08u, SeesawCrimber forced by command", clock->now());
        //     captain->decide(EVT_cmdCrimb);
        //     break;
        case CMD_STOP_S:
        case CMD_STOP_s:
            //syslog(LOG_NOTICE, "%08u, stop forced by command", clock->now());
            captain->decide(EVT_cmdStop);
            break;
        default:
            break;
    }
}

Radioman::~Radioman() {
    //_debug(syslog(LOG_NOTICE, "%08u, Radioman destructor", clock->now()));
    fclose(bt);
}

Observer::Observer(Motor* lm, Motor* rm, Motor* am, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    //_debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;//sano_t
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
    a_dis = 99; //sano
    b_dis = 99; //sano
    b1 =false; //sano
    b2 =false; //sano
    b3 =false; //sano
    ran_flg = 0; //sano t
    roots_flg=0; //sano_t
    slalom_flg=false; //sano テストのためtureに
    obj_flg=false; //sano
    right_angle =false; //sano テストのためtureに
    prev_deg=0; //sano_t
    yobi_deg=0; //sano_t
    lim_deg =0;//sano_t
    prev_dis=0; //sano_t

 
    gyroSensor->setOffset(0);//sano

    //ot_r = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_g = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_b = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

//sano t
void Observer::setGyro(GyroSensor* gs) {
    gyroSensor = gs;
}

void Observer::goOnDuty() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    //_debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
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
//sano_t
int16_t Observer::getAzimuth_180() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    if(degree>180){
        degree -=360;
    }
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

//sano_t 角度累積分を計算
int16_t Observer::getAngleDgree(int16_t prev_x,int16_t x){

    int16_t hoge = prev_x-x;
    if(hoge < 0){
        hoge = hoge * -1;
    }
    if(hoge > 180){
        hoge = 360-hoge;
    }

    return hoge;    
}

//sano_t 絶対値　関数がうまく使えないため
int16_t Observer::abs_p(int x){
    if(x<0){    
        x=x*(-1);
    }
    return (int16_t)x;
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
    g_color_brightness = colorSensor->getBrightness(); //sano_t


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
        //syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        captain->decide(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag && g_b3_turn_flg <=0) { //sano_t
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
        //syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        captain->decide(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        //syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        captain->decide(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        //syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        captain->decide(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        //syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        captain->decide(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        result = check_lost();
        if (result && !lost_flag) {
            //syslog(LOG_NOTICE, "%08u, line lost", clock->now());
            lost_flag = true;
            captain->decide(EVT_line_lost);
        } else if (!result && lost_flag) {
            //syslog(LOG_NOTICE, "%08u, line found", clock->now());
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
                 //syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
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
    }else if(b1 && cur_rgb.b - cur_rgb.r < 40){
        b1 = false; //１つめのブルー検知フラグを落とす
    }

    //b-r青判定、スラロームが近い（前方障害あり）、ブルー２個目b2フラグ立てる
    if(cur_rgb.b - cur_rgb.r > 60 && dis < 50  && cur_rgb.b <=255 && cur_rgb.r<=255 && g_b3_turn_flg==0){
        b2 =true;
        //armMotor->setPWM(20);
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
    if(slalom_flg && g_b3_turn_flg==0){
        //アーム停止
        //armMotor->setPWM(0);
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
        //printf("g_angle=%d,slalom_flg=%d\n",g_angle,slalom_flg);
         if(g_angle > 10){
            slalom_flg = false;
        }
 
    }


    //b3テスト用 sano    
    if(g_angle > 10 && g_b3_turn_flg==0){
        //printf("通った１\n");
        captain->decide(EVT_go_b3); 
    }
//    printf("g_angle=%d,kyori=%d\n",g_angle,getDistance());
    if( getDistance()>480 && g_b3_turn_flg==0){
    //b3テスト用ここまで

    //sano　青判定３回目 sano_t
 //   if( cur_rgb.b - cur_rgb.r > 60  && !b3){
//    if( cur_rgb.b - cur_rgb.r > 60 && right_angle && !b3){
        //printf(",b-r=%d,r=%d,g=%d,b=%d,right_angle=%d\n", cur_rgb.b - cur_rgb.r,cur_rgb.r,cur_rgb.g,cur_rgb.b,right_angle);
        b3 = true;
        //b3で回転検知のグローバルフラグをたてる
        g_b3_turn_flg = 10;
        captain->decide(EVT_turnb3); // ソナー稼働回転、方向を調整
    }
    //sano_t 青３回転地点でソナー探知
    else if(g_b3_turn_flg ==10){
        //printf("dis=%d,slalom_flg=%d\n",sonarSensor->getDistance(),slalom_flg);
        //clock->sleep(10);
        if(sonarSensor->getDistance() > 155 && sonarSensor->getDistance() <250){
            prev_deg = getAzimuth();
            yobi_deg = prev_deg;
            g_b3_turn_flg = 20;
        }
    }
    //物体検知後、放射角度を加味して正面に sano_t
    else if(g_b3_turn_flg == 20 && getAzimuth() - prev_deg > 17){
        g_b3_turn_flg = 30;
        prev_dis = getDistance();
        captain->decide(EVT_go_b3); // ラインに接近
        clock->sleep(1000);//スラローム後、大きくラインを外しても、手前の黒ラインに引っかからないためにスリープ
    }

    //  sano_t
    else if(g_b3_turn_flg == 30){
        
        //黒を見つけたら、下向きのライントレース
        if(cur_rgb.g + cur_rgb.b <= 80 && cur_rgb.r <=60 && cur_rgb.g <=40 && cur_rgb.b <=45 && ran_flg==0){
//        if(cur_rgb.g + cur_rgb.b + cur_rgb.r <= 150 && cur_rgb.r <=60 && abs_p(cur_rgb.g -cur_rgb.b) <=20 && ran_flg==0){
            captain->decide(EVT_turnRight); // ここで直角ターン
            ran_flg=1;
        }else if(ran_flg==1){
            prev_deg= getAzimuth();
            g_b3_turn_flg=40;
            //g_turn_cnt = 0;
            captain->decide(EVT_go_line_p); // 比例制御ライントレース
            roots_flg = 1;
            ran_flg=0;
        }
        
        //赤を見つけたら、赤からブロックへ  直進
        else if(cur_rgb.r - cur_rgb.b >=40 && cur_rgb.g <65 && cur_rgb.r - cur_rgb.g >30 && ran_flg == 0){
             captain->decide(EVT_turnRight2); // ここで物体へターン
             prev_deg=getAzimuth();
             printf("赤色超えました\n");
             ran_flg = 2;
        }
         else if(ran_flg == 2 && getAzimuth() - prev_deg >73){
             printf("赤色超えました2\n");
             captain->decide(EVT_go_30); // 物体に接近
             g_b3_turn_flg=50;
             ran_flg = 0;
             roots_flg=2;
        }

        //黄色を見つけたら、右に直進のライントレース
        else if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30 && ran_flg == 0){
            prev_dis = getDistance();
            captain->decide(EVT_turn_right_180); // ちょっとだけ外へずらす 
            clock->sleep(100);
            captain->decide(EVT_go_b3); // 物体に接近
            printf("黄色超えました1\n");
            ran_flg = 3;
        }
        else if(ran_flg == 3 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 && getDistance()-prev_dis>50){
            printf("黄色超えました2\n");
            captain->decide(EVT_go_line_t); // 物体に接近
            g_b3_turn_flg=50;
            ran_flg = 0;
        }
    }

    else if(g_b3_turn_flg==40){        
        //黄色を見つけたらターン
        if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30 && ran_flg==0){
            int x=getAzimuth();
            printf("ここのturn =%d,prev_deg=%d,azi=%d,sa=%d\n",g_turn_cnt,prev_deg,x,prev_deg-x);
            yobi_deg=prev_deg-x;
            prev_deg=getAzimuth();
            captain->decide(EVT_step); // 一旦停止
            captain->decide(EVT_turnLeft_m15p50); // 回転開始
            ran_flg = 1;
        }else if(ran_flg == 1 && prev_deg -getAzimuth() + yobi_deg > 67 + 15){
            captain->decide(EVT_step); // 一旦停止
            prev_dis = getDistance();
            captain->decide(EVT_go_30); // 物体に接近
            ran_flg = 5;
            printf("ここまで黄色エリア１ prev_deg=%d,azi=%d,sa=%d,gosa=%d\n",prev_deg,getAzimuth(),prev_deg -getAzimuth(),yobi_deg);
        }
        else if(ran_flg==5 && getDistance()-prev_dis > 180 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 ){
            printf("ここまで黄色エリア２\n");
            g_b3_turn_flg=50;
            ran_flg = 0;
            //if(roots_flg==3){
            //    captain->decide(EVT_go_30); // 物体に接近            
            //}else{
                captain->decide(EVT_go_line_t); // 物体に接近
            //}
        }
        //赤を見つけたら黒を見つけるまで直進、その後ライントレース
        else if(cur_rgb.r - cur_rgb.b >=40 && cur_rgb.g <60 && cur_rgb.r - cur_rgb.g >30 && ran_flg==0){
            ran_flg = 2;
            captain->decide(EVT_go_30); // 物体に接近            
        }else if( cur_rgb.r - cur_rgb.b < 20 && ran_flg==2){
            ran_flg = 3;
        }else if( cur_rgb.r - cur_rgb.b >=40 && ran_flg==3){
            ran_flg = 4;
        }else if( cur_rgb.r + cur_rgb.g + cur_rgb.b > 300 && ran_flg==4){
            captain->decide(EVT_turn_180); // ラインに戻る
            clock->sleep(10);
            captain->decide(EVT_go_line_t); // 黄色に接近
            ran_flg = 0;
        }else if( cur_rgb.r + cur_rgb.g + cur_rgb.b <= 100 && ran_flg==4){
            captain->decide(EVT_go_line_t); // 黄色に接近
            ran_flg = 0;
        }
    }
    else if(g_b3_turn_flg==50){                
        //ブロックに直進、ブロックの黄色を見つけたら
        if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160){
                printf("ブロックに来ました\n");                 
                g_b3_turn_flg=60;
        }
    }
    else if(g_b3_turn_flg == 60){
        if(ran_flg == 0){
            captain->decide(EVT_turn_180); // 180度回頭
            prev_deg = getAzimuth();
            yobi_deg =0;
            ran_flg = 1;
            if(roots_flg==2){ //赤〇ポイントからのブロック到達の場合
                lim_deg= 140;
            }else{            //黒ライン、黄色直進からのブロック到達の場合
                lim_deg = 80;
            }
        }
        else if(yobi_deg > lim_deg && ran_flg ==1){
            ran_flg=2;
            captain->decide(EVT_turn_180_slowly); // 180度回頭
            //せん回を柔らかく
        }else if(ran_flg ==1){ //角度が条件に該当しない場合、差分を累積していく。
            int16_t x=getAzimuth();
            int16_t hoge = prev_deg-x;
            prev_deg = x;
            if(hoge < 0){
                hoge = hoge * -1;
            }
            if(hoge < 180){
                yobi_deg += hoge;
            }else{
                yobi_deg += 360-hoge;
            }
            printf("yobi_deg=%d,hoge=%d,",yobi_deg,hoge);

        }else if(ran_flg==2){
            if(sonarSensor->getDistance()==255){
                ran_flg=3;
                yobi_deg=0;
                prev_deg=getAzimuth();
            }
        }else if(ran_flg==3){
            int x= getAzimuth();
            yobi_deg += getAngleDgree(prev_deg,getAzimuth());
            prev_deg = x;
            if(yobi_deg>=0){
                captain->decide(EVT_go_b3); // ガレージラインへ
                g_b3_turn_flg=70;
                ran_flg=0;
                prev_dis = getDistance();
            }
            printf("yobi=%d,",yobi_deg);
        }
    }
    else if(g_b3_turn_flg==70 && getDistance()-prev_dis > 500){
        //黒線ブロックを超えるまで、一定距離を走行
        g_b3_turn_flg=80;
    }
    else if(g_b3_turn_flg==80){
        if(cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60 && ran_flg==0){ //緑を見つけたら、減速
            captain->decide(EVT_go_slowly);
            //clock->sleep(200);
            ran_flg=1;
        }else if(cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60 && ran_flg==1){//緑を見つけたらカーブ開始
            captain->decide(EVT_turnRight_slowly); 
            //clock->sleep(200);
            ran_flg=2;
        }else if(cur_rgb.r + cur_rgb.g <= 50 && ran_flg < 3){  //青または黒をみつけたら、完全にターン)
            captain->decide(EVT_step); // 一旦停止
            captain->decide(EVT_turnb3); // ここでターン
            clock->sleep(500);
            ran_flg=3;
        }else if(ran_flg==3){
            int16_t x = sonarSensor->getDistance();
            if(x>35 && x<250){ // ガレージの奥の距離を捉えたら直進
                prev_deg=getAzimuth();
                yobi_deg=0;
                ran_flg=4;
            }
        }else if(ran_flg==4){
            yobi_deg += getAngleDgree(prev_deg,getAzimuth());
            prev_deg=getAzimuth();
            printf("yobi=%d,",yobi_deg);
            if(yobi_deg > 7){
                captain->decide(EVT_go_slowly);
                ran_flg=5;
            }
        }else if(ran_flg == 5 && sonarSensor->getDistance()<20){
            captain->decide(EVT_stop);
        }

    }
    

    if (g_b3_turn_flg >=10 && g_b3_turn_flg <=80){
        printf("dis=%d,distance=%d,prev_dis=%d,deg=%d,prev_deg=%d,r=%03u, g=%03u, b=%03u,state=%d,angle=%d,ran_flg=%d,\n",sonarSensor->getDistance(),getDistance(),prev_dis,getAzimuth(),prev_deg,g_rgb.r, g_rgb.g, g_rgb.b,g_b3_turn_flg,g_angle,ran_flg);//sano t
    }
    //printf("deg=%lf,prev_deg=%lf\n",getAzimuth(),prev_deg);//sano t



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
    //_debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}

Navigator::Navigator() {
    //_debug(syslog(LOG_NOTICE, "%08u, Navigator default constructor", clock->now()));
    ltPid = new PIDcalculator(P_CONST, I_CONST, D_CONST, PERIOD_NAV_TSK, -16, 16); 
}

void Navigator::goOnDuty() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_NAV_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_NAV_TSK/2/1000); // wait a while
    //_debug(syslog(LOG_NOTICE, "%08u, Navigator handler set", clock->now()));
}

void Navigator::goOffDuty() {
    activeNavigator = NULL;
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_NAV_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_NAV_TSK/2/1000); // wait a while
    //_debug(syslog(LOG_NOTICE, "%08u, Navigator handler unset", clock->now()));
}

Navigator::~Navigator() {
    //_debug(syslog(LOG_NOTICE, "%08u, Navigator destructor", clock->now()));
}

AnchorWatch::AnchorWatch(Motor* tm) {
    //_debug(syslog(LOG_NOTICE, "%08u, AnchorWatch constructor", clock->now()));
}

void AnchorWatch::haveControl() {
    activeNavigator = this;
   // syslog(LOG_NOTICE, "%08u, AnchorWatch has control", clock->now());
}

void AnchorWatch::operate() {
    //controlTail(TAIL_ANGLE_STAND_UP,10); /* 完全停止用角度に制御 */
}

AnchorWatch::~AnchorWatch() {
    //_debug(syslog(LOG_NOTICE, "%08u, AnchorWatch destructor", clock->now()));
}

LineTracer::LineTracer(Motor* lm, Motor* rm, Motor* tm,Motor* am) { //sano_t アーム追加
    //_debug(syslog(LOG_NOTICE, "%08u, LineTracer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor    = am;
    trace_pwmLR = 0;
    speed       = SPEED_NORM;
    frozen      = false;
    arm_flg     = false;//sano t
    turn_flg = 0;//sano t
    turn_p = 0; //sano_t
}

void LineTracer::haveControl() {
    activeNavigator = this;
    //syslog(LOG_NOTICE, "%08u, LineTracer has control", clock->now());
}

void LineTracer::operate() {
    //controlTail(TAIL_ANGLE_DRIVE,10); /* バランス走行用角度に制御 */

    if(turn_flg==1){//直角ターン命令のため。駆動力を自由に設定可 sano

        pwm_L = pwm_p_L;
        pwm_R = pwm_p_R;
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
        clock->sleep(50);
        //printf("turn_flg経由,pwm_L=%d,pwm_R=%d,",pwm_L,pwm_R);

    }else if(turn_flg==2){
        turn_p = calc_prop_P(); 
        pwm_L = pwm_p_L - turn_p;
        pwm_R = pwm_p_R + turn_p;
        g_turn_cnt = turn_p;
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

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
        printf("turn=%d,forward=%d,\n",turn,forward);
         g_turn_cnt = turn;
         pwm_L = forward - turn;
        pwm_R = forward + turn;

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

    } //sano

    //sano_t アームを一定位置で上げて維持
    if(arm_flg){
        if(armMotor->getCount()>cntArm){
            armMotor->setPWM(-pwmArm);
        }else if(armMotor->getCount()<cntArm){
            armMotor->setPWM(pwmArm);
        }else if(armMotor->getCount()==cntArm){
            armMotor->setPWM(3);
        }
    }

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
void LineTracer::turnC(int t,int p_L,int p_R) { //sano
    turn_flg = t;
    pwm_p_L = p_L;
    pwm_p_R = p_R;
}

//　アームを一定の場所で維持する sano_t
void LineTracer::armDo(bool a,int pwm_arm,int cnt_arm) { //sano
    arm_flg = a;      //アーム稼働フラグ
    pwmArm = pwm_arm; //駆動力
    cntArm = cnt_arm; //各位置
}
//sano t 比例制御turnお返し用
float LineTracer::calc_prop_P() {
  const float Kp = 0.83;
  const int target = 18;
  const int bias = 0;
  
  int diff = g_color_brightness - target; 
  return (Kp * diff + bias);                       
}

LineTracer::~LineTracer() {
    //_debug(syslog(LOG_NOTICE, "%08u, LineTracer destructor", clock->now()));
}

Captain::Captain() {
   // _debug(syslog(LOG_NOTICE, "%08u, Captain default constructor", clock->now()));
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
    limboDancer = new LimboDancer(leftMotor, rightMotor, tailMotor,armMotor);
    //seesawCrimber = new SeesawCrimber(leftMotor, rightMotor, tailMotor,armMotor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor,armMotor); //sano_t アーム追加
    
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
                    } else {  // event == EVT_cmdStart_R || (event == EVT_touch_On && !_LEFT)
                        state = ST_tracing_R;
                    }
                    //syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                    
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
                    //syslog(LOG_NOTICE, "%08u, Departed", clock->now());
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
                     clock->sleep(500); // wait a little
                     lineTracer->unfreeze();
                    // observer->unfreeze();
                    break;
                case EVT_turnCnr:
                // sano
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(1,100,0);
                    printf("ターンしています\n");
                    clock->sleep(765); // wait a little
                    printf("スリープ終了した\n");
                    lineTracer->turnC(0,0,0);
                    printf("ターンを０に\n");
                    lineTracer->freeze();
                    printf("回転後の停止１\n");
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    printf("動き出します\n");
                    break;
                case EVT_turnRight:
                // sano
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(1,50,-50);
                    printf("ターンしています\n");
                    clock->sleep(620); // wait a little
                    printf("スリープ終了した\n");
                    lineTracer->turnC(0,0,0);
                    printf("ターンを０に\n");
                    lineTracer->freeze();
                    printf("回転後の停止２\n");
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    printf("動き出します\n");
                break;
                case EVT_turnLeft_m15p50:
                    lineTracer->turnC(1,-15,50);
                break;
                case EVT_turnRight_slowly:
                    lineTracer->turnC(1,15,12);
                break;
                case EVT_go_slowly:
                    lineTracer->turnC(1,15,15);
                break;
                case EVT_turnLeft:
                // sano
                    turn_x = g_turn_cnt;
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    lineTracer->turnC(1,-15,50);
                    if(turn_x>3){
                        printf("ターンが３以上\n");
                        printf("ターンしています\n");
                        clock->sleep(550); // wait a little
                    }else if(turn_x > -3){
                        printf("ターンしています\n");
                        clock->sleep(600); // wait a little
                        printf("ターンが-３以上\n");
                    }else{
                        printf("ターンしています\n");
                        clock->sleep(700); // wait a little
                        printf("ターンがそれ以外\n");
                    }
                    printf("スリープ終了した\n");
                    lineTracer->turnC(0,0,0);
                    printf("ターンを０に\n");
                    lineTracer->freeze();
                    printf("回転後の停止３\n");
                    clock->sleep(1000); // wait a little
                    lineTracer->unfreeze();
                    printf("動き出します\n");
                    break;
                case EVT_turnb3:
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                    lineTracer->unfreeze();
                    lineTracer->turnC(1,8,-8);
                    break;
                case EVT_go_b3:
                    lineTracer->turnC(1,50,50);
                break;
                case EVT_turnRight2:
                    lineTracer->turnC(1,10,0);
                break;
                case EVT_go_30:
                    lineTracer->turnC(1,30,30);
                break;
                case EVT_stop:
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                break;
                case EVT_step:
                    lineTracer->turnC(0,0,0);
                    lineTracer->freeze();
                    clock->sleep(1000);
                    lineTracer->unfreeze();
                break;
                case EVT_turn_180:
                    lineTracer->turnC(1,4,30);
                break;
                case EVT_turn_180_slowly:
                    lineTracer->turnC(1,2,20);
                break;
                case EVT_turn_right_180:
                    lineTracer->turnC(1,0,40);
                break;
                // case EVT_arm_up:
                //     lineTracer->armDo(true,100,60);
                //     printf("armangle=%d,gyro=%d\n",armMotor->getCount(),gyroSensor->getAngle());
                // break;
                case EVT_go_line_t:
                    lineTracer->setSpeed(30);
                    lineTracer->turnC(0,30,30);
                break;
                case EVT_go_line_p:
                     lineTracer->turnC(2,30,30);               
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
    //syslog(LOG_NOTICE, "%08u, Landing...", clock->now());
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
    armMotor->reset(); //sano_t
    
    delete anchorWatch;
    delete lineTracer;
    //delete seesawCrimber;
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
    //_debug(syslog(LOG_NOTICE, "%08u, Captain destructor", clock->now()));
}
