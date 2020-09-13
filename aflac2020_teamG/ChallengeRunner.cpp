//
//  ChallengeRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "ChallengeRunner.hpp"

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

//　Activate challengeRunner PWM control according to challenge_stepNo
void ChallengeRunner::runChallenge() {

    switch (challenge_stepNo) {
        case 0:
            printf("ぶつかり\n");
            haveControl();
            setPwmLR(20,20,Mode_speed_constant,1);
            clock->sleep(1000);
            setPwmLR(10,10,Mode_speed_constant,1);
            clock->sleep(1000);
            rest();
            setPwmLR(-20,-20,Mode_speed_constant,1);
            clock->sleep(500);
            rest();
            setPwmLR(43,40,Mode_speed_decreaseLR,40);
            break;
        case 1:
            setPwmLR(-15,17,Mode_speed_incrsLdcrsR,100);
            break;
        case 2:
            setPwmLR(25,20,Mode_speed_increaseL,150);
            break;
        case 3:
            setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
            break;
        case 4:
            setPwmLR(32,27,Mode_speed_decreaseL,70);
            break;
        case 5:
            setPwmLR(-15,-15,Mode_speed_constant,1);
            clock->sleep(300);
            setPwmLR(15,-13,Mode_speed_constant,1);
            break;
        case 6:
            setPwmLR(14,16,Mode_speed_increaseR,80);
            break;
        case 7:
            setPwmLR(10,15,Mode_speed_decreaseL,90);
            break;
        case 8:
            setPwmLR(-15,10,Mode_speed_constant,1);
            clock->sleep(500);
            setPwmLR(-10,15,Mode_speed_constant,1);
            clock->sleep(500);
            break;
        case 9:
            setPwmLR(20,28,Mode_speed_constant,1);
            break;
        case 10:
            setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
            break;
        case 11:
            setPwmLR(32,25,Mode_speed_decreaseL,100);
            break;
        case 12:
            setPwmLR(15,-15,Mode_speed_incrsRdcrsL,110);
            break;
        case 13:
            setPwmLR(25,25,Mode_speed_increaseR,30);
            break;
        case 14:
            setPwmLR(15,-15,Mode_speed_incrsRdcrsL,150);
            break;
        case 15:
            setPwmLR(25,30,Mode_speed_incrsLdcrsR,100);
            break;
        default:
            break;
    }
}

//　左右の車輪に駆動にそれぞれ値を指定する
void ChallengeRunner::setPwmLR(int p_L,int p_R,int mode,int proc_count) {
    pwm_L = p_L;
    pwm_R = p_R;
    pwmMode = mode;
    procCount = proc_count;
    count = 0;
}

// rest for a while
void ChallengeRunner::rest() {
    freeze();
    clock->sleep(300);
    freeze();
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