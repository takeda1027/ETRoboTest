//
//  LimboDancer.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"

LimboDancer::LimboDancer(Motor* lm, Motor* rm, Motor* tm,Motor* am) : LineTracer(lm, rm, tm,am) {
    limboMode = LIMBO_MODE_INIT;
    counter = 0;
    angle = calibrator->getPropByInt16("tail.angle.limbo",TAIL_ANGLE_LIMBO);
    startTime = calibrator->getPropByInt16("limbo.time.start",LIMBO_TIME_START);
    moveTime = calibrator->getPropByInt16("limbo.time.move",LIMBO_TIME_MOVE);
    moveTimeAdd = calibrator->getPropByInt16("limbo.time.move.add",LIMBO_TIME_MOVE_ADD);
    startSpeed = calibrator->getPropByInt16("limbo.speed.start",LIMBO_SPEED_START);
    moveSpeed = calibrator->getPropByInt16("limbo.speed.move",LIMBO_SPEED_MOVE);
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer constructor", clock->now()));
}

void LimboDancer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, LimboDancer has control", clock->now());
}

void LimboDancer::operate() {

    switch ( limboMode ) {
    case LIMBO_MODE_INIT:
    case LIMBO_MODE_START:  // Dash small distance and start Limbo
        if ( ++counter > startTime ) {
	    limboMode = LIMBO_MODE_FORWARD1;
	}
        forward = startSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_FORWARD1: // Forward with slow speed by Limbo Style
        //controlTail(angle);
	if ( ++counter > startTime + moveTime ) {
	    limboMode = LIMBO_MODE_BACKWARD1;
	}
        forward = moveSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_BACKWARD1: // Back with slow speed by Limbo Style
        //controlTail(angle);
	if ( ++counter > startTime + moveTime*2 ) {
	    limboMode = LIMBO_MODE_FORWARD2;
	}
        forward = -moveSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_FORWARD2: // forward with slow speed by Limbo Style
        //controlTail(angle);
	if ( ++counter > startTime + moveTime*3 ) {
	    limboMode = LIMBO_MODE_BACKWARD2;
	}
        forward = moveSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_BACKWARD2: // Back with slow speed by Limbo Style
        //controlTail(angle);
	if ( ++counter > startTime + moveTime*4 ) {
	    limboMode = LIMBO_MODE_FORWARD3;
	}
        forward = -moveSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_FORWARD3: // forward with slow speed by Limbo Style
        //controlTail(angle);
	if ( ++counter > startTime + moveTime*5 + moveTimeAdd ) {
	  limboMode = LIMBO_MODE_STOP;
	}
        forward = moveSpeed;
	turn = 0;
	break;
    case LIMBO_MODE_STOP: // stop by Limbo Style
    default:
        //controlTail(angle);
        forward = 0;
	turn = 0;
	break;
    }
    pwm_L = forward;
    pwm_R = forward;
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // display pwm in every PERIOD_TRACE_MSG ms */
    if (++trace_pwmLR * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
        trace_pwmLR = 0;
        _debug(syslog(LOG_NOTICE, "%08u, LimboDancer::operate(): distance = %u, mode = %d", clock->now(), observer->getDistance(), limboMode));
        _debug(syslog(LOG_NOTICE, "%08u, LimboDancer::operate(): pwm_L = %d, pwm_R = %d", clock->now(), pwm_L, pwm_R));
    }
}

LimboDancer::~LimboDancer() {
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer destructor", clock->now()));
}

/**********************
 * 仮の実装。正しい実装ができたら、下のプログラムは削除する。
 *********************/

Calibrator::Calibrator()
{
    _debug(syslog(LOG_NOTICE, "%08u, Calibrator default constructor", clock->now()));
}

void	Calibrator::readPropFile( const char* filename )
{
}

int16_t	Calibrator::getPropByInt16( const char* propname, int16_t deflt )
{
    return deflt;
}

Calibrator::~Calibrator()
{
    _debug(syslog(LOG_NOTICE, "%08u, Calibrator destructor", clock->now()));
}

Calibrator* calibrator;

/********************
 * ここまで削除
 ********************/
