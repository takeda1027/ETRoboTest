//
//  LimboDancer.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef LimboDancer_hpp
#define LimboDancer_hpp

#include "crew.hpp"

#define TAIL_ANGLE_LIMBO      60  /* リンボーダンス時の角度[度] */

#define LIMBO_TIME_START      30  /* リンボーの態勢になる時間[回] */
#define LIMBO_TIME_MOVE     1000  /* リンボーの前進・後退時間[回] */
#define LIMBO_TIME_MOVE_ADD  500  /* リンボー終了前に進む時間[回] */

#define LIMBO_SPEED_START     30  /* リンボーの態勢になる時の速度 */
#define LIMBO_SPEED_MOVE      10  /* リンボーの移動時の速度 */

#define LIMBO_MODE_INIT	       0  /* 初期状態 */
#define LIMBO_MODE_START       1  /* リンボーの態勢になる */
#define LIMBO_MODE_FORWARD1    2  /* 前進１回目 */
#define LIMBO_MODE_BACKWARD1   3  /* 後退１回目 */
#define LIMBO_MODE_FORWARD2    4  /* 前進２回目 */
#define LIMBO_MODE_BACKWARD2   5  /* 後退２回目 */
#define LIMBO_MODE_FORWARD3    6  /* 前進３回目 */
#define LIMBO_MODE_STOP        7  /* 停止 */

class LimboDancer : public LineTracer {
private:
    int16_t angle;
    int16_t startTime;
    int16_t moveTime;
    int16_t moveTimeAdd;
    int16_t startSpeed;
    int16_t moveSpeed;
protected:
public:
    LimboDancer();
    LimboDancer(Motor* lm, Motor* rm, Motor* tm,Motor* am);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~LimboDancer();
    int16_t limboMode;
    int16_t counter;
};


/**********************
 * 仮の実装。正しい実装ができたら、下のプログラムは削除する。
 *********************/

class Calibrator
{
public:
    Calibrator();
    void	readPropFile( const char* filename );
    int16_t	getPropByInt16( const char* propname, int16_t deflt );
    ~Calibrator();
};

extern Calibrator*  calibrator;

/********************
 * ここまで削除
 ********************/

#endif /* LimboDancer_hpp */
