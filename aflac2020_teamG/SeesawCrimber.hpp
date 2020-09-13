//
//  SeesawCrimber.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef SeesawCrimber_hpp
#define SeesawCrimber_hpp

#include "crew.hpp"

#define TAIL_ANGLE_NORMAL_RUN    80  /* 通常走行時のしっぽの角度[度] */
#define TAIL_ANGLE_SEESAW_FIRST  40  /* シーソー上り始めのしっぽの角度[度] */

#define PERIOD_SEESAW	250		// 次の状態へ遷移するまでの回数

#define SEESAW_00	0	// 状態遷移番号
#define SEESAW_01	1	// 状態遷移番号
#define SEESAW_02	2	// 状態遷移番号
#define SEESAW_03	3	// 状態遷移番号
#define SEESAW_04	4	// 状態遷移番号
#define SEESAW_05	5	// 状態遷移番号
#define SEESAW_06	6	// 状態遷移番号
#define SEESAW_07	7	// 状態遷移番号
#define SEESAW_08	8	// 状態遷移番号
#define SEESAW_09	9	// 状態遷移番号
#define SEESAW_10	10	// 状態遷移番号
#define SEESAW_11	11	// 状態遷移番号
#define SEESAW_12	12	// 状態遷移番号
#define SEESAW_13	13	// 状態遷移番号

//チューニング用定数（準備中）
#define STOP_WHEEL		0	// 停止時の車輪の値
#define STOP_TAIL_FLAT	80	// 平面で停止時の尻尾の値


#define START_SPEED 0  /* START時の各モーターに与える値 */
#define S_PORT_4 3     // GyroSensorポート

#define PROP_NAME_LEN	48	// プロパティー名の最大長
#define NUM_PROPS	13	// プロパティーの個数

class SeesawCrimber : public LineTracer {
private:
	int16_t s_time;
	int16_t s_counter;
	int16_t s_speed;
	int16_t s_mode;
	int16_t s_angle;
	int16_t s_trace_counter;
	int16_t angleNow;
	int16_t angleBef;
	int16_t collisiontime;
	int16_t waitflg;


	struct property{
		char name[PROP_NAME_LEN];
		int value;
	} ;

	struct property props[NUM_PROPS];

	int readLine( FILE* file, char* dst, size_t len );
	void readPropFile( const char* filename );
	int getProp( const char* propname );

    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*			tailMotor;
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;

protected:
public:
    SeesawCrimber();
    SeesawCrimber(Motor* lm, Motor* rm, Motor* tm,Motor* am);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~SeesawCrimber();
};

#endif /* SeesawCrimber_hpp */
