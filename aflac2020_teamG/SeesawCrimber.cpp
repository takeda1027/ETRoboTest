//
//  SeesawCrimber.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"
#include <string.h>
#include <stdlib.h>
//#include "balancer.h"

SeesawCrimber::SeesawCrimber(Motor* lm, Motor* rm, Motor* tm,Motor* am) : LineTracer(lm, rm, tm,am) {
	readPropFile("/ev3rt/res/Seesaw_prop.txt");
	leftMotor = lm;
	rightMotor = rm;
	tailMotor = tm;
	
	s_trace_counter = 0;
	angleNow = 0;
	angleBef = 0;
	collisiontime = 0;
	waitflg = 0;

    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber constructor", clock->now()));
}

void SeesawCrimber::haveControl() {
    activeNavigator = this;

    // private変数の初期化
    s_time = 0;
    s_counter = 0;
    s_speed = 0;
    s_mode = SEESAW_00;
    s_angle = 100;

	// 初期状態での尻尾角度をTAIL_ANGLE_NORMAL_RUNに合わせて設定
	//controlTail(TAIL_ANGLE_NORMAL_RUN);

	// ジャイロセンサーの角速度を0にセットする
	//gyroSensor->reset();


    // ログ出力
    syslog(LOG_NOTICE, "%08lu, SeesawCrimber has control", clock->now());
}

void SeesawCrimber::operate() {

	// 開始ログ出力
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		_debug(syslog(LOG_NOTICE, "%08u, ■■■SeesawCrimber::operate():開始,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
	}


	// 状態遷移番号を判定し、各状態の処理を実行する
	// ●がついている箇所がチューニングポイント
	switch( s_mode ){
	//■□■SEESAW_00 停止：シーソー挑戦前に一度停止し、状況を整える■□■
	case SEESAW_00:
		//両輪を停止し、尻尾の角度を停止状態●に設定する
		pwm_L = STOP_WHEEL;			//左輪のパワー
		pwm_R = STOP_WHEEL;			//右輪のパワー
		s_angle = STOP_TAIL_FLAT;	//尻尾の角度●

		//判定値●秒経過したら両輪をゆっくり動かす
		if ( ++s_counter > PERIOD_SEESAW*5){
			pwm_L = 3;	//左輪のパワー●
			pwm_R = 3;	//右輪のパワー●
		}

		//判定値●秒経過（＝ゆっくり動いてから判定値●秒経過）したら、次のCaseに遷移する処理を行う
		//1.遷移先にSEESAW_01を指定
		//2.カウンタを初期化
		//3.尻尾の角度を乗り上げるための前傾姿勢にするための角度に調整●する
		//4.衝撃検知用の1回前の角速度（angleBef）を初期化する
		if ( ++s_counter > PERIOD_SEESAW*8){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():CaseChange,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			s_mode = SEESAW_01;	//SEESAW_01へ遷移
			s_counter = 0;		//カウンタ
			s_angle =100;		//尻尾の角度●
			angleBef = 0;		//衝撃検知用の1回前の角速度
		}

		break;

	//■□■SEESAW_01 シーソーに上る（加速）：シーソー両輪のパワーを上げて加速し、シーソーに乗り上げる■□■
	//衝撃検知後、すぐに状態遷移をするとうまく動かないため、時間差で停止すように処理をする
	case SEESAW_01:
		//スピードを設定
		pwm_L = 80;	//左輪のパワー●
		pwm_R = 80;	//右輪のパワー●
		++s_counter; //カウントアップ

		//現在の角速度を取得
		angleNow = g_anglerVelocity;

		//初回に限り、角速度の差分に大きな値が出る可能性があるため、差分が出ないように値を補正する
		if(angleBef == 0){
	        angleBef = angleNow;	//1回前の角速度に現在の角速度を設定
		}

		//現在の角速度と1回前の角速度を比較し差分が判定値●以上ある場合、シーソーにぶつかったと判定し処理を行う
		//1.現在の時間を衝突時間に設定する（時間差で処理をするため）
		//2.次の状態に遷移するためのフラグ（waitflg）を立てる
		//3.尻尾が乗り上げる際に邪魔にならないように少し尻尾の角度を上げる●
		if(abs(angleNow - angleBef) >= 50 && waitflg == 0){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():WaitmodeIn,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			collisiontime = s_counter;	//衝突した時間の設定
			waitflg = 1;				//状態遷移用のフラグ
//			s_angle = 80;				//尻尾の角度●
		}

		//現在の時間と衝突時間を比較し判定値●秒以上経過している場合、次のCaseへ遷移する処理を行う
		//1.遷移先にSEESAW_02を指定
		//2.カウンタを初期化
		//3.状態遷移用のフラグを初期化
		//4.尻尾の角度を坂道で止まるための角度●に調整する
		//5.衝撃検知用の1回前の角速度（angleBef）を初期化(0)する
		if(s_counter - collisiontime >= 300 && waitflg == 1){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():CaseChange,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			s_mode = SEESAW_02;	//SEESAW_02へ遷移
			s_counter = 0;		//カウンタ
			waitflg = 0;		//状態遷移用のフラグ
			s_angle = 100;		//尻尾の角度●
			angleBef = 0;		//衝撃検知用の1回前の角速度
		}

		//検証用ログ出力（角速度差分）
        _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber:★ angleABS = %d", clock->now() ,abs(angleNow - angleBef) ));

        //1回前の角速度に現在の角速度を設定
        angleBef = angleNow;

		break;

	//■□■SEESAW_02 シーソー上で止まる：シーソー上で一度停止する■□■
	case SEESAW_02:
		//スピードを設定
		pwm_L = STOP_WHEEL;		//左輪のパワー●
		pwm_R = STOP_WHEEL;		//右輪のパワー●

		//指定の秒数●を経過したら次のCaseへ遷移する処理を行う
		//1.遷移先にSEESAW_03を指定
		//2.カウンタを初期化
		//3.尻尾の角度を坂を上るための角度●に調整する
		//4.衝撃検知用の1回前の角速度（angleBef）を初期化(0)する
		if ( ++s_counter > PERIOD_SEESAW*2 ){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():CaseChange,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			s_mode = SEESAW_03;	//SEESAW_03へ遷移
			s_counter = 0;		//カウンタ
			s_angle = 100;		//尻尾の角度●
			angleBef = 0;		//衝撃検知用の1回前の角速度
		}

		//検証用ログ出力（角速度差分）
       _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber:★ angleABS = %d", clock->now() ,abs(angleNow - angleBef) ));

		break;

	//■□■SEESAW_03 シーソー上でゆっくり進む：シーソー上の傾斜が切り替わる地点までゆっくり前進する■□■
	case SEESAW_03:
		//スピードを設定
		pwm_L = 1;		//左輪のパワー●
		pwm_R = 1;		//右輪のパワー●
		++s_counter;	 //カウントアップ

		//現在の角速度を取得
		angleNow = g_anglerVelocity;

		//初回に限り、角速度の差分に大きな値が出る可能性があるため、差分が出ないように値を補正する
		if(angleBef == 0){
	        angleBef = angleNow;	//1回前の角速度に現在の角速度を設定
		}

		//現在の角速度と1回前の角速度を比較し差分が判定値●以上ある場合、シーソーの傾斜が切り替わったと判断し次の状態へ遷移する
		//1.遷移先にSEESAW_04を指定
		//2.カウンタを初期化
		//3.尻尾の角度を坂を下るための角度●に調整する
		//4.衝撃検知用の1回前の角速度（angleBef）を初期化する
		if(abs(angleNow - angleBef) >= 10 && ++s_counter > PERIOD_SEESAW*2){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():Changemode,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			s_mode = SEESAW_04;	//SEESAW_04へ遷移
			s_counter = 0;		//カウンタ
			s_angle = 30;		//尻尾の角度●
			angleBef = 0;		//衝撃検知用の1回前の角速度
		}

		//検証用ログ出力（角速度差分）
        _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber:★ angleABS = %d", clock->now() ,abs(angleNow - angleBef) ));

        //1回前の角速度に現在の角速度を設定
        angleBef = angleNow;

		break;

	//■□■SEESAW_04 シーソーを下る：シーソーの下り坂を一気に滑り落ちる■□■
	case SEESAW_04:
		//スピードを設定
		pwm_L = 50;	//左輪のパワー●
		pwm_R = 50;	//右輪のパワー●
		++s_counter; //カウントアップ

		//現在の角速度を取得
		angleNow = g_anglerVelocity;

		//初回に限り、角速度の差分に大きな値が出る可能性があるため、差分が出ないように値を補正する
		if(angleBef == 0){
	        angleBef = angleNow;	//1回前の角速度に現在の角速度を設定
		}

		//現在の角速度と1回前の角速度を比較し差分が判定値以上ある場合、シーソーを下ったと判断し次の状態へ遷移する
		//1.遷移先にSEESAW_05を指定
		//2.カウンタを初期化
		//3.尻尾の角度を坂を下るための角度●に調整する
		//4.衝撃検知用の1回前の角速度（angleBef）を初期化する
		if(abs(angleNow - angleBef) >= 20 && ++s_counter > PERIOD_SEESAW*2){
			_debug(syslog(LOG_NOTICE, "%08u, ■■■():Changemode,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));
			s_mode = SEESAW_05;	//SEESAW_04へ遷移
			s_counter = 0;		//カウンタ
			s_angle = 80;		//尻尾の角度●
			angleBef = 0;		//衝撃検知用の1回前の角速度
		}

		//検証用ログ出力（角速度差分）
        _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber:★ angleABS = %d", clock->now() ,abs(angleNow - angleBef) ));

        //1回前の角速度に現在の角速度を設定
        angleBef = angleNow;

		break;

	//■□■SEESAW_05 シーソーを下る：シーソーから落ちた後、車庫までゆっくり進む■□■
	case SEESAW_05:
		//スピードを設定
		pwm_L = 5;	//左輪のパワー●
		pwm_R = 5;	//右輪のパワー●

		//指定の秒数●を経過したら処理を終了する
		if ( ++s_counter > PERIOD_SEESAW*5 ){
		//いまだけテスト用に最初に戻す
			s_mode = SEESAW_00;	//SEESAW_00へ遷移
			s_counter = 0;		//カウンタ
		}

		//処理の終了
		_debug(syslog(LOG_NOTICE, "%08u, ■■■():End of Service,CASE NO = %d, tailAngle = %d", clock->now(), s_mode, s_angle));

		break;
	}


	if(abs(angleNow - angleBef) >= 100){
        _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber:★ angleABS = %d", clock->now() ,abs(angleNow - angleBef) ));
    	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
    					pwm_L = 0;
    					pwm_R = 0;
    	}
	}

	// 左右モーターと尻尾モーターへ値を渡す
	leftMotor->setPWM(pwm_L);
	rightMotor->setPWM(pwm_R);
	//controlTail(s_angle);

	// ログを PERIOD_TRAVE_MSG ms で出力する
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		if (collisiontime == 1){
			s_mode = SEESAW_02;
		}
	}

	// 終了ログ出力
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		_debug(syslog(LOG_NOTICE, "%08u, ■■■SeesawCrimber::operate():終了", clock->now()));
	}

}

int SeesawCrimber::readLine( FILE* file, char* dst, size_t len ){
    int c = 0;
    unsigned int i = 0;

    while ((c = fgetc(file)) != EOF) {
      if (c < 0) {
        return c;        // error handling
      }
      if ( i + 1 == len ) {  // reached end of buffer
        dst[i] = '\0';
        break;
      }
      if (c == '\n') {   // reached end of line
        dst[i] = '\0';
        break;
      }
      dst[i] = (char)c;
      i++;
    }
    return i;
}

void SeesawCrimber::readPropFile( const char* filename ){

	FILE* prop_file = NULL;
	prop_file = fopen( filename, "r" );
	if( prop_file==NULL){
		_debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber readProfile() file not found", clock->now()));
		return;
	}


	memset( &props, 0, sizeof(struct property) * NUM_PROPS );

	int retval = 0;
	int i = 0;
	char buf[256];
	memset( &buf, 0, sizeof(buf) );

	while( ( retval = readLine(prop_file, buf, 256)) > 0 ){
		char* comptr = strstr( buf, "," );
		if( i < NUM_PROPS && NULL != comptr ){
			*comptr = '\0';
			strcpy( props[i].name, buf );
			props[i].value = atoi(comptr+1);
			i++;
		}else
			break;
	}
	fclose( prop_file );

}

int SeesawCrimber::getProp( const char* propname ){
	if( props == NULL ){
		return 0;
	}

	for ( int i = 0; i < NUM_PROPS; i++ ){
		if( strcmp( propname, props[i].name ) == 0 ){
			return props[i].value;
		}
	}
	return 0;
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
