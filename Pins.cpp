/*
 * Pins2.cpp
 *
 *  Created on: 2019/02
 *      Author: mutsuro
 */

#include "Pins.h"

/*----------------------
 -----機能選択するピン-----
 ----------------------*/
CAN can(p30, p29);


/********************
 * 		enc			*
 ********************/
SingleLegQEI enc_RRf(p25, p26);
SingleLegQEI enc_RRr(p14, p13);
SingleLegQEI enc_RLf(p27, p28);
SingleLegQEI enc_RLr(p9, p10);


/********************
 * 		スイッチ		*
 ********************/
InitSwitch sw_RRf(p8, 1);
InitSwitch sw_RRr(p7, 1);
InitSwitch sw_RLf(p5, 1);
InitSwitch sw_RLr(p6, 1);


/********************
 * 		DCモータ		*
 ********************/
PwmOut motor_RRf(p25);
PwmOut motor_RRr(p26);
PwmOut motor_RLf(p23);
PwmOut motor_RLr(p24);

/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
Serial pc(USBTX, USBRX);
DigitalOut led0(LED1);
DigitalOut led1(LED2);
DigitalOut led2(LED3);
DigitalOut led3(LED4);
Timer AdCycle;



/*
 * 関数名	AdjustCycle
 *
 * 用途		マイコンの動作周期を調整する
 *
 * 引数		int t_us:目的の動作周期(us)
 *
 * 戻り値		なし
 *
 * 備考		関数実行時、前の実行時からt_us経っていない場合、t_us経つまで待つ
 * 			すでにt_us経っている場合、led3を点灯する
 */
void AdjustCycle(int t_us){
    if(AdCycle.read_us() == 0) AdCycle.start();

    if(AdCycle.read_us()>t_us){
    	led3=1;
//    	pc.printf("AdCycle=%dus\r\n",AdCycle.read_us());
    }
    else {
    	led3=0;
//    	pc.printf("AdCycle=%dus\r\n",AdCycle.read_us());
    }
    while(AdCycle.read_us()<=t_us);
    AdCycle.reset();
}


void initParts(){
	sw_RRf.mode(PullDown);
	sw_RRr.mode(PullDown);
	sw_RLf.mode(PullDown);
	sw_RLr.mode(PullDown);
	enc_RRf.reset();
	enc_RRr.reset();
	enc_RLf.reset();
	enc_RLr.reset();
	motor_RRf.period_us(50);
	motor_RRr.period_us(50);
	motor_RLf.period_us(50);
	motor_RLr.period_us(50);
}
