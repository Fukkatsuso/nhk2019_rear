/*
 * init2.h
 *
 *  Created on: 2018/09
 *      Author: mutsuro
 */
#ifndef PINS_H_
#define PINS_H_

#include "mbed.h"
#include "QEI/SingleLegQEI.h"
#include "InitSwitch.h"

#define CYCLE 5000 //動作周期(us)


/*----------------------
 -----機能選択するピン-----
 ----------------------*/
extern CAN can;


/********************
 * 		enc			*
 ********************/
extern SingleLegQEI enc_RRf;
extern SingleLegQEI enc_RRr;
extern SingleLegQEI enc_RLf;
extern SingleLegQEI enc_RLr;


/********************
 * 		スイッチ		*
 ********************/
extern InitSwitch sw_RRf;
extern InitSwitch sw_RRr;
extern InitSwitch sw_RLf;
extern InitSwitch sw_RLr;


/********************
 * 		DCモータ		*
 ********************/
extern PwmOut motor_RRf;
extern PwmOut motor_RRr;
extern PwmOut motor_RLf;
extern PwmOut motor_RLr;


/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
extern Serial pc;
extern DigitalOut led0;
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern Timer AdCycle; //AdjustCycleで使うタイマ


/*----------------------
 ---------関数---------
 ----------------------*/
extern void AdjustCycle(int t_us);
extern void initParts();


#endif /* PINS_H_ */

