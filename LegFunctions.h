/*
 * LegFunctions.h
 *
 *  Created on: 2019/03/13
 *      Author: mutsuro
 */

#ifndef LEGFUNCTIONS_H_
#define LEGFUNCTIONS_H_

#include "mbed.h"
#include "Pins.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"


struct InitLegInfo{
	bool enc_reset;
	bool finish_init;
	float angle_target;
	float x_target;
	float y_target;
};


void setLegs();
void set_limits();
void moveLeg(SingleLeg *front, SingleLeg *rear, float x, float y);

void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
			  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw);
void autoInit();
void orbit_log(ParallelLeg *invLeg, ForwardKinematics *fwLeg);


extern Serial pc;

extern LocalFileSystem local;//PIDゲイン調整に使用

extern Timer timer_PID;

extern ClockTimer timer_RR;
extern ClockTimer timer_RL;
extern SingleLeg RRf;
extern SingleLeg RRr;
extern ParallelLeg RR;
extern SingleLeg RLf;
extern SingleLeg RLr;
extern ParallelLeg RL;

extern ForwardKinematics fw_RR;
extern ForwardKinematics fw_RL;

extern CANMessage rcvMsg;
extern CANReceiver can_receiver;
extern MRMode MRmode;//実行の度に要確認


#endif /* LEGFUNCTIONS_H_ */
