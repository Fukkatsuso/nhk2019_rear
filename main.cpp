#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "LegFunctions.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"


LocalFileSystem local("local");//PIDゲイン調整に使用

Timer timer_PID;

ClockTimer timer_RR;
ClockTimer timer_RL;
SingleLeg RRf(Front, Right, BASE_X, 0);
SingleLeg RRr(Rear, Right, -BASE_X, 0);
ParallelLeg RR(Rear, Right, 200, 200);
SingleLeg RLf(Front, Left, BASE_X, 0);
SingleLeg RLr(Rear, Left, -BASE_X, 0);
ParallelLeg RL(Rear, Left, -200, 200);

ForwardKinematics fw_RR(BASE_X, 0, &enc_RRf, -BASE_X, 0, &enc_RRr);
ForwardKinematics fw_RL(BASE_X, 0, &enc_RLf, -BASE_X, 0, &enc_RLr);

CANMessage rcvMsg;
CANReceiver can_receiver(&can);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認

void set_cycle(float *period, float *duty);
void CANrcv();


int main(){
	float walk_period = 1;//2;
	float walk_duty = 0.55;//0.80;
	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();//センサー・モーター初期化
	setLegs();//不変的設定

	autoInit();//自動キャリブレーション

	set_limits();

	while(1){
		AdjustCycle(1000);

		MRmode.update();
		set_limits();

		set_cycle(&walk_period, &walk_duty);

		RR.set_period(walk_period);
		RR.set_duty(walk_duty);
		RL.set_period(walk_period);
		RL.set_duty(walk_duty);

		//脚固定系座標での目標位置計算
		if(MRmode.get_now()==MRMode::SandDuneFront || MRmode.get_now()==MRMode::SandDuneRear){
			if((int)can_receiver.get_data(CANID::LegUp)&0x2)RR.set_y_initial(280-100);
			if((int)can_receiver.get_data(CANID::LegUp)&0x8)RL.set_y_initial(280-100);
		}

		if(MRMode::StartClimb1<=MRmode.get_now() && MRmode.get_now()<=MRMode::MountainArea){
			RR.walk_stable(can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction), 0.25);
			moveLeg(&RRf, &RRr, RR.get_x(), RR.get_y());
			RL.walk_stable(can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction), 0.25);
			moveLeg(&RLf, &RLr, RL.get_x(), RL.get_y());
		}
		else{
			RR.walk();
			moveLeg(&RRf, &RRr, RR.get_x(), RR.get_y());
			RL.walk();
			moveLeg(&RLf, &RLr, RL.get_x(), RL.get_y());
		}

		//DEBUG
		if(pc.readable()){
			pc.printf("mode:%d  ", RR.get_mode());
			pc.printf("timer:%1.4f  ", timer_RR.read());
//			pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
//			pc.printf("x:%3.3f  y:%3.3f  ", RR.get_x(), RR.get_y());
//			pc.printf("enc:%3.2f  ", enc_RRf.getAngle());
//			pc.printf("angle:%3.2f  duty:%1.4f  ", RRf.get_angle(), RRf.get_duty());
//			pc.printf("[%d][%d][%d][%d] ", sw_RRf.read(), sw_RRr.read(), sw_RLf.read(), sw_RLr.read());

			orbit_log(&RR, &fw_RR);
			pc.printf("\r\n");
		}
	}
}


void set_cycle(float *period, float *duty){
	switch((int)MRmode.get_now()){
	case MRMode::GobiArea:
		*period = 2;//1.5;//1;
		*duty = 0.55;
		break;
	case MRMode::SandDuneFront:
		*period = 1.6;//5;//1;
		*duty = 0.55;//0.8;//0.55;
		break;
	case MRMode::SandDuneRear:
		*period = 1.6;//5;//1;
		*duty = 0.55;//0.8;//0.55;
		break;
	case MRMode::Tussock1:
		*period = 1;
		*duty = 0.55;
		break;
	case MRMode::Start2:
		*period = 2;
		*duty = 0.8;
		break;
	case MRMode::StartClimb1:
		*period = 4;
		*duty = 0.55;
		break;
	case MRMode::StartClimb2:
		*period = 4;
		*duty = 0.55;
	}
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		if(!CANID_is_from(id, CANID::FromMaster))return;
		if(CANID_is_type(id, CANID::TimerReset)){//タイマーリセット
			timer_RR.reset();
			timer_RL.reset();
			return;
		}
		else if(CANID_is_to(id, CANID::ToSlaveAll)){
			//歩行パラメータ取得
			can_receiver.receive(id, rcvMsg.data);
		}
	}
}
