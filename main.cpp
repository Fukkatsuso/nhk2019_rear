#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "LegFunctions.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/CANs/CANSender.h"
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
ParallelLeg RR(Rear, Right, 225, -200);
SingleLeg RLf(Front, Left, BASE_X, 0);
SingleLeg RLr(Rear, Left, -BASE_X, 0);
ParallelLeg RL(Rear, Left, -225, -200);

ForwardKinematics fw_RR(BASE_X, 0, &enc_RRf, -BASE_X, 0, &enc_RRr);
ForwardKinematics fw_RL(BASE_X, 0, &enc_RLf, -BASE_X, 0, &enc_RLr);

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
CANSender can_sender(&can);

MRMode MRmode(&can_receiver, &can_sender, MRMode::GobiArea, true);//実行の度に要確認

void set_cycle(float *period, float *duty);
void send_movedist(float dist, enum CANID::DataType type, enum CANID::From from);
void CANrcv();


int main(){
	float walk_period = 1;
	float walk_duty = 0.5;
	float walk_dist_right = 0;
	float walk_dist_left = 0;
	float walk_dist_rear = 0;
	int mrmode = MRmode.get_now();

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

		mrmode = (int)MRmode.get_now();

		if(mrmode==MRMode::SandDuneFront || mrmode==MRMode::SandDuneRear){
			RR.trigger_sanddune(kouden_SandDuneRear.read());
			RL.trigger_sanddune(kouden_SandDuneRear.read());
			RR.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			RL.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			if(mrmode==MRMode::SandDuneRear && MRmode.is_changing_area()){
				RR.set_height(20);
				RL.set_height(20);
			}
		}
		else if(mrmode==MRMode::Tussock){
			RR.trigger_tussock((int)can_receiver.get_data(CANID::LegUp)&0x2);
			RL.trigger_tussock((int)can_receiver.get_data(CANID::LegUp)&0x8);
			RR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}
		else if(MRMode::StartClimb1<=mrmode && mrmode<=MRMode::MountainArea){
			RR.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
		}
		else{
			RR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}

		//脚固定系座標での目標位置計算
		RR.walk();
		moveLeg(&RRf, &RRr, RR.get_x(), RR.get_y());
		RL.walk();
		moveLeg(&RLf, &RLr, RL.get_x(), RL.get_y());

		//歩行量計算+送信
		walk_dist_right += RR.get_x_distance_move();
		walk_dist_left += RL.get_x_distance_move();
		walk_dist_rear = (walk_dist_right + walk_dist_left) / 2.0;
		send_movedist(walk_dist_rear, CANID::MoveDistRear, CANID::FromRear);

		//DEBUG
		if(pc.readable()){
			pc.printf("[Rear]");
//			pc.printf("mode:%d  ", RR.get_mode());
//			pc.printf("kouden:%d  ", kouden_SandDuneRear.read());
//			pc.printf("timer:%1.4f  ", timer_RR.read());
//			pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
//			pc.printf("x:%3.3f  y:%3.3f  ", RR.get_x(), RR.get_y());
//			pc.printf("enc:%3.2f  ", enc_RRf.getAngle());
//			pc.printf("angle:%3.2f  duty:%1.4f  ", RRf.get_angle(), RRf.get_duty());
//			pc.printf("[%d][%d][%d][%d] ", sw_RRf.read(), sw_RRr.read(), sw_RLf.read(), sw_RLr.read());

			pc.printf("dist[%f][%f][%f]  ", walk_dist_right, walk_dist_left, walk_dist_rear);

//			orbit_log(&RR, &fw_RR);
			orbit_log(&RL, &fw_RL);
			pc.printf("\r\n");
		}
	}
}


void set_cycle(float *period, float *duty){
	*duty = 0.5;
	switch((int)MRmode.get_now()){
	case MRMode::GobiArea:
		*period = 1;
		break;
	case MRMode::SandDuneFront:
		*period = 2;
		break;
	case MRMode::SandDuneRear:
		*period = 2;
		break;
	case MRMode::Tussock:
		*period = 1;
		break;
	case MRMode::Start2:
		*period = 1;
		break;
	case MRMode::StartClimb1:
		*period = 1;
		break;
	case MRMode::StartClimb2:
		*period = 1;
		break;
	}
}


void send_movedist(float dist, enum CANID::DataType type, enum CANID::From from){
	can_sender.send(CANID_generate(from, CANID::ToController, type), dist);
//	pc.printf("dist%d:%2.5f  ", type, dist);
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		//歩行パラメータ取得
		if(CANID_is_to(id, CANID::ToSlaveAll)){
			can_receiver.receive(id, rcvMsg.data);
			return;
		}
		//タイマーリセット
		if(CANID_is_type(id, CANID::TimerReset)){
			timer_RR.reset();
			timer_RL.reset();
			return;
		}
	}
}
