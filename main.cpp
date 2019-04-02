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
void send_move_position(float dist, unsigned char kouden_sanddune, enum CANID::DataType type, enum CANID::From from);
void CANrcv();


int main(){
	float walk_period = 1;
	float walk_duty = 0.5;
	float walk_speed = 0;
	float walk_direction = 0;
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

	send_move_position(0, 0, CANID::MovePositionRear, CANID::FromRear);

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

		kouden_SandDuneRear.sensing();
		mrmode = (int)MRmode.get_now();

		if(mrmode==MRMode::SandDuneFront || mrmode==MRMode::SandDuneRear){
			if(mrmode==MRMode::SandDuneFront)kouden_SandDuneRear.reset_counter();//絶対に足上げない
			RR.trigger_sanddune(kouden_SandDuneRear.get_counter(60), 3);
			RL.trigger_sanddune(kouden_SandDuneRear.get_counter(60), 3);
			RR.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			RL.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
		}
		else if(mrmode==MRMode::Tussock){
			RR.trigger_tussock((int)can_receiver.get_leg_up_RR());
			RL.trigger_tussock((int)can_receiver.get_leg_up_RL());
			RR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}
		else if(MRMode::StartClimb1<=mrmode && mrmode<=MRMode::UukhaiZone){
			RR.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
		}
		else{
			if(mrmode!=MRMode::ReadyForTussock) //謎のバグ対策(4/1)
				kouden_SandDuneRear.reset_counter();
			RR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			RL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}

		walk_speed = can_receiver.get_speed();
		walk_direction = can_receiver.get_direction();

		//脚固定系座標での目標位置計算
		RR.walk(walk_speed, walk_direction);
		moveLeg(&RRf, &RRr, RR.get_x(), RR.get_y());
		RL.walk(walk_speed, walk_direction);
		moveLeg(&RLf, &RLr, RL.get_x(), RL.get_y());

		//歩行量計算+送信
		walk_dist_right += RR.get_x_distance_move();
		walk_dist_left += RL.get_x_distance_move();
		walk_dist_rear = (walk_dist_right + walk_dist_left) / 2.0;
		send_move_position(walk_dist_rear, (unsigned char)kouden_SandDuneRear.read(), CANID::MovePositionRear, CANID::FromRear);

		//DEBUG
		if(pc.readable()){
			pc.printf("[Rear]");
			pc.printf("mode:%d  ", MRmode.get_now());
			pc.printf("kouden:%d  ", kouden_SandDuneRear.get_counter());
//			pc.printf("timer:%1.4f  ", timer_RR.read());
//			pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_speed(), can_receiver.get_direction());
//			pc.printf("x:%3.3f  y:%3.3f  ", RR.get_x(), RR.get_y());
//			pc.printf("enc:%3.2f  ", enc_RRf.getAngle());
//			pc.printf("angle:%3.2f  duty:%1.4f  ", RRf.get_angle(), RRf.get_duty());
//			pc.printf("[%d][%d][%d][%d] ", sw_RRf.read(), sw_RRr.read(), sw_RLf.read(), sw_RLr.read());

			pc.printf("spd:%f  dir:%f  ", walk_speed, walk_direction);
//			pc.printf("pos[%f][%1d]  ", walk_dist_rear, kouden_SandDuneRear.read());

			pc.printf("RR:");
//			pc.printf("enc:%3.2f  ", enc_RRf.getAngle());
			orbit_log(&RR, &fw_RR);
			pc.printf("RL:");
			orbit_log(&RL, &fw_RL);
			pc.printf("\r\n");
		}
	}
}


void set_cycle(float *period, float *duty){
	*duty = 0.5;
	int mrmode = (int)MRmode.get_now();
	switch(mrmode){
	case MRMode::GobiArea:
		//*period = 1;
		*period = 200.0/240.0;
		break;
	case MRMode::SandDuneFront:
//		*period = 2;
		*period = 1.6;
		break;
	case MRMode::SandDuneRear:
//		*period = 2;
		*period = 1.6;
		break;
	case MRMode::ReadyForTussock:
		//*period = 1;
		*period = 200.0/240.0;
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
	default:
		*period = 1;
		break;
	}
}


void send_move_position(float dist, unsigned char kouden_sanddune, enum CANID::DataType type, enum CANID::From from){
	can_sender.send_move_position(CANID::generate(from, CANID::ToController, type), dist, kouden_sanddune);
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		//タイマーリセット
		if(CANID::is_type(id, CANID::TimerReset)){
			timer_RR.reset();
			timer_RL.reset();
			return;
		}
		//歩行パラメータ取得
		if(CANID::is_to(id, CANID::ToSlaveAll)){
			can_receiver.receive(rcvMsg);
			return;
		}
	}
}
