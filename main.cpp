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
void send_leg_info(enum CANID::From from, short dist, unsigned char state, unsigned char kouden);
void CANrcv();


int main(){
	//歩行に必要な情報
	float walk_period = 1;
	float walk_duty = 0.5;
	float walk_speed = 0;
	float walk_direction = 0;
	float walk_pitch = 0; //radに変換して格納すること
	//歩いた結果の情報
	float walk_dist_right = 0;
	float walk_dist_left = 0;
	float walk_dist_rear = 0;
	unsigned int kouden_SandDuneRear_max = 0;
	union can_LegState state_rear = {};
	state_rear.right = state_rear.left = Stay;
	union can_Kouden kouden_rear = {};

	int mrmode = MRmode.get_now();

	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();//センサー・モーター初期化
	setLegs();//不変的設定

	send_leg_info(CANID::FromRear,	0, state_rear.byte[0], 0);

	autoInit();//自動キャリブレーション

	set_limits();

	walk_duty = 0.5;
	while(1){
		AdjustCycle(1000);

		MRmode.update();
		set_limits();

		kouden_SandDuneRear.sensing();
		mrmode = (int)MRmode.get_now();

		//基本:脚下げ時に光電センサの値が閾値以下(カウンタ最大値の一定以下?)であれば一番下まで下げる
		//段差開始:Down時[flag==0 かつ 光電センサのカウンタ > ## かつ walk_on_dune==0] であれば、flagを1にする
		//段差終了:Down時[flag==1 かつ 光電センサのカウンタ < カウンタの最大値-??] であれば、flagを0にする
		//乗り上げ後、指定の歩数以上歩いたら強制的に一番下まで下げる(脚下げ時のみ判断)
		//trigger_sanddune(kouden.get_counter(0), cnt_max, ##, cnt_max-??, max_on_dune, area);
		RR.trigger_sanddune(kouden_SandDuneRear.get_counter(0), kouden_SandDuneRear_max,
				300,//250,
				uint_cut(kouden_SandDuneRear_max,
						270//230
						),
				4, MRMode::SandDuneRear);
		RL.trigger_sanddune(kouden_SandDuneRear.get_counter(0), kouden_SandDuneRear_max,
				300,//250,
				uint_cut(kouden_SandDuneRear_max,
						270//230
						),
				4, MRMode::SandDuneRear);

		if(mrmode==MRMode::SandDuneFront || mrmode==MRMode::SandDuneRear){
			RR.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			RL.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			if(mrmode==MRMode::SandDuneRear){
				//最大値更新
				kouden_SandDuneRear_max = max_uint(kouden_SandDuneRear_max, kouden_SandDuneRear.get_counter(0));
				//段差に乗っていない かつ 段差上を歩いた回数(各脚)>0 -> モード切替
				if(RR.get_count_walk_on_dune()>0 && RL.get_count_walk_on_dune()>0){
					if(!RR.is_on_dune() && !RL.is_on_dune()){
						if(RR.get_y()>200 && RL.get_y()>200) //ちゃんと下げきってからモード変更
							MRmode.request_to_change_area(MRMode::ReadyForTussock, CANID::FromRear);
					}
				}
			}
			else kouden_SandDuneRear.reset_counter();//絶対に足上げない
		}
		else if(mrmode==MRMode::Tussock){
			RR.trigger_tussock(
//					(int)can_receiver.get_leg_up_RR()
					1
					);
			RL.trigger_tussock(
//					(int)can_receiver.get_leg_up_RL()
					1
					);
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
		walk_direction =  - ((float)can_receiver.get_direction()) * M_PI / 180.0; //変換
		walk_pitch = ((float)can_receiver.get_pitch()) * M_PI / 180.0;

		//ちゃんと補正したかったが一旦断念
//		if(mrmode==MRMode::ReadyForTussock)
//			walk_direction = adjust_walk_direction(walk_direction);
		//坂道で斜めに歩く問題を解決するために2倍に増幅
		if(MRMode::StartClimb1<=mrmode && mrmode<=MRMode::UukhaiZone)
			walk_direction *= 3.0;

		//period, duty計算
		set_cycle(&walk_period, &walk_duty);
		RR.set_period_duty(walk_period, walk_duty);
		RL.set_period_duty(walk_period, walk_duty);

		//脚固定系座標での目標位置計算+動作
		RR.walk(walk_speed, walk_direction);
		moveLeg(&RRf, &RRr, RR.get_x(), RR.get_y());
		RL.walk(walk_speed, walk_direction);
		moveLeg(&RLf, &RLr, RL.get_x(), RL.get_y());

		//歩行量計算+送信
		walk_dist_right += RR.get_x_distance_move();
		walk_dist_left += RL.get_x_distance_move();

		walk_dist_rear = (walk_dist_right + walk_dist_left) / 2.0;
		kouden_rear.sand_dune = (unsigned char)kouden_SandDuneRear.read();
		state_rear.right = RR.get_mode();
		state_rear.left = RL.get_mode();
		send_leg_info(CANID::FromRear, (short)walk_dist_rear, state_rear.byte[0], kouden_rear.byte[0]);

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
//	*duty = 0.5;
	int mrmode = (int)MRmode.get_now();
	switch(mrmode){
	case MRMode::GobiArea:
		*period = 320.0/350.0;
		break;

	case MRMode::SandDuneFront:
		*period = 1.2;
		break;
	case MRMode::SandDuneRear:
		*period = 1.2;
		break;

	case MRMode::ReadyForTussock:
		*period = 200.0/240.0;
		break;
	case MRMode::Tussock:
		*period = 1;
		break;

	case MRMode::Start2:
		*period = 1;
		break;
	case MRMode::StartClimb1:
//		*period = 0.88;
		*period = 240.0/270.0;
		break;
	case MRMode::StartClimb2:
//		*period = 0.88;
		*period = 240.0/270.0;
		break;
	case MRMode::MountainArea:
//		*period = 0.88;
		*period = 240.0/270.0;
		break;
	case MRMode::UukhaiZone:
//		*period = 0.88;
		*period = 240.0/270.0;
		break;
	}
}


void send_leg_info(enum CANID::From from, short dist, unsigned char state, unsigned char kouden){
	can_sender.send_leg_info(CANID::generate(from, CANID::ToController, CANID::LegInfo), dist, state, kouden);
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
