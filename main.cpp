#include "mbed.h"
#include "Pins.h"
#include "functions.h"
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



void setLegs();
void set_limits();
void CANrcv();

struct InitLegInfo{
	bool enc_reset;
	bool finish_init;
	float angle_target;
	float x_target;
	float y_target;
};
void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
			  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw);
void autoInit();

CANMessage rcvMsg;
CANReceiver can_receiver(&can);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認


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
//		if(MRmode.is_switched())
			set_limits();

		if((int)can_receiver.get_data(CANID::LegUp)&0x2)RR.set_y_initial(280-110);
		if((int)can_receiver.get_data(CANID::LegUp)&0x8)RL.set_y_initial(280-110);

//		switch((int)MRmode.get_area(MRMode::Now)){
//		case MRMode::GobiArea:
//			walk_period = 1;
//			walk_duty = 0.55;
//			break;
//		case MRMode::SandDune:
//			walk_period = 2;
//			walk_duty = 0.8;
//			break;
//		case MRMode::Tussock1:
//			walk_period = 2;
//			walk_duty = 0.8;
//			break;
//		}

		RR.set_period(walk_period);
		RR.set_duty(walk_duty);
		RL.set_period(walk_period);
		RL.set_duty(walk_duty);

		//脚固定系座標での目標位置計算
		RR.walk();
		RL.walk();

		//駆動
		RRf.move_to(RR.get_x(), RR.get_y());
		RRr.move_to(RR.get_x(), RR.get_y());
		RLf.move_to(RL.get_x(), RL.get_y());
		RLr.move_to(RL.get_x(), RL.get_y());

		//DEBUG
		pc.printf("mode:%d  ", RR.get_mode());
		pc.printf("timer:%1.4f  ", timer_RR.read());
		pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
		pc.printf("x:%3.3f  y:%3.3f  ", RR.get_x(), RR.get_y());
		pc.printf("enc:%3.2f  ", enc_RRf.getAngle());
		pc.printf("angle:%3.2f  duty:%1.4f  ", RRf.get_angle(), RRf.get_duty());
		pc.printf("[%d][%d][%d][%d] ", sw_RRf.read(), sw_RRr.read(), sw_RLf.read(), sw_RLr.read());

		pc.printf("\r\n");
	}
}


void setLegs(){
	RRf.unitize(&motor_RRf, &enc_RRf, &sw_RRf, &timer_PID);
	RRr.unitize(&motor_RRr, &enc_RRr, &sw_RRr, &timer_PID);
	RLf.unitize(&motor_RLf, &enc_RLf, &sw_RLf, &timer_PID);
	RLr.unitize(&motor_RLr, &enc_RLr, &sw_RLr, &timer_PID);
	RRf.set_PID_from_file("/local/PID_RRf.txt");
	RRr.set_PID_from_file("/local/PID_RRr.txt");
	RLf.set_PID_from_file("/local/PID_RLf.txt");
	RLr.set_PID_from_file("/local/PID_RLr.txt");

	RRf.set_dependencies(&MRmode);
	RRr.set_dependencies(&MRmode);
	RLf.set_dependencies(&MRmode);
	RLr.set_dependencies(&MRmode);
	RR.set_dependencies(&timer_RR, &MRmode, &can_receiver);
	RL.set_dependencies(&timer_RL, &MRmode, &can_receiver);
}


void set_limits(){
	RRf.set_limits();
	RRr.set_limits();
	RLf.set_limits();
	RLr.set_limits();
	RR.set_limits();
	RL.set_limits();
	RR.set_orbits();
	RL.set_orbits();
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


void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
		  	  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw){
	//ゆっくりスイッチに近づける
	if(!info_f->enc_reset){
		info_f->angle_target = leg_f->get_enc() + 0.2;//200[roop/sec], 20[degree/sec] -> 0.1[degree/roop]
		info_r->angle_target = leg_r->get_enc() - 0.1;
		if(leg_f->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_f->reset_enc();
			info_f->enc_reset = true;
		}
	}
	else if(!info_r->enc_reset){
		info_f->angle_target = 30;
		info_r->angle_target = leg_r->get_enc() + 0.2;
		if(leg_r->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_r->reset_enc();
			info_r->enc_reset = true;
		}
	}
	else{//enc両方リセット完了
		if((fabs(0.0f - fw->get_x()) < 1.0) && (fabs(250.0f - fw->get_y()) < 1.0))
			info_f->finish_init = info_r->finish_init = true;
		fw->estimate();
		info_f->x_target = info_r->x_target = 0.0f;
		info_f->y_target = info_r->y_target = 250.0f;
	}
	//駆動
	if(!info_f->enc_reset){
		leg_f->set_duty_limit(0.55, 0.45);
		leg_r->set_duty_limit(0.55, 0.45);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else if(!info_r->enc_reset){
		leg_f->set_duty_limit(0.575, 0.4);
		leg_r->set_duty_limit(0.575, 0.4);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else{
		leg_f->set_duty_limit(0.6, 0.4);
		leg_r->set_duty_limit(0.6, 0.4);
		leg_f->move_to(info_f->x_target, info_f->y_target);
		leg_r->move_to(info_r->x_target, info_r->y_target);
	}
}

void autoInit(){
			//	右前,右後,左前,左後
	InitLegInfo Rf, Rr, Lf, Lr;
	Rf.enc_reset = Rr.enc_reset = Lf.enc_reset = Lr.enc_reset = false;
	Rf.finish_init = Rr.finish_init = Lf.finish_init = Lr.finish_init = false;
	while(!(Rf.finish_init && Rr.finish_init && Lf.finish_init && Lr.finish_init)){
		AdjustCycle(5000);
		initLegs(&RRf, &Rf, &RRr, &Rr, &fw_RR);
		initLegs(&RLf, &Lf, &RLr, &Lr, &fw_RL);
		pc.printf("enc");
		pc.printf("[%3.2f][%3.2f]", enc_RRf.getAngle(), enc_RRr.getAngle());
		pc.printf("[%3.2f][%3.2f]", enc_RLf.getAngle(), enc_RLr.getAngle());
		pc.printf("  target");
		if(!Rf.enc_reset || !Rr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", Rf.angle_target, Rr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", Rf.x_target, Rf.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_RR.get_x(), fw_RR.get_y());
		}
		if(!Lf.enc_reset || !Lr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", Lf.angle_target, Lr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", Lf.x_target, Lr.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_RL.get_x(), fw_RL.get_y());
		}
//		pc.printf("  sw");
//		pc.printf("[%d][%d]", sw_RRf.read(), sw_RRr.read());
//		pc.printf("[%d][%d]", sw_RLf.read(), sw_RLr.read());
		pc.printf("  duty");
		pc.printf("[%1.3f][%1.3f]", RRf.get_duty(), RRr.get_duty());
		pc.printf("[%1.3f][%1.3f]", RLf.get_duty(), RLr.get_duty());
		pc.printf("\r\n");

		if(pc.readable())if(pc.getc()=="s")break;//"s"を押したら強制終了
	}
}
