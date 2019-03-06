#include "mbed.h"
#include "Pins.h"
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


void initLegs();
void set_limits();
void CANrcv();

CANMessage rcvMsg;
CANReceiver can_receiver(&can);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認


int main(){
	float walk_period = 2;
	float walk_duty = 0.80;
	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();
	initLegs();

	while(1){
		AdjustCycle(1000);

		MRmode.update();
		if(MRmode.is_switched())set_limits();

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
		pc.printf("x:%3.3f  y:%3.3f  ", RL.get_x(), RL.get_y());
		pc.printf("enc:%3.2f  ", enc_RLf.getAngle());
		pc.printf("angle:%3.2f  duty:%1.4f  ", RLf.get_angle(), RLf.get_duty());

		pc.printf("\r\n");
	}
}


void initLegs(){
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
		if(CANID_is_from(id, CANID::FromMaster)){
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
}
