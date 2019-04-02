/*
 * LegFunctions.cpp
 *
 *  Created on: 2019/03/13
 *      Author: mutsuro
 */

#include "LegFunctions.h"


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
	RRf.mrmode_update();
	RRr.mrmode_update();
	RLf.mrmode_update();
	RLr.mrmode_update();
	RR.mrmode_update();
	RL.mrmode_update();
	RRf.set_limits();
	RRr.set_limits();
	RLf.set_limits();
	RLr.set_limits();
	RR.set_limits();
	RL.set_limits();
	RR.set_orbits();
	RL.set_orbits();
}


void moveLeg(SingleLeg *front, SingleLeg *rear, float x, float y){
	front->move_to(x, y);
	rear->move_to(x, y);
}


#define INITIAL_SET_X MRmode.get_orbits(MRmode.get_now())->init_x //0
#define INITIAL_SET_Y MRmode.get_orbits(MRmode.get_now())->init_y //240
void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
		  	  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw){
	//ゆっくりスイッチに近づける
	if(!info_f->enc_reset){
		info_f->angle_target = leg_f->get_enc() + 0.25;//200[roop/sec], 20[degree/sec] -> 0.1[degree/roop]
		info_r->angle_target = leg_r->get_enc() - 0.01;
		if(leg_f->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_f->reset_enc();
			info_f->enc_reset = true;
		}
	}
	else if(!info_r->enc_reset){
		info_f->angle_target = 30;
		info_r->angle_target = leg_r->get_enc() + 0.25;
		if(leg_r->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_r->reset_enc();
			info_r->enc_reset = true;
		}
	}
	else{//enc両方リセット完了
		if((fabs(INITIAL_SET_X - fw->get_x()) < 1.0) && (fabs(INITIAL_SET_Y - fw->get_y()) < 1.0))
			info_f->finish_init = info_r->finish_init = true;
		fw->estimate();
		info_f->x_target = info_r->x_target = INITIAL_SET_X;
		info_f->y_target = info_r->y_target = INITIAL_SET_Y;
	}

	//駆動
	if(!info_f->enc_reset){
		leg_f->set_duty_limit(0.625, 0.375);
		leg_r->set_duty_limit(0.51, 0.49);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else if(!info_r->enc_reset){
		leg_f->set_duty_limit(0.575, 0.425);
		leg_r->set_duty_limit(0.6, 0.4);
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
		led0 = 1;
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

		if(pc.readable())if(pc.getc()=='s')break;//"s"を押したら強制終了
	}
	led0 = 0;
}

void orbit_log(ParallelLeg *invLeg, ForwardKinematics *fwLeg){
	fwLeg->estimate();
	pc.printf("%3.4f\t%3.4f\t", invLeg->get_x(), invLeg->get_y());
	pc.printf("%3.4f\t%3.4f\t", fwLeg->get_x(), fwLeg->get_y());
}
