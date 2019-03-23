/*
 * MRMode.cpp
 *
 *  Created on: 2019/02/18
 *      Author: mutsuro
 */


#include "MRMode.h"


#define X_NORMAL_WIDTH 60
#define X_CLIMB_MAX 130
#define X_CLIMB_MIN -180

Limits limits[MRMode::Area_end] =
		{	//----------ParallelLeg----------   ----------SingleLeg----------
			//{{x.max, 			x.min},			  {y.max, 	y.min}, {angle.max, angle.min}, {duty.max, duty.min}}
			  {{X_NORMAL_WIDTH,	-X_NORMAL_WIDTH}, {280, 	200}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//WaitGobiUrtuu
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {280, 	200}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//GetGerege
			  {{X_NORMAL_WIDTH,	-X_NORMAL_WIDTH}, {280, 	200}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//PrepareWalking
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {280, 	200}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//Start1
			  {{90, -90}, {280, 	200}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//GobiArea
			  {{100, -100}, {260, 	120}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//SandDuneFront
			  {{100, -100}, {260, 	120}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//SandDuneRear
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {280, 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//ReadyForTussock
			  {{90, -90}, {280, 	80}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//Tussock
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {280, 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//Finish1
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {250, 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//WaitMountainUrtuu
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, {250, 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//GetSign
			  {{X_CLIMB_MAX, 	X_CLIMB_MIN},  	  {250, 	150}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//Start2
			  {{X_CLIMB_MAX, 	X_CLIMB_MIN},  	  {250,   	150}, 	{110, 		-20}, 		{0.75, 	   0.25}},//StartClimb1
			  {{X_CLIMB_MAX, 	X_CLIMB_MIN},  	  {250,   	100}, 	{110, 		-20},		{0.75, 	   0.25}},//StartClimb2//Rear開始
			  {{X_CLIMB_MAX, 	X_CLIMB_MIN},	  {250, 	100}, 	{110, 		-20}, 		{0.75, 	   0.25}},	//MountainArea
			  {{100, 			-100}, 			  {250, 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//UukhaiZone
			  {{100, 			-100}, 			  {250,	 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}},	//Uukhai
			  {{100,			-100}, 			  {250,	 	150}, 	{110, 		-20}, 		{0.7, 	   0.3}}	//Finish2
		};

#define GRAD_SLOPE 14.9
#define X_CLIMB_INIT -70//-53 // = init_y * tan(gradient)

Orbits orbits[MRMode::Area_end] =
		{		//gradient, 	init_x,			init_y, height
				{0,				0, 				280, 	10},	//WaitGobiUrtuu
				{0, 			0,				280, 	10},	//GetGerege
				{0, 			0,				280, 	10},	//PrepareWalking
				{0, 			0,				280, 	50},	//Start1
				{0, 			0,				260, 	30},	//GobiArea
				{0, 			0,				260,	20},	//SandDuneFront
				{0,				0,				260,	140},	//SandDuneRear
				{0, 			0,				250, 	50},	//ReadyForTussock
				{0, 			-10,			260, 	30},	//Tussock
				{0, 			0,				280, 	100},	//Finish1
				{0, 			0,				200, 	50},	//WaitMountainUrtuu
				{0, 			0,				200, 	50},	//GetSign
				{0, 			0,				200, 	50},	//Start2
				{0, 			0, 				200, 	40},	//StartClimb1
				{GRAD_SLOPE, 	X_CLIMB_INIT,	200, 	80},	//StartClimb2//Rear開始
				{0, 			X_CLIMB_INIT,	200, 	40},	//MountainArea
				{0, 			0,				250, 	80},	//UukhaiZone
				{0, 			0,				250, 	80},	//Uukhai
				{0, 			0,				250, 	50}	//Finish2
		};


MRMode::MRMode(CANReceiver *rcv, enum Area init_area, bool operate=false)
{
	this->can_receiver = rcv;
	area[Now] = area[Initial] = init_area;
	flag.operate = operate;
}


//getする前に必ず実行すること
void MRMode::update()
{
	area[Now] = (Area)(can_receiver->get_area());//今のArea
	roop_prev = roop_now;	roop_now = area[Now];
	flag.switched = (roop_now!=roop_prev);//Area切り替わりの判断
	area[Prev] = (Area)((int)area[Now] - (((int)area[Now]>(int)area[Initial])? 1:0));//1つ前のArea
	area[Next] = (Area)((int)area[Now] + (((int)area[Now]<(int)Finish2)? 1:0));//次のArea（予定）
}

bool MRMode::is_switched()
{
	return flag.switched;
}


MRMode::Area MRMode::get_area(enum Reference ref){
	return area[ref];
}

MRMode::Area MRMode::get_now(){
	return area[Now];
}


Limits* MRMode::get_limits(enum Area area){
	return &limits[area];
}


Orbits* MRMode::get_orbits(enum Area area){
	return &orbits[area];
}
