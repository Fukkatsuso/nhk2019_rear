/*
 * MRMode.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_MRMODE_H_
#define WALK_MRMODE_H_

#include "mbed.h"
#include "CANs/CANReceiver.h"


//normal
#define DUTY_MAX (0.8)///0.8
#define DUTY_MIN (0.2)//0.2

/*stay
#define DUTY_MAX (0.65)//(0.8)
#define DUTY_MIN (0.35)//(0.2)
*/


enum LegPosition{
	Front = 1,
	Rear = -1,
	Right = 1,
	Left = -1
};

enum LegMode{
	Move = 1,
	Up = 2,
	Down = 3,
	Stay = 4
};

struct Limits{
	struct{
		float max;
		float min;
	}x, y, angle, duty;
};

struct Orbits{
	float gradient; //フィールド勾配
	float init_x; //足先の初期位置(x)
	float init_y; //(y)
	float height; //足先を上げるときの最大高さ
};


class MRMode
{
public:
	enum Area{
		WaitGobiUrtuu = 0,//待機
		GetGerege,//ゲルゲ受け取り検知
		PrepareWalking,
		Start1,//歩行開始
		GobiArea,//直進
		SandDune,//段差
		ReadyForTussock,//ここに何か入れるべき
		Tussock1,//紐1
		Tussock2,//紐2
		Finish1,//到着
		WaitMountainUrtuu,//待機
		GetSign,//非接触の合図
		Start2,//歩行開始
		MountainArea,//登山
		UukhaiZone,//ウーハイゾーン
		Uukhai,//ウーハイ
		Finish2,//終了
		Area_end,
	};
	enum Reference{
		Initial,
		Prev,
		Now,
		Next,
		Reference_end
	};

	MRMode(CANReceiver *rcv, enum Area init_area, bool operate);
	void update();
	bool is_switched();

	Area get_area(enum Reference ref);
	Limits *get_limits(enum Area area);
	Orbits *get_orbits (enum Area area);

private:
	CANReceiver *can_receiver;
	Area area[MRMode::Reference_end];
	Area roop_prev, roop_now;//前回と今回のループでのモード
	struct{
		bool operate;//手動
		bool switched;//モードが切り替わった直後
	}flag;
};

extern Limits limits[MRMode::Area_end];
extern Orbits orbits[MRMode::Area_end];

/*
 * example:
 * MRMode mode(&cancommand, MRMode::WaitGobiUrtuu, false);
 * FR.set_x_lim(mode.get_limits(mode.get_area(Now))->x.max, mode.get_limits(mode.get_area(Now))->x.min);
 */


#endif /* WALK_MRMODE_H_ */
