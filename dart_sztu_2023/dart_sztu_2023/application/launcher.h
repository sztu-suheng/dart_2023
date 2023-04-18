//
// Created by xhuanc on 2021/11/2.
//

#ifndef DEMO1_LAUNCHER_H
#define DEMO1_LAUNCHER_H

#include "can_receive.h"

/******************define******************/
//TODO:后期根据射速限制来修改摩擦轮的最大转速
#define FIRE_SPEED_MAX -4000
#define TRIGGER_CONTINUES_SPEED -3000
#define FIRE_L  0
#define FIRE_R  1
#define TRIGGER 2
#define DEGREE_60_TO_ENCODER -49150.8f
//连发准备计时完成
#define CONTINUES_SHOOT_TIMING_COMPLETE HAL_GetTick()-continue_shoot_time>1500
//摩擦轮转速PID
#define SHOOT_FIRE_L_PID_KP 10
#define SHOOT_FIRE_L_PID_KI 0.f
#define SHOOT_FIRE_L_PID_KD 0.f
#define SHOOT_FIRE_L_PID_MAX_OUT    6000
#define SHOOT_FIRE_L_PID_MAX_IOUT   0

#define SHOOT_FIRE_R_PID_KP 40
#define SHOOT_FIRE_R_PID_KI 0.f
#define SHOOT_FIRE_R_PID_KD 0.f
#define SHOOT_FIRE_R_PID_MAX_OUT    16000
#define SHOOT_FIRE_R_PID_MAX_IOUT   0

//拨弹电机角度环PID
#define SHOOT_TRI_ANGLE_PID_KP 8.f
#define SHOOT_TRI_ANGLE_PID_KI 0.025f
#define SHOOT_TRI_ANGLE_PID_KD 100.0f
#define SHOOT_TRI_ANGLE_PID_MAX_OUT 15000
#define SHOOT_TRI_ANGLE_PID_MAX_IOUT 10000

//拨弹电机速度环PID
#define SHOOT_TRI_SPEED_PID_KP  180.f
#define SHOOT_TRI_SPEED_PID_KI  0.05f
#define SHOOT_TRI_SPEED_PID_KD  60.f
#define SHOOT_TRI_SPEED_PID_MAX_OUT 20000
#define SHOOT_TRI_SPEED_PID_MAX_IOUT 10000

//换弹电机角度环PID
//#define SHOOT_TRI_ANGLE_PID_KP 8.f
//#define SHOOT_TRI_ANGLE_PID_KI 0.03f
//#define SHOOT_TRI_ANGLE_PID_KD 100.0f
//#define SHOOT_TRI_ANGLE_PID_MAX_OUT 10000
//#define SHOOT_TRI_ANGLE_PID_MAX_IOUT 10000
//
//
////换弹电机速度环PID
//#define SHOOT_TRI_SPEED_PID_KP  100.f
//#define SHOOT_TRI_SPEED_PID_KI  0.6f
//#define SHOOT_TRI_SPEED_PID_KD  30.f
//#define SHOOT_TRI_SPEED_PID_MAX_OUT 20000
//#define SHOOT_TRI_SPEED_PID_MAX_IOUT 10000

/******************struct&enum******************/

typedef enum{
    Fire_OFF=0,
    Fire_ON=1,
}fire_mode;

typedef enum{
    SHOOT_CLOSE=0,
    SHOOT_SINGLE,
    SHOOT_ING,
    SHOOT_CONTINUES,
}trigger_cmd;

typedef struct {
    fire_mode fire_mode;//摩擦轮状态

    fire_mode fire_last_mode;//摩擦轮上一次状态

    trigger_cmd trigger_cmd;    //发射机构单发还是

//    motor_2006_t fire_l;
//
//    motor_2006_t fire_r;

//    motor_2006_t trigger;
    fp32 relative_angle_get;
    fp32 relative_angle_set;

    motor_3508_t trigger;
    motor_3508_t FL;
    motor_3508_t FR;
    motor_3508_t BL;
    motor_3508_t BR;
}launcher_t;

extern void launcher_init();
extern void launcher_mode_set();
extern void launcher_control();



#endif //DEMO1_LAUNCHER_H
