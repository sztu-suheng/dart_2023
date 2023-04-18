#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/*include*/
#include "struct_typedef.h"
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "user_lib.h"

/*define*/

//任务开始空闲一段时间

#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define up_channel 4
#define CHASSIS_MODE_CHANNEL 0

//
//#define up_speed_PID_KP  1.5f
//#define up_speed_PID_KI     0.0f
//#define up_speed_PID_KD     0.0f
//#define up_speed_PID_MAX_OUT 5000.0f
//#define up_speed_PID_MAX_IOUT 1000.0f
//
//#define up_angle_PID_KP  1.5f
//#define up_angle_PID_KI     0.0f
//#define up_angle_PID_KD     0.0f
//#define up_angle_PID_MAX_OUT 5000.0f
//#define up_angle_PID_MAX_IOUT 1000.0f


//推弹点击PID
#define up_speed_PID_KP  1.5f
#define up_speed_PID_KI     0.0f
#define up_speed_PID_KD     0.0f
#define up_speed_PID_MAX_OUT 5000.0f
#define up_speed_PID_MAX_IOUT 1000.0f


#define up_angle_PID_KP  1.5f
#define up_angle_PID_KI     0.0f
#define up_angle_PID_KD     100.f
#define up_angle_PID_MAX_OUT 5000.0f
#define up_angle_PID_MAX_IOUT 1000.0f

typedef enum {
    RF=0,
    LF,
    LB,
    RB
}chassis_motor_index;

typedef enum
{
    up_on,
    up_off,
}up_mode;

typedef struct
{

    motor_3508_t motor_chassis[4];
    up_mode mode;

}chassis_t;

typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_BACK,
    CHASSIS_ONLY,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_SPIN,
} chassis_mode_e;




extern void chassis_task(void const *pvParameters);

//
#endif //DEMO1_CHASSIS_H

