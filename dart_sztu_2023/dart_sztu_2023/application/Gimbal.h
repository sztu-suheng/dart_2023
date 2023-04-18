#ifndef DEMO1_GIMBAL_H
#define DEMO1_GIMBAL_H
/*      Include     */

#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"

/*      define     */

#define GIMBAL_TASK_INIT_TIME 201
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

#define RC_TO_YAW 0.002
#define RC_TO_PITCH 0.01
#define MAX_ABS_ANGLE 110
#define MIN_ABS_ANGLE -110
#define MAX_RELA_ANGLE 40
#define MIN_RELA_ANGLE -20
//��̨ת���ٶ�ϵ��
#define GIMBAL_RC_MOVE_RATIO_PIT 0.8f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.1f

#define GIMBAL_YAW_ANGLE_PID_KP     12.f
#define GIMBAL_YAW_ANGLE_PID_KI     0.0f
#define GIMBAL_YAW_ANGLE_PID_KD     25.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT    1000.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   2000.f

#define GIMBAL_YAW_SPEED_PID_KP     120.f
#define GIMBAL_YAW_SPEED_PID_KI     0.0f
#define GIMBAL_YAW_SPEED_PID_KD     25.f
#define GIMBAL_YAW_SPEED_MAX_OUT    20000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT   6000.f

#define GIMBAL_PITCH_ANGLE_PID_KP   0.2f
#define GIMBAL_PITCH_ANGLE_PID_KI   0.f
#define GIMBAL_PITCH_ANGLE_PID_KD   0.0f
#define GIMBAL_PITCH_ANGLE_MAX_OUT  80.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 30.f

#define GIMBAL_PITCH_SPEED_PID_KP   10.0f
#define GIMBAL_PITCH_SPEED_PID_KI   0.0f
#define GIMBAL_PITCH_SPEED_PID_KD   0.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT  12000.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT 12000.f

/*      �ṹ���ö��     */

typedef enum {
    GIMBAL_RELAX=0,//��̨ʧ��
    GIMBAL_BACK,
    GIMBAL_ACTIVE
}gimbal_mode_e;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;
    motor_2006_t trigger;

    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

//    AHRS_Eulr_t*Eulr;   //��̬��

    bool_t yaw_is_back;
    bool_t pitch_is_back;
}gimbal_t;

extern void gimbal_task(void const*pvParameters);


#endif //DEMO1_GIMBAL_H
