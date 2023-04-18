/*轮子电机id：                            四个摩擦轮                     解算坐标：      x(前)
 *                                           前
                ****         ****                     ****               ****                              |
                          FL                                     BL                          |
                ****         ****                     ****               ****                              |
                                                                                                                |
    左                                                                                         右
                                                                                                                |
                ****              ****                     ****               ****                              |
                           BR                                       FR                 |
                ****              ****                     ****               ****
                                                 后
    */

#include "launcher.h"
#include "Gimbal.h"
#include "can_receive.h"
#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "key_board.h"

#define frition_rate  0.00066139
#define frition_speed 1
#define M2006_POSITION_TO_ECD (36 * 8192)
#define M2006_ECD_TO_POSITION  (1/M2006_POSITION_TO_ECD)

uint8_t rc_last_sw_L;

uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间 或者 鼠标左键按下的持续时间

uint8_t trigger_flag=0;

extern Key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;
extern gimbal_t gimbal;
extern motor_measure_t motor_2006_measure[3];
extern chassis_t chassis;
extern motor_measure_t motor_3508_dart[4];
extern motor_measure_t motor_turn_measure;

//通过赋值进行发射机构的初始化
launcher_t launcher;
int32_t total_ecd_ref;

int32_t turn_count=0;

int32_t  finish_flag=0;

fp32 set_position;
float goal_ecd;
float up_speed_out;
float get_position;

float turn[4];
int turn_i;

void launcher_mode_set(){

    //摩擦轮关闭时,做拨杆向上拨一下开启摩擦轮
    if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_OFF)
    {
        launcher.fire_mode=Fire_ON;
    }
    //摩擦轮开启时,做拨杆向上拨一下关闭摩擦轮  控制达到妙电机旋转九十度
    else if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_ON)
    {
        launcher.fire_mode=Fire_OFF;
    }

    //拨轮控制
    if(launcher.fire_mode==Fire_ON&&switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        if((!switch_is_down(rc_last_sw_L)&&switch_is_down(rc_ctrl.rc.s[RC_s_L])||KeyBoard.Mouse_l.status==KEY_CLICK))
        {
            launcher.trigger_cmd=SHOOT_SINGLE;
        }

    }
    else
    {
        launcher.trigger_cmd=SHOOT_CLOSE;
    }

    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];

}


//发射机构控制
void launcher_control(){


      launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                          launcher.trigger.motor_measure->offset_ecd);

        if(launcher.fire_mode==Fire_ON)
        {
            //上弹电机控制

//            if(turn_count==0&&rc_ctrl.rc.ch[4]==0)
//            {
//                chassis.motor_chassis[RF].give_current=0;
//            }

//            else if(turn_count==1 && rc_ctrl.rc.ch[4]!=0) {
            //    set_position = 0;

          // 推弹电机速度的杆量控制
            set_position=  (float) rc_ctrl.rc.ch[4] / 1000.0f;
                goal_ecd = set_position * M2006_POSITION_TO_ECD;
                get_position=chassis.motor_chassis[RF].motor_measure->total_ecd;
                up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,
                                        get_position,
                                        goal_ecd);
                chassis.motor_chassis[RF].give_current = (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                            chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                            up_speed_out);

                chassis.motor_chassis[RF].give_current = (int16_t) (-chassis.motor_chassis[RF].give_current);


            //摩擦轮控制ss
            launcher.FL.speed=2500;//+  左前
            launcher.BL.speed=-2500;//-  右前

            launcher.FR.speed=-1500;//-   右后

            launcher.BR.speed=1500;//+   左后

            //翻转电机控制
            if (launcher.trigger_cmd == SHOOT_CLOSE) {
                launcher.trigger.speed = 0;
            }
            else if (launcher.trigger_cmd == SHOOT_SINGLE) { //收到单发命令
                launcher.trigger_cmd = SHOOT_ING;//进入正在单发状态
                launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                                      launcher.trigger.motor_measure->offset_ecd);
                if(turn_i==4)
                {
                    turn_i=0;
                }
                      launcher.relative_angle_set = turn[turn_i];//实测角度
                    turn_i++;
                    turn_count=1;


                    if(launcher.relative_angle_set>180)
                    {
                        launcher.relative_angle_set=launcher.relative_angle_set-360;
                    }
                    else if(launcher.relative_angle_set<-180)
                {
                    launcher.relative_angle_set=launcher.relative_angle_set+360;
                }


            }
            else if (launcher.trigger_cmd == SHOOT_ING) { //单发状态
                //判断是否到达目标位置26
                if(KeyBoard.Mouse_l.status!=KEY_PRESS)
                {
                    launcher.trigger_cmd =SHOOT_CLOSE;
                    launcher.trigger.speed=0;
                }

            }


            launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                              launcher.relative_angle_set,180,-180);
            launcher.trigger.give_current= pid_calc(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);

//            if(turn_count==0&&rc_ctrl.rc.ch[4]==0)
//            {
//                chassis.motor_chassis[RF].give_current=0;
//            }
//            else if(turn_count==1 && rc_ctrl.rc.ch[4]!=0) {
//               set_position=set_position+1;
//                goal_ecd = set_position * M2006_POSITION_TO_ECD;
//                up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,
//                                        chassis.motor_chassis[RF].motor_measure->total_ecd,
//                                        goal_ecd);
//                chassis.motor_chassis[RF].give_current = (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
//                                                                            chassis.motor_chassis[RF].motor_measure->speed_rpm,
//                                                                            up_speed_out);
//
//                chassis.motor_chassis[RF].give_current = (int16_t) (-chassis.motor_chassis[RF].give_current);
//                turn_count=0;
//            }

        }


        else
        {
            launcher.FL.speed=0;
            launcher.FR.speed=0;
            launcher.BL.speed=0;
            launcher.BR.speed=0;
            launcher.trigger.give_current=0;
            chassis.motor_chassis[RF].give_current=0;
        }




    launcher.BL.give_current= pid_calc(&launcher.BL.speed_p,launcher.BL.motor_measure->speed_rpm, launcher.BL.speed);
    launcher.BR.give_current= pid_calc(&launcher.BR.speed_p,launcher.BR.motor_measure->speed_rpm, launcher.BR.speed);
    launcher.FL.give_current= pid_calc(&launcher.FL.speed_p,launcher.FL.motor_measure->speed_rpm, launcher.FL.speed);
    launcher.FR.give_current= pid_calc(&launcher.FR.speed_p,launcher.FR.motor_measure->speed_rpm, launcher.FR.speed);

//    launcher.BL.give_current=0;
//    launcher.BR.give_current= 0;
//    launcher.FL.give_current= 0;
//    launcher.FR.give_current=0;




}




void launcher_init(){

     launcher.trigger_cmd=SHOOT_CLOSE;//初始时发射机构默认关闭

     launcher.fire_last_mode=Fire_OFF;//初始时摩擦轮默认关闭

     launcher.fire_mode=Fire_OFF;//初始时摩擦轮默认关闭

     //摩擦轮电机初始化
     launcher.FL.motor_measure=&motor_3508_dart[0];  //左前1
     launcher.FR.motor_measure=&motor_3508_dart[1];  //右后2
     launcher.BL.motor_measure=&motor_3508_dart[2];  //右前3
     launcher.BR.motor_measure=&motor_3508_dart[3];  //左后4

     chassis.motor_chassis[0].motor_measure=&motor_2006_measure[0];//上弹电机初始化

    launcher.trigger.motor_measure=&motor_turn_measure;//翻转电机初始化


    //上弹电机pid
    chassis.motor_chassis[0].speed_p.max_output=up_speed_PID_MAX_OUT;
    chassis.motor_chassis[0].speed_p.integral_limit=up_speed_PID_MAX_IOUT;
    chassis.motor_chassis[0].speed_p.p=up_speed_PID_KP;
    chassis.motor_chassis[0].speed_p.i=up_speed_PID_KI;
    chassis.motor_chassis[0].speed_p.d=up_speed_PID_KD;

    chassis.motor_chassis[0].angle_p.max_output=up_angle_PID_MAX_OUT;
    chassis.motor_chassis[0].angle_p.integral_limit=up_angle_PID_MAX_IOUT;
    chassis.motor_chassis[0].angle_p.p=up_angle_PID_KP;
    chassis.motor_chassis[0].angle_p.i=up_angle_PID_KI;
    chassis.motor_chassis[0].angle_p.d=up_angle_PID_KD;

    chassis.motor_chassis[0].motor_measure->offset_ecd=motor_2006_measure->ecd;

    //摩擦轮pid
    launcher.BR.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.BR.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.BR.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.BR.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.BR.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.BL.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.BL.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.BL.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.BL.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.BL.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.FL.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.FL.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.FL.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.FL.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.FL.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.FR.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.FR.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.FR.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.FR.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.FR.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    //翻转电机pid
    launcher.trigger.angle_p.p=SHOOT_TRI_ANGLE_PID_KP;
    launcher.trigger.angle_p.i=SHOOT_TRI_ANGLE_PID_KI;
    launcher.trigger.angle_p.d=SHOOT_TRI_ANGLE_PID_KD;
    launcher.trigger.angle_p.max_output=SHOOT_TRI_ANGLE_PID_MAX_OUT;
    launcher.trigger.angle_p.integral_limit=SHOOT_TRI_ANGLE_PID_MAX_IOUT;

    launcher.trigger.speed_p.p=SHOOT_TRI_SPEED_PID_KP;
    launcher.trigger.speed_p.i=SHOOT_TRI_SPEED_PID_KI;
    launcher.trigger.speed_p.d=SHOOT_TRI_SPEED_PID_KD;
    launcher.trigger.speed_p.max_output=SHOOT_TRI_SPEED_PID_MAX_OUT;
    launcher.trigger.speed_p.integral_limit=SHOOT_TRI_SPEED_PID_MAX_IOUT;


    launcher.trigger.motor_measure->offset_ecd=8167;
    launcher.relative_angle_set=0;

    //电机要转的调度
    turn[0]=(float)-1.14;
    turn[1]=(float)178.59;
    turn[2]=(float)90.48;
    turn[3]=(float)-89.69;
    turn_i=0;

}
