//˵�ǵ���  ��ʵֻ���ϵ���һ�����
/*include*/
#include <stdlib.h>
#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"

#include "ramp.h"
#include "key_board.h"
#include "balance_ctrl.h"
#include "Gimbal.h"
#include "launcher.h"
/*define*/

/*����*/

#define DEGREE_TO_ENCODER 294912
extern RC_ctrl_t rc_ctrl;
extern motor_measure_t motor_left_measure;
extern motor_measure_t motor_right_measure;
extern launcher_t launcher;
ramp_function_source_t chassis_3508_ramp[4];
chassis_t chassis;
extern gimbal_t gimbal;

uint8_t rc_last_sw_R;


int32_t  total_ecd_ref_flag=0;
/*���� & ����*/

static void chassis_init(chassis_t *chassis);



void chassis_task(void const *pvParameters){

    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    chassis_init(&chassis);//���̳�ʼ��

    //TODO:�жϵ��̵���Ƿ�����

    //������ѭ��
    while (1){
//rc_ctrl.rc.ch[4]


        vTaskDelay(5);
    }

}



static void chassis_init(chassis_t *chassis) {


    if (chassis == NULL)
        return;




}

