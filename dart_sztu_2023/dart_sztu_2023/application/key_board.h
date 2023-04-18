//
// Created by xhuanc on 2021/12/3.
//

#ifndef DEMO1_KEY_BOARD_H
#define DEMO1_KEY_BOARD_H
#include "remote.h"

#define DEFAULT_CNT 100/24 // 100/��������=ʵ�ʾ���ʱ��
#define MOUSE_CLICK_R_CNT 5
#define MOUSE_CLICK_L_CNT 3000

typedef enum{
    KEY_RELAX,//����û������
    KEY_CLICK,//PRESS һ��֮���RELAX ����CLICK
    KEY_DOWN,//PRESS ����һ��ʱ�� DOWN
    KEY_PRESS
}Key_Status;

typedef struct {
    Key_Status status;
    Key_Status last_status;
    uint32_t press_cnt;
    uint8_t click_flag;
    uint32_t click_cnt;
}Key;

//���̽ṹ��
typedef struct {
    Key Mouse_l;
    Key Mouse_r;
    Key W;
    Key S;
    Key A;
    Key D;
    Key Q;
    Key E;
    Key R;
    Key F;
    Key G;
    Key Z;
    Key X;
    Key C;
    Key B;
    Key V;
    Key SHIFT;
    Key CTRL;
}Key_board_t;
extern void update_pc_info();

#endif //DEMO1_KEY_BOARD_H
