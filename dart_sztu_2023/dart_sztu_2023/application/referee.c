//%%88888888888888888888888888888888888888888888888888888888888888888888888888888%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8%&%BB%B%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%i. .  `iJ8B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B8%%Bw''.  '.''`q%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB].. .......'.w$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BBm^'.........`+B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88' .........."B%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8Bf .........'.0&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%B&@8B%8%%%%%%%%%%%%BB8%%w  ........`]CB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%BW@%B%8%%%%%%%%%%%%%%%@~  . .......^88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%BB%8%%8#>d8%%$%8%%%%%%%B8%%b. ........  #8%%%%%8%%}}C,UWW%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%W&%pl` ''''`[LB%%BZq-x{Z@BJ   ..... ..lJ%%B}C}W%W*`...... ]8$%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%O`'`'.    . '  '.'^ ?%%B%b:. ...... ..&@%%B~.... .. .....^` `w#&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%W`.'..'    ... .'.X{BB%L..   ...  ..O%8BBz..            ..:C%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%BB_'......... ' `w@%B%B`  `......`Ib@%%&%JI .           ..QBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%J' ..... '. Ih@#BB%%{a'  .......'"&}BBBBBBB%B%%hu '.    . ''Q$&%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%%%8%8%b^''..' . "+{WW8%%%%&%J       .....0@BBBBBBB%%%%BB%B$w .'.     'h8%%%%B%%%B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%@^ .'''"..''.''.'.''M%#8B%%%%%%8M`'..     ...h%BBBBB%%%%%%%BBB%BB%C[`   .. '','''.'` B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%Bj`    .'. '......^B%B%%%%%%%%%%k"'.'       ,M@BBB&%%BBBB8%&ZUYhaw#%&W'`  ...........' 8%%%%%%%%%%%%%%%%%%
//%%%%%%%8L``'.   '    . ^`?@%%8%%%%%%B%8L` '        ''.'...'...........'..z@BB@BL.'...        .`_Z%%%%%%%%%%%%%%%%%
//%%%%%%%m;`''''''......."&8%%8%%%%%%%%B%` .       . . .'.  .......    '.'C%%BB%B&J'         ..'':_%@%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%o ..... .v%B%%%%%%%%%BB%'^..                          .''J&88%%%B%8t.....   0&@88%8%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%f      .'$B%%%%%%%%%%8Q+'. . ... '''..    '`^.`.        a8&8%8%%%%B$`.......kB%%888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%+'      ^B%%%%%%%%8%%&.'  . '';Y%%%8%BB%BBBBb  . . .   08BB88%%%%%%}'.     .n%88888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B8h^      'MB8%%%%%%%8%:.   ......^u&%%B%%BB&h       '.'q%8%%%%%%%%%BZ'.   . .bB88888%%%%%%%%%%%%%%%%%
//%%%%%%%W%$MQh:..     .._B%%%%%%%B#j       .  .U%%88888BB!..    ... v%8%%%%%%%%%%Bl     .'''nw&W@$%%%%%%%%%%%%%%%%%
//%%%%%%%#+''.''`'''... . *$%%%%%%W''        . zB%%%8888B<'`    ....J&8%%%%%%%%%BBw.      ...'.',.b%%%%%%%%%%%%%%%%%
//%%%%%%%B8l.............'."O@%%8W^.  ..     :u&%&%B%%%p' '''..   .bBW%%%%%%%%%%C+.'     ....... %W%%%%%%%%%%%%%%%%%
//%%%%%%%%%&..'.''. `  ..''' n&%QnkhJM&8B@BBB%%88%8888q...        o%88%%%%%%%&%]`.. ...  .  .  'M&%%%%%%%%%%%%%%%%%%
//%%%%%%%B%@ZadJLO888h`'''..   ;%@8%BB%%8&8%%888888%%Q'..       '+&8%%%%%%%W@l. .. .'''C%BW0Xkkq%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B%%B%8%BWQ '.''''' `>{BW%%B8%%%%%8888BZ`'.     . 'q88%BB%B8B!^. ..'... [&%%BBB%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%B%8%%8%%Bk .'''''`.^`'xY8%%%%%%%%%8f ^       . r$8%%&Bt`^   .. ... Y@&88%%%%8%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%B%%%%B%8B%8%Wl^'.. ......  ''',[J&%8Bv` .       .,%8%_.`' .....   . 'Z&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%B%%%%%%%%%%%%%%%%%%@k'`''........ .   . +B%%X!.        ..b}%al    '... .    '.`tBB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%8B{~''`'........:j,'.']BBBJ. .     ..  v%@q ^  '``... .     '.'L8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%@%&BQ` ''  ,#%B%8BB&8B&0' .      ..'<&8&888%88&%%Bu     'w#$%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%88WkMB8%%%%%%%%%%f'          .b%8%%%%%%%%%%%%8%@mW@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%8B%%%B%%8%%%%%B%x`.        . -B8%%%%%%%%%%%B&8%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#z'`.      .' .B&B8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8B@M` ..      . ^nOB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%C'.. .     ..']%B%%%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB#....        ' BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bq .   .    ' Q%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88888] .^^.   "p@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B%%B%$d'"  ^.&BB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB%8M '^hB%8%88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B88%%%%W&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "Referee.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Detection.h"
#include "key_board.h"
#include "launcher.h"

//��ģʽ����ʱ�ļ��
#define UI_CHAR_LENGTH 15
extern UART_HandleTypeDef huart6;
extern Key_board_t KeyBoard;
Graphic_Operate static_update_flag=UI_NONE;
Graphic_Operate one_layer_update_flag=UI_NONE;
Graphic_Operate two_layer_update_flag=UI_NONE;
Graphic_Operate three_layer_update_flag=UI_NONE;

uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};

Referee_info_t Referee;

/*
 * UI����״̬
 */
ui_robot_status_t ui_robot_status={

    .gimbal_mode=GIMBAL_RELAX,
    .chassis_mode=CHASSIS_RELAX,
    .block_warning=false,
    .super_cap_value=0.f,
//    .shoot_heat_limit=0,
    .pitch_value=0.f,
    .relative_yaw_value=0.f,
};

/*����������*/
static void referee_unpack_fifo_data(void);
static bool_t Referee_read_data(uint8_t *ReadFromUsart);
static void ui_static_draw();
/*����ϵͳ������*/

extern fp32 INS_angle[3];

//�����жϺ���
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);//��ȡUART6-SR ��UART6-DR; ����жϱ�־λ
//        static uint16_t this_time_rx_len = 0;
//
//        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart6.hdmarx);
////            detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
////            detect_hook(REFEREE_TOE);
//        }

        __HAL_DMA_DISABLE(huart6.hdmarx); //ʹ��dma_rx

        Referee_read_data(&usart6_buf[0]);

        memset(&usart6_buf[0],0,REFEREE_BUFFER_SIZE);//��0

        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx,DMA_LISR_TCIF1); //���������ɱ�־λ

        __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);//����DMA �������ݴ�С ��λΪ�ֽ�

        __HAL_DMA_ENABLE(huart6.hdmarx); //ʹ��DMARx

    }
}

//���ݲ���ϵͳ��Ϣ�жϻ����˵�ID�Ͷ�Ӧ�ͻ��˵�ID
void judge_team_client(){
    //��������Ϊ�췽
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 		 	= 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane		 	= 6;
        Referee.ids.teammate_sentry		= 7;

        Referee.ids.client_hero 		 	= 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane			= 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;
            
            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;
            
            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//��������Ϊ����
    else{
        Referee.ids.teammate_hero 		 	= 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane		 	= 106;
        Referee.ids.teammate_sentry		= 107;

        Referee.ids.client_hero 		 	= 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane			= 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}
int CmdID=0;//�������������

bool_t Referee_read_data(uint8_t *ReadFromUsart)
{

    uint16_t judge_length;

    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

    if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //�ж�֡ͷ�Ƿ�Ϊ0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))
            {
//                retval_tf=1;//��ʾ���ݿ���
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 ����״̬ 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 �������   ������������
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        break;

                    case Referee_ID_game_robot_survivors://0x0003 ������״̬HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_survivors);
                        break;

                    case Referee_ID_game_dart_state: //0x0004 ���ڷ���״̬
                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
                        break;

                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF״̬     1HZ
                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
                        break;

                    case Referee_ID_event_data://0x0101 �����¼�����      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                    case Referee_ID_supply_projectile_action://0x0102 ���ز���վ������ʶ����   �����ı�֮����
                        memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        break;

                    case Referee_ID_supply_warm://0x0104    ����ϵͳ��������    ��������֮����
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_shoot_time://0x0105    ���ڷ���ڵ���ʱ    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_missile_shoot_time);
                        break;

                    case Referee_ID_game_robot_state://0x0201   ������״̬����     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//�ж�һ�»����������Ķ�������� �Լ���Ӧ�Ļ�е��id�Ϳͻ���id
                        break;

                    case Referee_ID_power_heat_data://0x0202    ʵʱ������������    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);
                        break;

                    case Referee_ID_game_robot_pos://0x0203     ������λ������     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  ��������������     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        break;

                    case Referee_ID_aerial_robot_energy://0x0205    ���л���������״̬���� 10HZ
                        memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        break;

                    case Referee_ID_robot_hurt://0x0206     �˺�״̬����  �˺���������
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        break;

                    case Referee_ID_shoot_data://0x0207     ʵʱ�������  �������
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   ʣ�෢����   10HZ���ڷ���
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    ������RFID״̬��1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  ���ڻ����˿ͻ���ָ����, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;
//
//                    case 0x0301: //���Ի����˽���
//                        memcpy(&test_buf,ReadFromUsart+DATA, sizeof(tx_client_string));
//                        break;
                    default:
                        break;
                }
                detect_handle(DETECT_REFEREE);
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //���һ�����ݰ������˶�֡����,���ٴζ�ȡ
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}


//���Ʊ���
uint8_t state_first_graphic;//0~7ѭ�� ���µ�ͼ����
uint8_t ClientTxBuffer[200];//���͸��ͻ��˵����ݻ�����
//��������
//��0�㻭���ַ��� �ַ����ֻ�� 30 Byte
/**************************************/
/**
 * �����ַ���
 * @param graphic
 * @param name
 * @param op_type
 * @param layer
 * @param color
 * @param size
 * @param length
 * @param width
 * @param start_x
 * @param start_y
 * @param character
 */
void String_Graphic(ui_string_t*clientData,
                  const char* name,
                  uint32_t op_type,
                  uint32_t layer,
                  uint32_t color,
                  uint32_t size,
                  uint32_t length,
                  uint32_t width,
                  uint32_t start_x,
                  uint32_t start_y,
                  const char *character)// ����
{
    ui_graphic_data_struct_t*data_struct=&clientData->graphic_data_struct;
    data_struct->graphic_tpye=UI_CHAR;

    for(char i=0;i<3;i++)
        data_struct->graphic_name[i] = name[i];	//�ַ�����
    data_struct->operate_tpye=op_type;// ͼ�����  1Ϊ����
    data_struct->layer=layer;//�ڵڼ�ͼ��
    data_struct->color=color;//��ɫ
    data_struct->start_angle=size;
    data_struct->end_angle=length;
    data_struct->width=width;
    data_struct->start_x=start_x;
    data_struct->start_y=start_y;
    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(clientData->data,character,30);
}

/**
 * ���Ƹ�ͼ��
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param start_angle
 * @param end_angle
 * @param width
 * @param start_x
 * @param start_y
 * @param radius
 * @param end_x
 * @param end_y
 */
void Figure_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//����ʲôͼ��
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;         //Char��
    graphic->layer        = layer;//���ڵ�һ��
    graphic->color        = color;//��ɫ
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}

static void draw_static_string(ext_string_data_t* ui_string)
{
    switch (state_first_graphic) {
        case 0:{
            char first_line[30]  = {"CHA: RELX FOLW SPIN BLOK"};//
            String_Graphic(&ui_string->clientData, "CL1", static_update_flag, UI_ZERO_LAYER, UI_GREEN, 15, strlen(first_line), 2, 124,
                           1080-385, first_line);
        }
            break;
        case 1: {
            char second_line[30] = {"GIM: RELX GYRO AUTO BUFF"};//��̨ģʽ
            String_Graphic(&ui_string->clientData, "CL2", static_update_flag, UI_ZERO_LAYER, UI_GREEN, 15, strlen(second_line), 2, 124,
                           1080-425, second_line);
            break;
        }
        case 2:{
            char cap_line[30]={"CAP:"};
            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
                           400,cap_line);
            break;
        }

        case 3:{
            char pitch_line[30]={"PIT:"};
            String_Graphic(&ui_string->clientData,"PIT",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(pitch_line),2,620,
                           1080-538,pitch_line);
            break;
        }

        case 4:{
            char yaw_line[30]={"YAW:"};
            String_Graphic(&ui_string->clientData,"YAW",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(yaw_line),2,620,
                           1080-588,yaw_line);
            break;
        }
        default:
        {
            state_first_graphic=0;
        }
            break;
    }
}

/**
 * ���Ƹ�����
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param decimal
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Float_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                   const char* name,
                   uint32_t operate_tpye,
                   uint32_t graphic_tpye,//����ʲôͼ��
                   uint32_t layer,
                   uint32_t color,
                   uint32_t size,
                   uint32_t decimal,
                   uint32_t width,
                   uint32_t start_x,
                   uint32_t start_y,
                   float number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//
    graphic->color        = color;//��ɫ
    graphic->start_angle  = size;
    graphic->end_angle    = decimal;//С����Чλ
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number*1000;//�������͵�Ҫ��1000��ת��Ϊһ��int32���͵�
}

/**
 * ��������
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param zero
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Int_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                 const char* name,
                 uint32_t operate_tpye,
                 uint32_t graphic_tpye,//����ʲôͼ��
                 uint32_t layer,
                 uint32_t color,
                 uint32_t size,
                 uint32_t zero,
                 uint32_t width,
                 uint32_t start_x,
                 uint32_t start_y,
                 int32_t number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//���ڵ�һ��
    graphic->color        = color;//��ɫ
    graphic->start_angle  = size;
    graphic->end_angle    = zero;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number;
}

/*
 * ��Ԫ�ػ滭,��ÿ�ζ�Ҫ�滭   ���Ի���ͼ������ÿ�ζ�ҪADD
 */
void static_char_draw() {

    ext_string_data_t ui_string;
    if (state_first_graphic >= 5) {
        state_first_graphic = 0;
    }
    //��֡��֡ͷ
    ui_string.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_string.txFrameHeader.data_length =
            sizeof(ext_student_interactive_header_data_t) + sizeof(ui_string_t);
    ui_string.txFrameHeader.seq = 0;//�����
    memcpy(ClientTxBuffer, &ui_string.txFrameHeader, sizeof(frame_header_struct_t));
    append_CRC8_check_sum(ClientTxBuffer, sizeof(frame_header_struct_t));//ͷУ��

    //����֡��֡ͷ
    // ������ ��֡�����ID�������ϵͳͨ�ŵ���֡���ݵ�ID Referee_��ͷ
    ui_string.CmdID=Referee_ID_robot_interactive_header_data;//�����˼佻��������ID ������֡��ID

    //�˴���IDΪ�������ݵ�ID UI_INTERACT��ͷ
    ui_string.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_char_graphic;//�����˽������ݵ�ID �滭�ַ���ID
    ui_string.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_string.dataFrameHeader.receiver_ID=Referee.SelfClient;

    //�����������
    draw_static_string(&ui_string);//���ƾ�̬Ԫ�� Ԫ������Ϊchar �ַ�������

    //����֡ͷ���ַ��뻺����
    memcpy(ClientTxBuffer +  Referee_LEN_FRAME_HEAD, (uint8_t *) &ui_string.CmdID,
           Referee_LEN_CMD_ID + ui_string.txFrameHeader.data_length);
    //�Է�����������CRC16У��λ
    append_CRC16_check_sum(ClientTxBuffer, sizeof(ui_string));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBuffer, sizeof(ui_string));

    //��O��滭���ַ�������
    state_first_graphic++;
}

int32_t cap_percentage=90;

//��׼���߻���
void small_aimLine_draw(){
    ext_graphic_five_data_t ui_aim_line;//���ͱ���

    //����ͨ��֡ͷ����

    ui_aim_line.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_aim_line.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*5;
    ui_aim_line.txFrameHeader.seq=0;    //��������

    //����ͼ�����ݸ�ֵ����xxx_data_struct��


    memcpy(ClientTxBuffer,&ui_aim_line.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_aim_line.CmdID= Referee_ID_robot_interactive_header_data;

    //����֡ͷ
    ui_aim_line.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_aim_line.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_aim_line.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_five_graphic;

    switch (Referee.GameRobotStat.shooter1_17mm_speed_limit)
    {
        case 15:
        {
            //6m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[0],"SI1",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,974,372,
                           0,974,366);

            Figure_Graphic(&ui_aim_line.clientData[1],"SI2",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,989,372,
                           0,989,366);

                    //5m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[2],"SI3",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,972,406,
                           0,972,398);

            Figure_Graphic(&ui_aim_line.clientData[3],"SI4",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,995,406,
                           0,995,398);
        }break;

        case 18:
        {
            //5m 17.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[0],"SI1",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,969,425,
                           0,969,417);

            Figure_Graphic(&ui_aim_line.clientData[1],"SI2",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,992,425,
                           0,992,417);

            //6m 17.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[2],"SI3",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,970,410,
                           0,970,404);

            Figure_Graphic(&ui_aim_line.clientData[3],"SI4",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,989,410,
                           0,989,404);
        }break;

        case 30:{
            //5-6m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[0],"SI1",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,970,472,
                   0,970,462);

            Figure_Graphic(&ui_aim_line.clientData[1],"SI2",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,989,472,
                   0,989,462);

            Figure_Graphic(&ui_aim_line.clientData[2],"SI3",UI_DELETE,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,970,472,
                           0,970,462);

            Figure_Graphic(&ui_aim_line.clientData[3],"SI4",UI_DELETE,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,989,472,
                           0,989,462);
        }break;

        default:{
            //6m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[0],"SI1",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_GREEN,0,0,2,974,372,
                           0,974,366);

            Figure_Graphic(&ui_aim_line.clientData[1],"SI2",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_GREEN,0,0,2,989,372,
                           0,989,366);

            //5m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[2],"SI3",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_GREEN,0,0,2,972,406,
                           0,972,398);

            Figure_Graphic(&ui_aim_line.clientData[3],"SI4",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_GREEN,0,0,2,995,406,
                           0,995,398);
        }break;
    }

//    Int_Graphic(&ui_aim_line.clientData[4],"SI5",one_layer_update_flag,UI_INT,
//                UI_ONE_LAYER,UI_GREEN,17,0,2,1920-528,308,cap_percentage);
//    Figure_Graphic(&ui_aim_line.clientData[4],"SI5",UI_NONE,UI_LINE,
//                   UI_ONE_LAYER,UI_GREEN,0,0,2,995,406,
//                   0,995,398);

    //����֡ͷ���ַ��뻺����
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_aim_line.CmdID,
           Referee_LEN_CMD_ID+ui_aim_line.txFrameHeader.data_length);
    //֡βCRC16����
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_aim_line));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_aim_line));

}

void aimLine_cap_draw(){
    ext_graphic_seven_data_t ui_aim_line;//���ͱ���

    //����ͨ��֡ͷ����

    ui_aim_line.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_aim_line.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*7;
    ui_aim_line.txFrameHeader.seq=0;    //��������
    //����ͼ�����ݸ�ֵ����xxx_data_struct��


    memcpy(ClientTxBuffer,&ui_aim_line.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_aim_line.CmdID= Referee_ID_robot_interactive_header_data;

    //����֡ͷ
    ui_aim_line.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_aim_line.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_aim_line.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_seven_graphic;

    switch (Referee.GameRobotStat.shooter1_17mm_speed_limit) {
        case 15: {
            //6m 14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[0], "LI4", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 973, 369,
                           0, 1006, 369);

            //3m 14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[1], "LI2", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 955, 439,
                           0, 1028, 439);

            //1m 14.5m/s ��Բ
            Figure_Graphic(&ui_aim_line.clientData[2], "LI3", one_layer_update_flag, UI_CIRCLE,
                           UI_ONE_LAYER, UI_PINK, 0, 0, 2, 985, 439,
                           15, 0, 0);

            //5m  14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[3], "LI1", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 968, 1080 - 678,
                           0, 1010, 1080 - 678);

            //3m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[4],"LI5",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,958,444,
                           0,958,434);

            Figure_Graphic(&ui_aim_line.clientData[5],"LI6",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,995,444,
                           0,995,434);

        }break;

        case 18:
        {
            //6m  17.0m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[0],"LI1",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,970,407,
                           0,1003,407);

            //5m 17.0m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[1],"LI2",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,962,421,
                           0,1000,421);

            //3m 17.5m/s
            Figure_Graphic(&ui_aim_line.clientData[2],"LI3",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,952,448,
                           0,1016,448);

            //1m 17.5m/s Բ
            Figure_Graphic(&ui_aim_line.clientData[3],"LI4",one_layer_update_flag,UI_CIRCLE,
                           UI_ONE_LAYER,UI_PINK,0,0,2,986,447,
                           15,0,0);

            //3m 17.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[4],"LI5",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,963,455,
                           0,963,441);

            Figure_Graphic(&ui_aim_line.clientData[5],"LI6",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,1000,455,
                   0,1000,441);


        }break;

        case 30:
        {
            //6m 29m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[0],"LI1",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,965,467,
                           0,1003,467);

            //5~6 m 29m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[1],"LI2",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,965,467,
                           0,1003,467);

            //3m 29m/s
            Figure_Graphic(&ui_aim_line.clientData[2],"LI3",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,951,470,
                           0,1015,470);

            //1m 29m/s Բ
            Figure_Graphic(&ui_aim_line.clientData[3],"LI4",one_layer_update_flag,UI_CIRCLE,
                           UI_ONE_LAYER,UI_PINK,0,0,2,983,443,
                           15,0,0);

            //3m 29m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[4],"LI5",one_layer_update_flag,UI_LINE,
                        UI_ONE_LAYER,UI_YELLOW,0,0,2,962,477,
                        0,962,463);

            Figure_Graphic(&ui_aim_line.clientData[5],"LI6",one_layer_update_flag,UI_LINE,
                        UI_ONE_LAYER,UI_YELLOW,0,0,2,999,477,
                        0,999,463);

        }break;

        default:{
            //6m 14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[0], "LI4", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 973, 369,
                           0, 1006, 369);

            //3m 14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[1], "LI2", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 955, 439,
                           0, 1028, 439);


            //1m 14.5m/s ��Բ
            Figure_Graphic(&ui_aim_line.clientData[2], "LI3", one_layer_update_flag, UI_CIRCLE,
                           UI_ONE_LAYER, UI_PINK, 0, 0, 2, 985, 439,
                           15, 0, 0);

            //5m  14.5m/s ��װ��
            Figure_Graphic(&ui_aim_line.clientData[3], "LI1", one_layer_update_flag, UI_LINE,
                           UI_ONE_LAYER, UI_YELLOW, 0, 0, 2, 968, 1080 - 678,
                           0, 1010, 1080 - 678);

            //3m 14.5m/s Сװ�װ�
            Figure_Graphic(&ui_aim_line.clientData[4],"LI5",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,958,444,
                           0,958,434);

            Figure_Graphic(&ui_aim_line.clientData[5],"LI6",one_layer_update_flag,UI_LINE,
                           UI_ONE_LAYER,UI_YELLOW,0,0,2,995,444,
                           0,995,434);


        }break;
    }



    //����֡ͷ���ַ��뻺����
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_aim_line.CmdID,
           Referee_LEN_CMD_ID+ui_aim_line.txFrameHeader.data_length);
    //֡βCRC16����
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_aim_line));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_aim_line));
}

void self_line_draw(){
    ext_graphic_five_data_t ui_selfLine;

    ui_selfLine.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_selfLine.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*5;
    ui_selfLine.txFrameHeader.seq=0;    //��������
    //����ͼ�����ݸ�ֵ����xxx_data_struct��

    memcpy(ClientTxBuffer,&ui_selfLine.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_selfLine.CmdID= Referee_ID_robot_interactive_header_data;

    //����֡ͷ
    ui_selfLine.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_selfLine.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_selfLine.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_five_graphic;

    Figure_Graphic(&ui_selfLine.clientData[1],"SL0",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,563,1080-1078,
                   0,726,1080-839);

    Figure_Graphic(&ui_selfLine.clientData[2],"SL1",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,726,1080-839,
                   0,1257,1080-839);

    Figure_Graphic(&ui_selfLine.clientData[3],"SL2",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    Figure_Graphic(&ui_selfLine.clientData[4],"SL3",UI_NONE,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    Figure_Graphic(&ui_selfLine.clientData[5],"SL4",UI_NONE,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    //����֡ͷ���ַ��뻺����
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_selfLine.CmdID,
           Referee_LEN_CMD_ID+ui_selfLine.txFrameHeader.data_length);
    //֡βCRC16����
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_selfLine));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_selfLine));

}

void one_layer_draw()
{
    static uint8_t oneLayerStep=-1;
    oneLayerStep++;

    switch (oneLayerStep) {
        case 0:{
            aimLine_cap_draw();
        }break;

        case 1:{
            self_line_draw();
        }break;

        default:{
            oneLayerStep=-1;
            one_layer_update_flag=UI_MODIFY;
        }break;
    }
}

uint32_t static_cnt=0;

#define LINE_HIGH 40
#define LINE1_LEFT_X 196
#define LINE1_LEFT_Y 1080-381
#define LINE1_RIGHT_X 257
#define LINE1_RIGHT_Y 1080-403
#define UI_START_LEN_X 75
#define LINE2_RIGHT_X LINE1_RIGHT_X
#define LINE2_LEFT_X LINE1_LEFT_X
#define LINE2_LEFT_Y LINE1_LEFT_Y-LINE_HIGH*1
#define LINE2_RIGHT_Y LINE1_RIGHT_Y-LINE_HIGH*1

static void robot_status_draw(){
    ext_graphic_five_data_t ui_mode;//���ͱ���
    static Graphic_Color COLOR=UI_FUCHSIA;
    //����ͨ��֡ͷ����

    ui_mode.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_mode.txFrameHeader.data_length= sizeof (ext_student_interactive_header_data_t) +
                                       sizeof(ui_graphic_data_struct_t)*5;
    ui_mode.txFrameHeader.seq=0;    //��������

    //����ͼ�����ݸ�ֵ����xxx_data_struct��


    memcpy(ClientTxBuffer, &ui_mode.txFrameHeader, sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_mode.CmdID= Referee_ID_robot_interactive_header_data;

    //����֡ͷ
    ui_mode.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_mode.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_mode.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_five_graphic;

    //����ģʽ
    switch(ui_robot_status.chassis_mode)
    {
        case CHASSIS_RELAX:{
            Figure_Graphic(&ui_mode.clientData[0],"MD0",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE1_LEFT_X,LINE1_LEFT_Y,
                           0,LINE1_RIGHT_X,LINE1_RIGHT_Y);
        }break;

        case CHASSIS_FOLLOW_GIMBAL:{
            Figure_Graphic(&ui_mode.clientData[0],"MD0",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE1_LEFT_X+UI_START_LEN_X*1,LINE1_LEFT_Y,
                           0,LINE1_RIGHT_X+UI_START_LEN_X*1,LINE1_RIGHT_Y);
        }break;

        case CHASSIS_SPIN:{
            Figure_Graphic(&ui_mode.clientData[0],"MD0",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE1_LEFT_X+UI_START_LEN_X*2,LINE1_LEFT_Y,
                           0,LINE1_RIGHT_X+UI_START_LEN_X*2,LINE1_RIGHT_Y);
        }break;

//        case CHASSIS_BLOCK:{
//            Figure_Graphic(&ui_mode.clientData[0],"MD0",one_layer_update_flag,UI_RECTANGLE,
//                           UI_ONE_LAYER,COLOR,0,0,3,LINE1_LEFT_X+UI_START_LEN_X*3,LINE1_LEFT_Y,
//                           0,LINE1_RIGHT_X+UI_START_LEN_X*3,LINE1_RIGHT_Y);
//        }break;

        default:{
            Figure_Graphic(&ui_mode.clientData[0],"MD0",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE1_LEFT_X,LINE1_LEFT_Y,
                           0,LINE1_RIGHT_X,LINE1_RIGHT_Y);
        }break;

    }

    //��̨ģʽ
    switch(ui_robot_status.gimbal_mode)
    {
        case GIMBAL_RELAX:{
            Figure_Graphic(&ui_mode.clientData[1],"MD1",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE2_LEFT_X,LINE2_LEFT_Y,
                           0,LINE2_RIGHT_X,LINE2_RIGHT_Y);
        }break;

        case GIMBAL_ACTIVE:{
            Figure_Graphic(&ui_mode.clientData[1],"MD1",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE2_LEFT_X+UI_START_LEN_X,LINE2_LEFT_Y,
                           0,LINE2_RIGHT_X+UI_START_LEN_X,LINE2_RIGHT_Y);
        }break;

//        case GIMBAL_AUTO:{
//            Figure_Graphic(&ui_mode.clientData[1],"MD1",one_layer_update_flag,UI_RECTANGLE,
//                           UI_ONE_LAYER,COLOR,0,0,3,LINE2_LEFT_X+UI_START_LEN_X*2,LINE2_LEFT_Y,
//                           0,LINE2_RIGHT_X+UI_START_LEN_X*2,LINE2_RIGHT_Y);
//        }break;
//
//        case GIMBAL_BUFF:{
//            Figure_Graphic(&ui_mode.clientData[1],"MD1",one_layer_update_flag,UI_RECTANGLE,
//                           UI_ONE_LAYER,COLOR,0,0,3,LINE2_LEFT_X+UI_START_LEN_X*3,LINE2_LEFT_Y,
//                           0,LINE2_RIGHT_X+UI_START_LEN_X*3,LINE2_RIGHT_Y);
//        }break;

        default:{
            Figure_Graphic(&ui_mode.clientData[1],"MD1",one_layer_update_flag,UI_RECTANGLE,
                           UI_ONE_LAYER,COLOR,0,0,3,LINE2_LEFT_X,LINE2_LEFT_Y,
                           0,LINE2_RIGHT_X,LINE2_RIGHT_Y);
        }break;

    }
    //TODO:δ�����
    //pitch�Ƕ�
    Float_Graphic(&ui_mode.clientData[2],"PVA",one_layer_update_flag,UI_FLOAT,
                  UI_ONE_LAYER,UI_WHITE,15,2,2,680,1080-538,
                  ui_robot_status.pitch_value);
    //������Խǲ���
    Float_Graphic(&ui_mode.clientData[3],"YVA",one_layer_update_flag,UI_FLOAT,
                  UI_ONE_LAYER,UI_WHITE,15,2,2,680,1080-588,
                  ui_robot_status.relative_yaw_value);

    Figure_Graphic(&ui_mode.clientData[4],"MD4",UI_NONE,UI_RECTANGLE,
                   UI_ONE_LAYER,UI_ORANGE,0,0,2,320,620,
                   0,320+UI_CHAR_LENGTH*4,241-UI_CHAR_LENGTH);

    //����֡ͷ���ַ��뻺����
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_mode.CmdID,
           Referee_LEN_CMD_ID+ui_mode.txFrameHeader.data_length);
    //֡βCRC16����
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_mode));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_mode));

}
void UI_all_draw(){
    static int step=-1;
    step++;
    switch (step) {
        case 0:{
            static_char_draw();//��̬�ַ�
        }break;

        case 1:{
            aimLine_cap_draw();//����׼�ߺ͵���
        }break;

        case 2:{
            self_line_draw();//��ȫ��
        }break;

        case 3:{
            small_aimLine_draw();//Сװ�װ���׼
        }break;

        case 4:{
            robot_status_draw();//������ģʽ�л�
        }
        default:
        {
            step=-1;
        }break;
    }

}

void UI_dynamic_draw(){
    static int dynamic_step=-1;//���ƶ�̬Ԫ�ز���
    //��̬Ԫ�ػ滭
    dynamic_step++;
    switch (dynamic_step) {

        case 0:{
            aimLine_cap_draw();
        }break;

        case 1:
        {
            small_aimLine_draw();
        }break;

        case 2:{
            self_line_draw();
        }break;

        case 3:{
            robot_status_draw();
        }
        default:{
            dynamic_step=-1;
        }break;
    }


}
static void UI_reset_flag()
{
    static_update_flag=UI_ADD;
    one_layer_update_flag=UI_ADD;
    two_layer_update_flag=UI_ADD;
}


static void UI_set_flag()
{
    static_update_flag=UI_NONE;
    one_layer_update_flag=UI_MODIFY;
    two_layer_update_flag=UI_MODIFY;
}

//��Ϊ��̬�;�̬��B��һ��ˢ������ͼ��
void UI_parse_info()
{

}

void UI_paint_task(void const*argument){
    vTaskDelay(20);
    bool isReset=false;

    while (1) {

        //ͨ����������ˢ��
        if (KeyBoard.B.click_flag == 1)//B���һ�¸���һ������ͼ��
        {
            UI_reset_flag();
            isReset=true;
            static_cnt = HAL_GetTick();//��¼��ʱ�䣬����ʱ���͸�ΪMODIFY
            KeyBoard.B.click_flag = 0;
        }

        if (isReset)//true  ˢ������Ԫ��
        {
            UI_all_draw();
            if (HAL_GetTick() - static_cnt > 3000) {//Ŀǰ3000����ˢ��ȫ��Ԫ��
                isReset=false;//��ʾ���»������
            }
        }
        else {
            UI_set_flag();
            UI_dynamic_draw();
        }


        vTaskDelay(100);//20ms ���̹��ʷ���Ƶ����50HZ
    }
}