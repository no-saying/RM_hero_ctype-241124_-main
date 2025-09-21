// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buffer.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
// @todo 8191转换成360的精度太低,会损失精度
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
static buf_t *buffer_yaw, *buffer_pitch;
static Robot_Status_e robot_state; // 机器人整体工作状态
static uint8_t flag=1;
static float aligned_total_yaw, aligned_total_pitch;
static uint8_t DOC_MOUSE_L;
static uint8_t REVERSE_FLAG;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    buffer_yaw = BUFRegister();
    buffer_pitch = BUFRegister();

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    chassis_cmd_send.offset_angle = chassis_cmd_send.offset_angle < 180 ? chassis_cmd_send.offset_angle : -(360 - chassis_cmd_send.offset_angle);
    chassis_cmd_send.offset_angle = chassis_cmd_send.offset_angle > -180 ? chassis_cmd_send.offset_angle : (360 + chassis_cmd_send.offset_angle);
    // if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    // else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    // else
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left) || vision_recv_data->yaw || vision_recv_data->pitch) // 左侧开关状态[中],视觉模式
    {
        //  待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
        if(vision_recv_data->yaw < 50 || 
           vision_recv_data->pitch < 50 || 
           vision_recv_data->yaw > -50 || 
           vision_recv_data->pitch > -50) // 异常数据判断
        {
            gimbal_cmd_send.yaw = aligned_total_yaw - vision_recv_data->yaw;
            gimbal_cmd_send.pitch = aligned_total_pitch - vision_recv_data->pitch;
        }
    }

    // 云台参数,确定云台控制数据

    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || !vision_recv_data->yaw || !vision_recv_data->pitch)
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        if(gimbal_cmd_send.yaw - aligned_total_yaw < 360 || gimbal_cmd_send.yaw - aligned_total_yaw > -360)
        {
            gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_r_;
        }
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_r1;
    }
    // 云台软件限位
    gimbal_cmd_send.pitch = gimbal_cmd_send.pitch > PITCH_MIN_ANGLE ? gimbal_cmd_send.pitch : PITCH_MIN_ANGLE;
    gimbal_cmd_send.pitch = gimbal_cmd_send.pitch < PITCH_MAX_ANGLE ? gimbal_cmd_send.pitch : PITCH_MAX_ANGLE;
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = +40.0f * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
    chassis_cmd_send.vy = +40.0f * (float)rc_data[TEMP].rc.rocker_l1; // 1数值方向


    
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;     
    }                                              
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } 
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_LEG;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; 
        // gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        // gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        // gimbal_cmd_send.gimbal_mode =  GIMBAL_DEBUG_MODE;
    }


    // 发射参数
    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)//连发
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else if(rc_data[TEMP].rc.dial < -300)//连发
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 4;

    
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vy = -rc_data[TEMP].key[KEY_PRESS].s * 12000 + rc_data[TEMP].key[KEY_PRESS].w * 12000; // 系数待测
    chassis_cmd_send.vx = -rc_data[TEMP].key[KEY_PRESS].d * 12000 + rc_data[TEMP].key[KEY_PRESS].a * 12000;
    if(gimbal_cmd_send.yaw - aligned_total_yaw < 360 || gimbal_cmd_send.yaw - aligned_total_yaw > -360)
    {
        gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    }
    gimbal_cmd_send.pitch -= (float)rc_data[TEMP].mouse.y / 660 * 10;
    shoot_cmd_send.fair_flag = rc_data[TEMP].mouse.press_l;
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速（未启用）
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        chassis_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        chassis_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        chassis_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data [TEMP].mouse.press_l) // 右键设置发射模式
    {
        case 0: 
            DOC_MOUSE_L=0;
            REVERSE_FLAG=1;
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        case 1:
            if(DOC_MOUSE_L==0)
            { 
                REVERSE_FLAG=0;
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }           
            else
            {
                shoot_cmd_send.load_mode = LOAD_STOP;
                REVERSE_FLAG=1;
            } 
            DOC_MOUSE_L=1;
            break;
    }
    if(REVERSE_FLAG==1)
    {
        switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键退弹
        {
            case 0:
                REVERSE_FLAG=0;
                shoot_cmd_send.load_mode=LOAD_STOP;
                break;
            default:          
                shoot_cmd_send.load_mode=LOAD_REVERSE;
                break; 
        }   
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        chassis_cmd_send.friction_mode = 0;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        chassis_cmd_send.friction_mode = 1;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch(rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 3)
    {
        case 0:
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            break;
        case 1:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case 2:
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        default:
            break;
    }
    switch (rc_data[TEMP].mouse.press_r) // 右键设置自瞄
    {
        case 1:
            gimbal_cmd_send.yaw = aligned_total_yaw - vision_recv_data->yaw;
            gimbal_cmd_send.pitch = aligned_total_pitch - vision_recv_data->pitch;
            break;
        default:

            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
    gimbal_cmd_send.pitch = gimbal_cmd_send.pitch > PITCH_MIN_ANGLE ? gimbal_cmd_send.pitch : PITCH_MIN_ANGLE;
    gimbal_cmd_send.pitch = gimbal_cmd_send.pitch < PITCH_MAX_ANGLE ? gimbal_cmd_send.pitch : PITCH_MAX_ANGLE;
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        // LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        // LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    //flag = ~flag;
    
    aligned_total_yaw = BUFUpdata(buffer_yaw, gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, 20);
    aligned_total_pitch = BUFUpdata(buffer_pitch, gimbal_fetch_data.gimbal_imu_data.Pitch, 20);
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)
    vision_send_data.enemy_color = chassis_fetch_data.enemy_color;
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}
