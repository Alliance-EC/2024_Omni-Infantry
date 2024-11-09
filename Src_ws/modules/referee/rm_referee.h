#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
// #include "robot_def.h"

// 屏幕宽度
#define SCREEN_WIDTH 1080
// 屏幕长度
#define SCREEN_LENGTH 1920

// 步兵机器人不同等级对应的功率（功率优先）
#define robot_power_level_1     60
#define robot_power_level_2     65
#define robot_power_level_3     70
#define robot_power_level_4     75
#define robot_power_level_5     80
#define robot_power_level_6     85
#define robot_power_level_7     90
#define robot_power_level_8     95
#define robot_power_level_9to10 100
#define robot_power_level_MAX   200 // 当机器人满级且获得最大功率增益（步兵机器人的功率上限）

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;        // 机器人颜色
    uint16_t Robot_ID;          // 本机器人ID
    uint16_t Cilent_ID;         // 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState;                            // 0x0001
    ext_game_result_t GameResult;                          // 0x0002
    ext_game_robot_HP_t GameRobotHP;                       // 0x0003
    ext_event_data_t EventData;                            // 0x0101
    ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
    ext_game_robot_state_t GameRobotState;                 // 0x0201
    ext_power_heat_data_t PowerHeatData;                   // 0x0202
    ext_game_robot_pos_t GameRobotPos;                     // 0x0203
    ext_buff_musk_t BuffMusk;                              // 0x0204
    aerial_robot_energy_t AerialRobotEnergy;               // 0x0205
    ext_robot_hurt_t RobotHurt;                            // 0x0206
    ext_shoot_data_t ShootData;                            // 0x0207
    projectile_allowance_t ProjectileAllowance;            // 0x0208

    // 自定义交互数据的接收
    Communicate_ReceiveData_t ReceiveData;

    uint8_t init_flag;
} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
// typedef struct
// {
//     Referee_Interactive_Flag_t Referee_Interactive_Flag;
//     // 为UI绘制以及交互数据所用
//     chassis_mode_e chassis_mode;             // 底盘模式
//     shoot_mode_e shoot_mode;                 // 发射模式设置
//     friction_mode_e friction_mode;           // 摩擦轮关闭
//     uint8_t auto_rune;                       // 打符标志

//     // 上一次的模式，用于flag判断
//     chassis_mode_e chassis_last_mode;
//     shoot_mode_e shoot_last_mode;
//     friction_mode_e friction_last_mode;
//     uint8_t last_rune;

// } Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统硬件初始化
 *
 * @param referee_usart_handle
 * @return referee_info_t*
 */
referee_info_t *RefereeHardwareInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H