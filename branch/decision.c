/**
  ******************************************************************************
  * @file           : decision.c
  * @version        : v2.1
  * @author         : 周灿
  * @date           : 2025-8-1
  * @brief          : 哨兵电控决策模块
  * @attention      :
  *   宏配置说明:
  *     - RED_START_NAVIGATION   红方起始建图(切换红蓝方时修改)
  *     - if_shoot_Engineer      是否击打工程(每场前确定)
  *   坐标系说明:
  *     1. 云台手标点坐标     -> map_control_fill()
  *     2. 小地图路径提示     -> Path_display
  *     3. 导航点推送         -> Navigation_Tx_Send(&navigation_tx)
  *     4. 哨兵位置为里程计坐标(相对程序起始点而非零点)
  ******************************************************************************
  */

/* ============================== 头文件包含 ============================== */
#include "decision.h"
#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include "referee.h"
#include "stdbool.h"
#include "Nautilus_Vision.h"
#include "CAN_receive.h"
#include "navigation.h"
#include "struct_typedef.h"
#include "Nautilus_UI.h"
#include "string.h"
#include "math.h"
#include "ins_task.h"
#include "Sentry_cmd.h"

/* ======================== 编译开关 ======================== */
#define RED_START_NAVIGATION     /* 红方建图(蓝方建图时注释此行) */
//#define if_shoot_Engineer       /* 是否击打工程车(每场前确定) */

/* ======================== 全局变量定义 ======================== */

/* 决策核心实例 */
decision_t   decision;
Sentry_cmd_t Sentry_cmd_send;

/* 导航点位坐标表(全局坐标系, 14个点位) */
float Red_Navi_position[DECISION_POSITION_NUM][2] =
{
    {  0.00f,  0.00f },   /* [ 0] 初始化放置点 */
    {  2.42f, -2.35f },   /* [ 1] 己方补给区(训练) */
    { 10.35f, 14.27f },   /* [ 2] 己方飞坡落点 */
    {  8.50f,  7.50f },   /* [ 3] 己方堡垒 */
    {  8.50f,  7.50f },   /* [ 4] 己方防守点 */
    {  6.65f,  9.10f },   /* [ 5] 己方堡垒前方 */
    {  6.28f,  6.84f },   /* [ 6] 己方堡垒右侧 */
    { 15.75f,  9.39f },   /* [ 7] 敌方前哨站 */
    { 21.18f,  5.67f },   /* [ 8] 敌方英雄打前哨站防护点 */
    { 23.80f,  7.59f },   /* [ 9] 敌方英雄吊射防护点 */
    { 11.43f,  4.36f },   /* [10] 己方前哨站 */
    { 21.18f,  5.67f },   /* [11] 敌方堡垒 */
    {  4.00f, -3.00f },   /* [12] 高地背敌处(训练) */
    {  8.90f,  7.50f },   /* [13] 云台手标点 */
};

/* ======================== 内部状态变量 ======================== */

/* --- 受击计时 --- */
static int hurt_time = 0;

/* --- 导航到达判断 --- */
static uint8_t  if_close_to_des = 0;
static float    navigation_x, navigation_y;
static float    current_x, current_y;

/* --- 补给相关 --- */
static uint16_t last_allowance_17  = 300;
static uint16_t allow_to_get_17mm  = 0;
static uint16_t already_allowance_17 = 0;
static uint8_t  remain_time        = 0;

/* --- 基地护甲 --- */
uint16_t last_dart_time   = 0;
uint16_t lock_dart_count  = 0;
uint8_t  if_random_dart   = 0;

/* --- 基地吊射判断 --- */
uint16_t Last_base_hurt_time = 420;

/* --- 敌方小能量机关 --- */
static uint16_t enemy_small_energy_time = 500;

/* ================================================================
 *                    核心任务与初始化
 * ================================================================ */

/**
  * @brief  自主决策主任务(FreeRTOS线程入口)
  * @param  argument : FreeRTOS线程参数(未使用)
  * @note   循环周期约1ms, 依次执行:
  *         决策状态->射击决策->裁判数据->指令处理->导航填充->推送导航
  */
void Auto_run(void const *argument)
{
    Decision_Init(&decision);
    hurt_time = 420;

    decision.If_point_change = 0;
    memset(&decision, 0, sizeof(decision_t));
    decision.Cmd_condition.if_update = 1;
    memset(&Sentry_cmd_send, 0, sizeof(Sentry_cmd_t));

#ifdef if_shoot_Engineer
    Red_Navi_position[ENEMY_OUTPOST_PROTECT_POINT][0] = 12.09f;
    Red_Navi_position[ENEMY_OUTPOST_PROTECT_POINT][1] = 8.90f;
#endif

    for (;;)
    {
        Decison_State_Ctl(&decision);
        sentry_shoot_decision(&decision);
        get_referr_data();
        Sentry_cmd_decision(&decision);
        decision_point_fill();
        Navigation_Tx_Send(&navigation_tx);
        vTaskDelay(1);
    }
}

/**
  * @brief  根据遥控器开关组合选择决策模式
  * @param  mode : 决策结构体指针
  * @note   仅在比赛进行中(game_progress==4)响应遥控器
  *         开关映射: s_l=1,s_r=1激进; s_l=3,s_r=3保守;
  *                   s_l=3,s_r=2飞坡; s_l=3,s_r=1巡逻; s_l=1,s_r=3保护
  */
void AGV_auto_mode(decision_t *mode)
{
    if (game_state.game_progress == 4)
    {
        if (rc_ctrl.rc.s_l == 1 && rc_ctrl.rc.s_r == 1)
            mode->decision_mode = extreme;
        else if (rc_ctrl.rc.s_l == 3 && rc_ctrl.rc.s_r == 3)
            mode->decision_mode = conservative;
        else if (rc_ctrl.rc.s_l == 3 && rc_ctrl.rc.s_r == 2)
            mode->decision_mode = flying;
        else if (rc_ctrl.rc.s_l == 3 && rc_ctrl.rc.s_r == 1)
            mode->decision_mode = patrol;
        else if (rc_ctrl.rc.s_l == 1 && rc_ctrl.rc.s_r == 3)
            mode->decision_mode = protect;
    }
    else
    {
        mode->decision_mode = extreme;
    }
}

/**
  * @brief  从裁判系统获取机器人颜色信息
  * @note   根据robot_id判断: 1~9红方, >=101蓝方, 其他未连接
  */
void get_referr_data(void)
{
    if (robot_status.robot_id <= 9 && robot_status.robot_id > 0)
        decision.robot_data.robot_color = red;
    else if (robot_status.robot_id >= 101)
        decision.robot_data.robot_color = blue;
    else
        decision.robot_data.robot_color = NO_CONTACT;

    if (decision.robot_data.robot_color == red)
        Robot_ID = UI_Data_RobotID_RSentry;
    else
        Robot_ID = UI_Data_RobotID_BSentry;
}

static uint16_t Last_HP;
static uint16_t Last_projectile_allowance_17mm;

/* ================================================================
 *                   哨兵自主指令处理
 * ================================================================ */

/**
  * @brief  哨兵自主决策指令处理(复活/兑换弹丸/兑换血量)
  */
void Sentry_cmd_decision(decision_t *mode)
{
    if (game_state.game_progress == 4)
    {
        mode->Cmd_condition.Exchange_Projectile_Num = 0;
        mode->Cmd_condition.If_remote_exchange_HP  = 0;
        mode->Cmd_condition.If_Immediately_Revive   = 0;

        if (robot_status.current_HP == 0 && Last_HP > 0)
            mode->Cmd_condition.Die_cnt++;

        mode->Cmd_condition.If_revive = 1;

        Sentry_Cmd_Fill(&Sentry_cmd_send,
                        mode->Cmd_condition.If_revive,
                        mode->Cmd_condition.If_Immediately_Revive,
                        mode->Cmd_condition.Exchange_Projectile_Num,
                        mode->Cmd_condition.If_remote_exchange_HP);

        Last_HP = robot_status.current_HP;
        Last_projectile_allowance_17mm = projectile_allowance.projectile_allowance_17mm;
    }
    else
    {
        memset(&mode->Cmd_condition, 0, sizeof(Cmd_condition_t));
        Sentry_Cmd_Fill(&Sentry_cmd_send,
                        mode->Cmd_condition.If_revive,
                        mode->Cmd_condition.If_Immediately_Revive,
                        mode->Cmd_condition.Exchange_Projectile_Num,
                        mode->Cmd_condition.If_remote_exchange_HP);
    }
}

/**
  * @brief  决策模块初始化, 设置初始点位为INIT_PACK_POINT
  */
void Decision_Init(decision_t *mode)
{
    mode->point = INIT_PACK_POINT;
}

/* ================================================================
 *                    决策状态主控制器
 * ================================================================ */

/**
  * @brief  决策状态主控制器(每周期调用)
  * @note   比赛进行中: 更新判决条件->遥控器选模式->执行点位决策
  *         非比赛状态: 强制设为目标条件满足, 复位所有状态
  */
void Decison_State_Ctl(decision_t *mode)
{
    if (game_state.game_progress == 4)
    {
        Judge_Continuous_Handle(&decision.Judge_condition);
        AGV_auto_mode(mode);
        decision_point_chose(mode);
    }
    else
    {
        mode->decision_mode = extreme;

        If_Point_arrived();
        judge_if_location_over(&decision.Judge_condition);

        mode->Judge_condition.IF_10s_NotHurted  = 1;
        mode->Judge_condition.IF_3s_NotFound    = 1;
        mode->Judge_condition.IF_5s_NotFound    = 1;
        mode->Judge_condition.IF_10s_NotFound   = 1;
        mode->Judge_condition.IF_HP_Less_50     = 0;
        mode->Judge_condition.IF_HP_Less_100    = 0;
        mode->Judge_condition.IF_outpost_destroyed = 0;
        mode->Judge_condition.IF_fire_lock      = 0;
        mode->Judge_condition.IF_allowance_less_50  = 0;
        mode->Judge_condition.IF_HP_recover     = 1;
        mode->Judge_condition.If_enemy_outpost_lock = 0;

        judge_if_on_toss(&mode->Judge_condition);
        judge_if_moving_v(&mode->Judge_condition);

        mode->Judge_condition.If_chassis_weak    = 0;
        mode->Judge_condition.IF_need_to_protect = 0;
        judge_if_need_to_protect(&decision.Judge_condition);

        mode->Judge_condition.IF_base_armor_spred  = 0;
        mode->Judge_condition.If_fortress_free     = 1;
        mode->Judge_condition.If_get_allow_17      = 0;
        mode->Judge_condition.If_chip_base         = 0;
        mode->Judge_condition.IF_3s_NotHurted      = 1;
        mode->Judge_condition.IF_5s_NotHurted      = 1;
        mode->Judge_condition.If_close_to_enemy_out = 0;

        judge_if_moving_v(&mode->Judge_condition);

        lock_dart_count   = 0;
        if_random_dart    = 0;
        last_dart_time    = 0;
        if_update          = 0;
        hurt_time          = 430;
        Last_base_hurt_time = 430;
        decision.keyboard_disable = 0;
        memset(&map_command, 0, sizeof(map_command));

        mode->point = INIT_PACK_POINT;
    }
}

/* ================================================================
 *                    辅助判断函数
 * ================================================================ */

/**
  * @brief  判断是否距离敌方前哨站较近
  */
void judge_if_close_to_enemy_out(Judge_condition_t *mode)
{
    if ((fabs(navigation_rx.current_x - navigation_tx.navi_set_x_pos) < 1.5f
      && fabs(navigation_rx.current_y - navigation_tx.navi_set_y_pos) < 1.5f)
      || mode->IF_Arrived == 1)
        mode->If_close_to_enemy_out = 1;
    else
        mode->If_close_to_enemy_out = 0;
}

/**
  * @brief  判断键盘一键失能/使能, D键失能, W键恢复
  */
void judge_if_keyboard_disable(decision_t *mode)
{
    if (if_update == 1 && map_command.cmd_keyboard == 'D')
        mode->keyboard_disable = 1;
    else if (if_update == 1 && map_command.cmd_keyboard == 'W')
        mode->keyboard_disable = 0;
}

/**
  * @brief  测试用简易决策(血量低回补给区, 否则去高地)
  */
void sentry_test_decision(decision_t *mode)
{
    if (mode->Judge_condition.IF_HP_Less_100)
        mode->point = WE_DEPOT_POINT;
    else
        mode->point = CENTRL_HIGH_POINT;
}

uint8_t if_navi_receive = 0;

/* ================================================================
 *                    决策点位选择与路由
 * ================================================================ */

/**
  * @brief  决策点位选择(云台手标点->模式路由)
  * @note   云台手更新点位时切换到AIR_CONTROL模式
  *         决策模式切换时将点位复位到初始点
  */
void decision_point_chose(decision_t *mode)
{
    if (if_update == 1)
    {
        mode->decision_mode = air_control;
        judge_if_keyboard_disable(mode);
    }

    if (if_update == 1 && if_map_correct == 1)
        map_control_fill(mode);

    if (mode->decision_mode != mode->last_decision_mode)
    {
        mode->last_decision_mode = mode->decision_mode;
        mode->point = INIT_PACK_POINT;
    }

    sentry_test_decision(mode);
}

/* ================================================================
 *                各决策模式实现
 * ================================================================ */

void sentry_air_control_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case MANUAL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100 == 1
             || mode->Judge_condition.If_get_allow_17 == 1)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived == 1
             && mode->Judge_condition.IF_HP_recover == 1)
            {
                mode->point = MANUAL_POINT;
            }
            break;
        }
        default:
        {
            mode->point = MANUAL_POINT;
            break;
        }
    }
}

/**
  * @brief  激进模式决策(主战模式)
  * @note   主动进攻敌方前哨站/堡垒, 根据血量/弹药/护甲等条件流转
  */
void sentry_extreme_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = ENEMY_OUTPOST_POINT;
            break;
        }
        case ENEMY_OUTPOST_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (!mode->Judge_condition.IF_3s_NotHurted
                  && mode->Judge_condition.If_hp_less_200)
            {
                mode->point = CENTRL_HIGH_POINT;
            }
            else if (mode->Judge_condition.IF_HP_Less_100
                  && !mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_OUTPOST_POINT;
            }
            else if ((mode->Judge_condition.IF_outpost_destroyed
                   && mode->Judge_condition.IF_need_to_protect)
                  || mode->Judge_condition.IF_base_armor_spred)
            {
                mode->point = WE_PATROL_POINT;
            }
            else if (mode->Judge_condition.IF_enemy_outpost_destroyed
                  && !mode->Judge_condition.IF_allowance_less_50
                  && !mode->Judge_condition.If_chassis_weak)
            {
                mode->point = ENEMY_OUTPOST_PROTECT_POINT;
            }
            else if (mode->Judge_condition.IF_enemy_outpost_destroyed
                  && mode->Judge_condition.IF_allowance_less_50)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived == 1
             && mode->Judge_condition.IF_HP_recover == 1)
            {
                if (mode->Judge_condition.If_chassis_weak)
                {
                    mode->point = WE_PATROL_POINT;
                }
                else if ((mode->Judge_condition.IF_outpost_destroyed
                       && mode->Judge_condition.IF_need_to_protect)
                      || mode->Judge_condition.IF_base_armor_spred)
                {
                    mode->point = WE_FORTRESS_POINT;
                }
                else if (!mode->Judge_condition.IF_enemy_outpost_destroyed
                      && !mode->Judge_condition.IF_allowance_less_100)
                {
                    mode->point = ENEMY_OUTPOST_POINT;
                }
                else
                {
                    mode->point = WE_PATROL_POINT;
                }
            }
            break;
        }
        case WE_PATROL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (!mode->Judge_condition.If_chassis_weak)
            {
                if ((mode->Judge_condition.IF_outpost_destroyed
                  && mode->Judge_condition.IF_need_to_protect)
                 || mode->Judge_condition.IF_base_armor_spred)
                {
                    mode->point = WE_FORTRESS_POINT;
                }
                else if (!mode->Judge_condition.IF_allowance_less_100)
                {
                    mode->point = ENEMY_OUTPOST_POINT;
                }
            }
            break;
        }
        case CENTRL_HIGH_POINT:
        {
            if (mode->Judge_condition.IF_3s_NotHurted)
            {
                mode->point = ENEMY_OUTPOST_POINT;
            }
            break;
        }
        case WE_FORTRESS_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (!mode->Judge_condition.IF_need_to_protect
                  && !mode->Judge_condition.IF_base_armor_spred)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        case WE_OUTPOST_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_HP_recover)
            {
                mode->point = ENEMY_OUTPOST_POINT;
            }
            break;
        }
        case ENEMY_OUTPOST_PROTECT_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_HP_Less_100
                  && !mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_OUTPOST_POINT;
            }
            else if ((mode->Judge_condition.IF_outpost_destroyed
                   && mode->Judge_condition.IF_need_to_protect)
                  || mode->Judge_condition.IF_base_armor_spred)
            {
                mode->point = WE_PATROL_POINT;
            }
            else if (mode->Judge_condition.If_need_to_enemy_fortress)
            {
                mode->point = ENEMY_FORTRESS_POINT;
            }
            break;
        }
        case ENEMY_FORTRESS_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_HP_Less_100
                  && !mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_OUTPOST_POINT;
            }
            else if (!mode->Judge_condition.If_need_to_enemy_fortress)
            {
                mode->point = ENEMY_OUTPOST_PROTECT_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/**
  * @brief  保守模式决策: 不主动上敌方堡垒, 优先防守己方区域
  */
void sentry_conservative_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = WE_FLYING_POINT;
            break;
        }
        case WE_FLYING_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_enemy_outpost_destroyed)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        case WE_PATROL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_FORTRESS_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived
             && mode->Judge_condition.IF_HP_recover)
            {
                if (!mode->Judge_condition.IF_enemy_outpost_destroyed)
                {
                    mode->point = WE_FLYING_POINT;
                }
                else if (!mode->Judge_condition.IF_outpost_destroyed)
                {
                    mode->point = WE_PATROL_POINT;
                }
                else
                {
                    mode->point = WE_FORTRESS_POINT;
                }
            }
            break;
        }
        case WE_FORTRESS_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/**
  * @brief  巡逻模式决策: 只打前哨站, 不进入敌方半场深处
  */
void sentry_patrol_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = ENEMY_OUTPOST_POINT;
            break;
        }
        case ENEMY_OUTPOST_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (!mode->Judge_condition.IF_5s_NotHurted
                  && mode->Judge_condition.If_hp_less_200)
            {
                mode->point = CENTRL_HIGH_POINT;
            }
            else if (mode->Judge_condition.IF_enemy_outpost_destroyed)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived
             && mode->Judge_condition.IF_HP_recover)
            {
                if (mode->Judge_condition.IF_allowance_less_50)
                {
                    mode->point = WE_PATROL_POINT;
                }
                else if (!mode->Judge_condition.IF_outpost_destroyed)
                {
                    mode->point = ENEMY_OUTPOST_POINT;
                }
            }
            break;
        }
        case WE_PATROL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        case CENTRL_HIGH_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_5s_NotHurted)
            {
                mode->point = ENEMY_OUTPOST_POINT;
            }
            else if (mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/**
  * @brief  飞坡模式决策: 飞坡落点+打前哨站
  */
void sentry_flying_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = ENEMY_OUTPOST_POINT;
            break;
        }
        case ENEMY_OUTPOST_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (!mode->Judge_condition.IF_5s_NotHurted
                  && mode->Judge_condition.If_hp_less_200)
            {
                mode->point = CENTRL_HIGH_POINT;
            }
            else if (mode->Judge_condition.IF_enemy_outpost_destroyed)
            {
                mode->point = WE_FLYING_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived
             && mode->Judge_condition.IF_HP_recover)
            {
                if (mode->Judge_condition.IF_allowance_less_50)
                {
                    mode->point = WE_FLYING_POINT;
                }
                else if (!mode->Judge_condition.IF_outpost_destroyed)
                {
                    mode->point = ENEMY_OUTPOST_POINT;
                }
            }
            break;
        }
        case WE_FLYING_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        case CENTRL_HIGH_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_50
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if (mode->Judge_condition.IF_5s_NotHurted)
            {
                mode->point = ENEMY_OUTPOST_POINT;
            }
            else if (mode->Judge_condition.IF_outpost_destroyed)
            {
                mode->point = WE_FLYING_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/**
  * @brief  保护模式决策(分区赛上场版本)
  */
void sentry_protect_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = WE_FLYING_POINT;
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived == 1
             && mode->Judge_condition.IF_HP_recover == 1)
            {
                if (mode->Judge_condition.IF_outpost_destroyed == 1)
                {
                    mode->point = WE_PATROL_POINT;
                }
                else
                {
                    mode->point = WE_FLYING_POINT;
                }
            }
            break;
        }
        case WE_FLYING_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100 == 1
             || mode->Judge_condition.If_get_allow_17 == 1)
            {
                mode->point = WE_DEPOT_POINT;
            }
            else if ((mode->Judge_condition.IF_outpost_destroyed == 1
                   || mode->Judge_condition.IF_enemy_outpost_destroyed)
                  && mode->Judge_condition.IF_3s_NotFound)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        case WE_PATROL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100 == 1
             || mode->Judge_condition.If_get_allow_17 == 1)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/**
  * @brief  两点决策模式: 防守点与补给区之间切换, 最简单的保底决策
  */
void sentry_two_point_decision(decision_t *mode)
{
    switch (mode->point)
    {
        case INIT_PACK_POINT:
        {
            mode->point = WE_PATROL_POINT;
            break;
        }
        case WE_PATROL_POINT:
        {
            if (mode->Judge_condition.IF_HP_Less_100
             || mode->Judge_condition.If_get_allow_17)
            {
                mode->point = WE_DEPOT_POINT;
            }
            break;
        }
        case WE_DEPOT_POINT:
        {
            if (mode->Judge_condition.IF_Arrived == 1
             && mode->Judge_condition.IF_HP_recover == 1)
            {
                mode->point = WE_PATROL_POINT;
            }
            break;
        }
        default:
        {
            mode->point = INIT_PACK_POINT;
            break;
        }
    }
}

/* ================================================================
 *                    云台手控制相关
 * ================================================================ */

void air_control_ctl(decision_t *mode)
{
    mode->point = MANUAL_POINT;
}

/**
  * @brief  云台手地图标点转导航坐标系
  * @note   红方建图直接使用, 蓝方坐标系翻转(28-x,15-y)
  */
void map_control_fill(decision_t *mode)
{
    (void)mode;
#ifdef RED_START_NAVIGATION
    Red_Navi_position[MANUAL_POINT][0] = map_command.target_position_x;
    Red_Navi_position[MANUAL_POINT][1] = map_command.target_position_y;
#else
    Red_Navi_position[MANUAL_POINT][0] = 28.0f - map_command.target_position_x;
    Red_Navi_position[MANUAL_POINT][1] = 15.0f - map_command.target_position_y;
#endif
}

/* ================================================================
 *                    导航点位填充
 * ================================================================ */

/**
  * @brief  计算导航目标坐标并写入navigation_tx
  * @note   普通点位计算相对偏移, 视觉锁定时根据装甲板距离实时推算
  */
void decision_point_fill(void)
{
    float diff_yaw, transform_angle;
    float dist_x, dist_y;
    float transform_x, transform_y;

    if (receive_gimbal_data.vision_state != vision_frount)
    {
#ifdef RED_START_NAVIGATION
        navigation_tx.navi_set_x_pos = Red_Navi_position[decision.point][0]
                                     - Red_Navi_position[INIT_PACK_POINT][0];
        navigation_tx.navi_set_y_pos = Red_Navi_position[decision.point][1]
                                     - Red_Navi_position[INIT_PACK_POINT][1];
#else
        if (decision.point != MANUAL_POINT)
        {
            if (decision.robot_data.robot_color == red)
            {
                navigation_tx.navi_set_x_pos = (28.0f - Red_Navi_position[decision.point][0])
                                             - Red_Navi_position[0][0];
                navigation_tx.navi_set_y_pos = (15.0f - Red_Navi_position[decision.point][1])
                                             - Red_Navi_position[0][1];
            }
            else
            {
                navigation_tx.navi_set_x_pos = Red_Navi_position[decision.point][0]
                                             - Red_Navi_position[INIT_PACK_POINT][0];
                navigation_tx.navi_set_y_pos = Red_Navi_position[decision.point][1]
                                             - Red_Navi_position[INIT_PACK_POINT][1];
            }
        }
        else
        {
            navigation_tx.navi_set_x_pos = Red_Navi_position[decision.point][0]
                                         - Red_Navi_position[INIT_PACK_POINT][0];
            navigation_tx.navi_set_y_pos = Red_Navi_position[decision.point][1]
                                         - Red_Navi_position[INIT_PACK_POINT][1];
        }
#endif
    }
    else if (receive_gimbal_data.vision_state == vision_frount
          && decision.point != WE_DEPOT_POINT)
    {
        transform_angle = receive_gimbal_data.current_transform_angle * 2.0f * 3.14f / 360.0f;
        dist_x = -receive_gimbal_data.armor_dist * cos(transform_angle);
        dist_y = -receive_gimbal_data.armor_dist * sin(transform_angle);

        diff_yaw = INS.Yaw - 0.0f;
        if (diff_yaw > 180.0f)       diff_yaw -= 360.0f;
        else if (diff_yaw < -180.0f) diff_yaw += 360.0f;

        float diff_angle = diff_yaw * 2.0f * 3.14f / 360.0f;
        transform_x = dist_x * cos(diff_angle) + dist_y * sin(diff_angle);
        transform_y = -dist_x * sin(diff_angle) + dist_y * cos(diff_angle);

        navigation_tx.navi_set_x_pos = navigation_rx.current_x + transform_x;
        navigation_tx.navi_set_y_pos = navigation_rx.current_y + (-transform_y);
    }
}

/* ================================================================
 *                    到达判断
 * ================================================================ */

/**
  * @brief  判断哨兵是否到达目标点位(误差<0.8m)
  */
void If_Point_arrived(void)
{
    if (decision.point != MANUAL_POINT)
    {
        navigation_x = Red_Navi_position[decision.point][0]
                     - Red_Navi_position[INIT_PACK_POINT][0];
        navigation_y = Red_Navi_position[decision.point][1]
                     - Red_Navi_position[INIT_PACK_POINT][1];

        if (fabs(navigation_x - navigation_rx.current_x) < 0.8f
         && fabs(navigation_y - navigation_rx.current_y) < 0.8f)
            if_close_to_des = 1;
        else
            if_close_to_des = 0;
    }
    else
    {
        navigation_x = Red_Navi_position[decision.point][0];
        navigation_y = Red_Navi_position[decision.point][1];

        if (decision.robot_data.robot_color == red)
        {
            current_x = navigation_rx.current_x + Red_Navi_position[INIT_PACK_POINT][0];
            current_y = navigation_rx.current_y + Red_Navi_position[INIT_PACK_POINT][1];
        }
        else
        {
            current_x = navigation_rx.current_x + (28.0f - Red_Navi_position[INIT_PACK_POINT][0]);
            current_y = navigation_rx.current_y + (15.0f - Red_Navi_position[INIT_PACK_POINT][1]);
        }

        if (fabs(navigation_x - current_x) < 0.8f
         && fabs(navigation_y - current_y) < 0.8f)
            if_close_to_des = 1;
        else
            if_close_to_des = 0;
    }

    if (navigation_rx.If_get_path == 0 || navigation_rx.if_lost_navi)
        navigation_rx.if_arrived = 1;
    else
        navigation_rx.if_arrived = 0;
}

/**
  * @brief  判断是否需要上敌方堡垒
  * @note   红方: 3/4号步兵x>17(敌方半场); 蓝方: 3/4号步兵x<11(敌方半场)
  */
void judge_if_need_to_enemy_fortress(Judge_condition_t *mode)
{
    if (decision.robot_data.robot_color == red)
        mode->If_need_to_enemy_fortress =
            ((ground_robot_position.standard_3_x > 17 && ground_robot_position.standard_3_x < 28)
          && (ground_robot_position.standard_4_x > 17 && ground_robot_position.standard_4_x < 28)) ? 1 : 0;
    else if (decision.robot_data.robot_color == blue)
        mode->If_need_to_enemy_fortress =
            ((ground_robot_position.standard_3_x > 0  && ground_robot_position.standard_3_x < 11)
          && (ground_robot_position.standard_4_x > 0  && ground_robot_position.standard_4_x < 11)) ? 1 : 0;
}

/**
  * @brief  决策点切换检测(预留, 暂未实现)
  */
void IF_decision_point_change(Judge_condition_t *mode)
{
    (void)mode;
}

/* ================================================================
 *                    状态判断函数
 * ================================================================ */

/**
  * @brief  判断基地是否在吊射状态(持续扣血>=40)
  */
void judge_if_chip_base(uint16_t base_hp, Judge_condition_t *mode)
{
    static uint16_t Last_base_hp;

    if ((Last_base_hp - base_hp >= 40) && mode->IF_need_to_protect == 0)
        mode->If_chip_base = 1;
    else if (base_hp == 5000 || mode->IF_need_to_protect == 1)
        mode->If_chip_base = 0;

    Last_base_hp = base_hp;
}

/**
  * @brief  判断是否需要回补给区补给17mm弹丸
  * @note   发弹量<50且视野丢失且可兑换>=200, 或发弹量<5且剩余<5s
  */
void judge_if_need_allow_17(Judge_condition_t *mode)
{
    if (projectile_allowance.projectile_allowance_17mm > last_allowance_17)
        already_allowance_17 += (projectile_allowance.projectile_allowance_17mm - last_allowance_17);

    allow_to_get_17mm = ((420 - game_state.stage_remain_time) / 60) * 100;
    remain_time = game_state.stage_remain_time % 60;

    if ((projectile_allowance.projectile_allowance_17mm < 50
      && receive_gimbal_data.Vision_look_state == vision_lost
      && (allow_to_get_17mm - already_allowance_17 >= 200))
     || (projectile_allowance.projectile_allowance_17mm < 5
      && remain_time < 5 && remain_time > 0))
        mode->If_get_allow_17 = 1;
    else
        mode->If_get_allow_17 = 0;

    last_allowance_17 = projectile_allowance.projectile_allowance_17mm;
}

/**
  * @brief  判断哨兵是否未受击超过阈值(3s/5s/10s)
  * @note   受击判定: 血量单次下降>=10
  */
bool judge_if_nothurt(Judge_condition_t *mode)
{
    static uint16_t Last_HP;

    if (Last_HP - robot_status.current_HP >= 10)
        hurt_time = game_state.stage_remain_time;

    mode->IF_10s_NotHurted = (hurt_time - game_state.stage_remain_time > 10) ? 1 : 0;
    mode->IF_5s_NotHurted  = (hurt_time - game_state.stage_remain_time > 5)  ? 1 : 0;
    mode->IF_3s_NotHurted  = (hurt_time - game_state.stage_remain_time > 3)  ? 1 : 0;

    Last_HP = robot_status.current_HP;
    return mode->IF_3s_NotHurted;
}

/**
  * @brief  判断哨兵是否未发现敌人超过阈值
  * @note   视觉锁定且已到达时重置, 点位切换时清零
  */
void judge_if_not_found(Judge_condition_t *mode)
{
    static int nfound_time = 0;
    static uint8_t last_point;

    if (receive_gimbal_data.vision_state != vision_lost
     && decision.Judge_condition.IF_Arrived == 1)
        nfound_time = 0;
    else
        nfound_time++;

    mode->IF_3s_NotFound  = (nfound_time > TIM_3S)  ? 1 : 0;
    mode->IF_5s_NotFound  = (nfound_time > TIM_5S)  ? 1 : 0;
    mode->IF_10s_NotFound = (nfound_time > TIM_10S) ? 1 : 0;

    if (decision.point != last_point)
    {
        mode->IF_3s_NotFound  = 0;
        mode->IF_5s_NotFound  = 0;
        mode->IF_10s_NotFound = 0;
    }
    last_point = decision.point;
}

void judge_if_location_over(Judge_condition_t *mode)
{
    mode->IF_Arrived = (navigation_rx.if_arrived == 1) ? 1 : 0;
}

void judge_if_allowance_less_50(Judge_condition_t *mode)
{
    mode->IF_allowance_less_50 = (projectile_allowance.projectile_allowance_17mm < 30) ? 1 : 0;
}

void judge_if_allowance_less_100(Judge_condition_t *mode)
{
    mode->IF_allowance_less_100 = (projectile_allowance.projectile_allowance_17mm < 70) ? 1 : 0;
}

void judge_if_HP_less_200(Judge_condition_t *mode)
{
    mode->If_hp_less_200 = (robot_status.current_HP <= 150) ? 1 : 0;
}

void judge_if_HP_less_100(Judge_condition_t *mode)
{
    mode->IF_HP_Less_100 = (robot_status.current_HP <= 110) ? 1 : 0;
}

void judge_if_HP_less_50(Judge_condition_t *mode)
{
    mode->IF_HP_Less_50 = (robot_status.current_HP <= 70) ? 1 : 0;
}

/**
  * @brief  判断己方基地护甲是否展开
  * @note   基地血量<=2200 或 有随机飞镖 或 飞镖命中计数达4次
  */
void judge_base_armor_spred(Judge_condition_t *mode, int16_t base_HP)
{
    if (last_dart_time != event_data.thelast_dart_hit_time
     && event_data.thelast_dart_hit_goal == 3)
        lock_dart_count++;

    if (event_data.thelast_dart_hit_goal == 4)
        if_random_dart = 1;

    if (event_data.thelast_dart_hit_goal == 3)
        last_dart_time = event_data.thelast_dart_hit_time;

    mode->IF_base_armor_spred =
        (base_HP <= 2200 || if_random_dart == 1 || lock_dart_count == 4) ? 1 : 0;
}

/**
  * @brief  判断敌方是否开启小能量机关
  * @note   前哨站单次掉5点且剩余>120s判定开启, 持续约45s后复位
  */
bool judge_if_enemy_small_energy(Judge_condition_t *mode, int16_t outpost_HP)
{
    static uint16_t Last_outpost_HP;

    if (outpost_HP - Last_outpost_HP == 5
     && mode->If_enemy_small_energy == 0
     && game_state.stage_remain_time > 120)
    {
        enemy_small_energy_time = game_state.stage_remain_time;
        mode->If_enemy_small_energy = 1;
    }

    if (mode->If_enemy_small_energy == 1
     && (enemy_small_energy_time - game_state.stage_remain_time >= 40))
        mode->If_enemy_small_energy = 0;

    Last_outpost_HP = outpost_HP;
    return mode->If_enemy_small_energy;
}

void judge_if_outpost_destroyed(Judge_condition_t *mode, int16_t outpost_HP)
{
    mode->IF_outpost_destroyed = (outpost_HP <= 250) ? 1 : 0;
}

void judge_if_fire_lock(Judge_condition_t *mode)
{
    if (robot_status.current_HP == 0
     && robot_status.power_management_shooter_output == 0)
        mode->IF_fire_lock = 1;
    else if (robot_status.power_management_shooter_output == 1)
        mode->IF_fire_lock = 0;
}

void judge_if_HP_recover(Judge_condition_t *mode)
{
    mode->IF_HP_recover = (robot_status.current_HP == 400) ? 1 : 0;
}

/* ================================================================
 *                    底盘速度控制
 * ================================================================ */

/**
  * @brief  根据导航与遇敌/受击情况自主选择底盘模式
  * @note   到达时根据受击切换陀螺速度; 导航中荒地/U型弯无陀螺, 受击时提高陀螺
  */
void AGV_auto_chassis(decision_t *mode)
{
    if (rc_ctrl.rc.s_l != 2)
    {
        if (mode->Judge_condition.IF_Arrived)
        {
            if (mode->Judge_condition.If_chassis_weak == 0)
            {
                if (mode->Judge_condition.IF_3s_NotHurted
                 && mode->point == ENEMY_OUTPOST_POINT)
                {
                    sentry_system.chassis_mode = normol_move;
                    sentry_system.chassis_set.Vz_state = no_spine;
                }
                else if (!mode->Judge_condition.IF_3s_NotHurted
                      && mode->point == ENEMY_OUTPOST_POINT)
                {
                    sentry_system.chassis_mode = normol_move;
                    sentry_system.chassis_set.Vz_state = low_spine;
                }
                else if (!mode->Judge_condition.IF_5s_NotHurted)
                {
                    sentry_system.chassis_mode = normol_move;
                    sentry_system.chassis_set.Vz_state = high_spine;
                }
                else
                {
                    sentry_system.chassis_mode = normol_move;
                    sentry_system.chassis_set.Vz_state = low_spine;
                }
            }
            else
            {
                sentry_system.chassis_mode = normol_move;
                sentry_system.chassis_set.Vz_state = low_spine;
            }
        }
        else
        {
            if (mode->Judge_condition.If_on_toss
             || (mode->Judge_condition.If_moving_v
              && mode->Judge_condition.IF_3s_NotHurted))
            {
                sentry_system.chassis_mode = navigation_move;
                sentry_system.chassis_set.Vz_state = no_spine;
            }
            else if (mode->Judge_condition.If_chassis_weak)
            {
                sentry_system.chassis_mode = navigation_move;
                sentry_system.chassis_set.Vz_state = no_spine;
            }
            else if (mode->Judge_condition.If_moving_v
                  && !mode->Judge_condition.IF_3s_NotHurted)
            {
                sentry_system.chassis_mode = navigation_move;
                sentry_system.chassis_set.Vz_state = mid_spine;
            }
            else if (!mode->Judge_condition.IF_5s_NotHurted)
            {
                sentry_system.chassis_mode = navigation_move;
                sentry_system.chassis_set.Vz_state = mid_spine;
            }
            else
            {
                sentry_system.chassis_mode = navigation_move;
                sentry_system.chassis_set.Vz_state = no_spine;
            }
        }
    }
    else
    {
        sentry_system.chassis_mode = no_move;
        sentry_system.chassis_set.Vz_state = no_spine;
    }

    chassis_speed_set(&sentry_system);
}

/* ================================================================
 *                连续状态判断处理
 * ================================================================ */

/**
  * @brief  连续状态判断处理(每周期调用, 按顺序更新所有判决条件)
  */
void Judge_Continuous_Handle(Judge_condition_t *mode)
{
    if (game_state.game_progress != 4)
        return;

    If_Point_arrived();
    judge_if_location_over(mode);
    judge_if_nothurt(mode);
    judge_if_not_found(mode);
    judge_if_HP_less_100(mode);
    judge_if_fire_lock(mode);
    judge_if_HP_recover(mode);
    judge_if_stop_navi();
    judge_if_chassis_weak();
    judge_if_need_allow_17(mode);
    judge_if_fortress_free(mode);
    judge_if_enemy_outpost_lock(mode);
    judge_if_need_to_protect(mode);
    judge_if_moving_v(mode);
    judge_if_on_toss(mode);
    judge_if_allowance_less_50(mode);
    judge_if_allowance_less_100(mode);
    judge_if_HP_less_50(mode);
    judge_if_fortress_allow_less_50(mode);
    judge_if_close_to_enemy_out(mode);

    if (decision.robot_data.robot_color == red)
    {
        judge_base_armor_spred(mode, game_robot_HP.red_base_HP);
        judge_if_outpost_destroyed(mode, game_robot_HP.red_outpost_HP);
        judge_if_chip_base(game_robot_HP.red_base_HP, mode);
        judge_if_enemy_outpost_destroyed(mode, game_robot_HP.blue_outpost_HP);
        judge_if_enemy_small_energy(mode, game_robot_HP.blue_outpost_HP);
    }
    else if (decision.robot_data.robot_color == blue)
    {
        judge_base_armor_spred(mode, game_robot_HP.blue_base_HP);
        judge_if_outpost_destroyed(mode, game_robot_HP.blue_outpost_HP);
        judge_if_chip_base(game_robot_HP.blue_base_HP, mode);
        judge_if_enemy_outpost_destroyed(mode, game_robot_HP.red_outpost_HP);
        judge_if_enemy_small_energy(mode, game_robot_HP.red_outpost_HP);
    }
}

/* ================================================================
 *                场地/地形判断
 * ================================================================ */

void judge_if_fortress_allow_less_50(Judge_condition_t *mode)
{
    mode->IF_fortress_allow_less_50 =
        (projectile_allowance.projectile_allowance_fortress < 50) ? 1 : 0;
}

void judge_if_on_toss(Judge_condition_t *mode)
{
    float cur_x = navigation_rx.current_x + Red_Navi_position[0][0];
    float cur_y = navigation_rx.current_y + Red_Navi_position[0][1];

    if ((cur_x > WE_TOSS_START_X && cur_x < WE_TOSS_END_X
      && cur_y > WE_TOSS_START_Y && cur_y < WE_TOSS_END_Y)
     || (cur_x > ENEMY_TOSS_START_X && cur_x < ENEMY_TOSS_END_X
      && cur_y > ENEMY_TOSS_START_Y && cur_y < ENEMY_TOSS_END_Y))
        mode->If_on_toss = 1;
    else
        mode->If_on_toss = 0;
}

void judge_if_moving_v(Judge_condition_t *mode)
{
    float cur_x = navigation_rx.current_x + Red_Navi_position[0][0];
    float cur_y = navigation_rx.current_y + Red_Navi_position[0][1];

    if ((cur_x > WE_U_START_X && cur_x < WE_U_END_X
      && cur_y > WE_U_START_Y && cur_y < WE_U_END_Y)
     || (cur_x > ENEMY_U_START_X && cur_x < ENEMY_U_END_X
      && cur_y > ENEMY_U_START_Y && cur_y < ENEMY_U_END_Y))
        mode->If_moving_v = 1;
    else
        mode->If_moving_v = 0;
}

void judge_if_enemy_outpost_destroyed(Judge_condition_t *mode, int16_t enemy_outpost_HP)
{
    mode->IF_enemy_outpost_destroyed = (enemy_outpost_HP == 0) ? 1 : 0;
}

/**
  * @brief  判断堡垒增益区是否空闲
  * @note   通过3号和4号步兵位置判断是否有人在堡垒
  */
void judge_if_fortress_free(Judge_condition_t *mode)
{
    if (decision.robot_data.robot_color == red)
    {
        if ((fabs(ground_robot_position.standard_3_x - RED_FORTRESS_X) < 0.5f
          && fabs(ground_robot_position.standard_3_y - RED_FORTRESS_Y) < 0.5f)
         || (fabs(ground_robot_position.standard_4_x - RED_FORTRESS_X) < 0.5f
          && fabs(ground_robot_position.standard_4_y - RED_FORTRESS_Y) < 0.5f))
            mode->If_fortress_free = 0;
        else
            mode->If_fortress_free = 1;
    }
    else if (decision.robot_data.robot_color == blue)
    {
        if ((fabs(ground_robot_position.standard_3_x - BLUE_FORTRESS_X) < 0.5f
          && fabs(ground_robot_position.standard_3_y - BLUE_FORTRESS_Y) < 0.5f)
         || (fabs(ground_robot_position.standard_4_x - BLUE_FORTRESS_X) < 0.5f
          && fabs(ground_robot_position.standard_4_y - BLUE_FORTRESS_Y) < 0.5f))
            mode->If_fortress_free = 0;
        else
            mode->If_fortress_free = 1;
    }
}

void judge_if_enemy_outpost_lock(Judge_condition_t *mode)
{
    mode->If_enemy_outpost_lock =
        (mode->IF_base_armor_spred == 1 || game_state.stage_remain_time < 240) ? 1 : 0;
}

/**
  * @brief  判断是否需要去保护基地
  * @note   红方: 英雄x在0~1100(己方半场); 蓝方: 英雄x在1700~2800
  *         当前强制设为0(待启用)
  */
void judge_if_need_to_protect(Judge_condition_t *mode)
{
    u8_to_u16 hero_x, hero_y, infantry_3_x, infantry_3_y, infantry_4_x, infantry_4_y;

    hero_x.d[0]      = robot_interaction_data.data[0];
    hero_x.d[1]      = robot_interaction_data.data[1];
    hero_y.d[0]      = robot_interaction_data.data[2];
    hero_y.d[1]      = robot_interaction_data.data[3];
    infantry_3_x.d[0] = robot_interaction_data.data[4];
    infantry_3_x.d[1] = robot_interaction_data.data[5];
    infantry_3_y.d[0] = robot_interaction_data.data[6];
    infantry_3_y.d[1] = robot_interaction_data.data[7];
    infantry_4_x.d[0] = robot_interaction_data.data[8];
    infantry_4_x.d[1] = robot_interaction_data.data[9];
    infantry_4_y.d[0] = robot_interaction_data.data[10];
    infantry_4_y.d[1] = robot_interaction_data.data[11];

    if (decision.robot_data.robot_color == red)
        mode->IF_need_to_protect = (hero_x.data > 0 && hero_x.data < 1100) ? 1 : 0;
    else if (decision.robot_data.robot_color == blue)
        mode->IF_need_to_protect = (hero_x.data > 1700 && hero_x.data < 2800) ? 1 : 0;

    mode->IF_need_to_protect = 0;   /* 当前强制关闭 */
}

void judge_if_chassis_weak(void)
{
    decision.Judge_condition.If_chassis_weak =
        (game_state.stage_remain_time < 90) ? 1 : 0;
}

void judge_if_stop_navi(void)
{
    decision.Judge_condition.If_stop_navi =
        (receive_gimbal_data.vision_state != vision_lost) ? 1 : 0;
}

/* ================================================================
 *                    射击决策
 * ================================================================ */

/* --- 射击条件判断前向声明 --- */
bool judge_if_shoot_hero(int16_t robot_hp);
bool judge_if_shoot_Engineer(int16_t robot_hp);
bool judge_if_shoot_infantr3(int16_t robot_hp);
bool judge_if_shoot_infantr4(int16_t robot_hp);
bool judge_if_shoot_sentry(int16_t robot_hp, int16_t outpost_HP);

void sentry_shoot_decision(decision_t *mode)
{
    judge_if_shoot(mode);
    judge_shoot_top_senior_priority(mode);
}

/**
  * @brief  确定自瞄最高优先级目标
  * @note   红方瞄准蓝方, 蓝方瞄准红方; 优先残血(5~50)和英雄
  */
void judge_shoot_top_senior_priority(decision_t *mode)
{
    uint16_t vision_robot_HP = 0;

    if (decision.robot_data.robot_color == red)
    {
        if (receive_gimbal_data.vision_armor_id == ARMOR_HERO)
            vision_robot_HP = game_robot_HP.blue_1_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_ENGINEER)
            vision_robot_HP = game_robot_HP.blue_2_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_INFANTRY3)
            vision_robot_HP = game_robot_HP.blue_3_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_INFANTRY4)
            vision_robot_HP = game_robot_HP.blue_4_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_SENTRY)
            vision_robot_HP = game_robot_HP.blue_7_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_OUTPOST)
            vision_robot_HP = game_robot_HP.blue_outpost_HP;
    }
    else if (decision.robot_data.robot_color == blue)
    {
        if (receive_gimbal_data.vision_armor_id == ARMOR_HERO)
            vision_robot_HP = game_robot_HP.red_1_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_ENGINEER)
            vision_robot_HP = game_robot_HP.red_2_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_INFANTRY3)
            vision_robot_HP = game_robot_HP.red_3_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_INFANTRY4)
            vision_robot_HP = game_robot_HP.red_4_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_SENTRY)
            vision_robot_HP = game_robot_HP.red_7_robot_HP;
        else if (receive_gimbal_data.vision_armor_id == ARMOR_OUTPOST)
            vision_robot_HP = game_robot_HP.red_outpost_HP;
    }

    if ((vision_robot_HP > 5 && vision_robot_HP <= 50)
     || receive_gimbal_data.vision_armor_id == ARMOR_HERO)
        mode->top_senior_priority = receive_gimbal_data.vision_armor_id;
    else if (decision.point == CENTRL_HIGH_POINT)
        mode->top_senior_priority = ARMOR_ENGINEER;
    else
        mode->top_senior_priority = ARMOR_HERO;
}

/**
  * @brief  射击条件判断(填充Vision_ByteBits位域)
  */
void judge_if_shoot(decision_t *mode)
{
    if (game_state.game_progress != 4)
    {
        mode->Vision_ByteBits.shoot_Hero      = 1;
        mode->Vision_ByteBits.shoot_Engineer  = 1;
        mode->Vision_ByteBits.shoot_infantr3  = 1;
        mode->Vision_ByteBits.shoot_infantr4  = 1;
        mode->Vision_ByteBits.shoot_Sentry    = 1;
        mode->Vision_ByteBits.shoot_outpost   = 1;
        mode->Vision_ByteBits.shoot_base      = 1;
    }
    else if (decision.robot_data.robot_color == red)
    {
        mode->Vision_ByteBits.shoot_Hero     = judge_if_shoot_hero(game_robot_HP.blue_1_robot_HP);
        mode->Vision_ByteBits.shoot_Engineer = judge_if_shoot_Engineer(game_robot_HP.blue_2_robot_HP);
        mode->Vision_ByteBits.shoot_infantr3 = judge_if_shoot_infantr3(game_robot_HP.blue_3_robot_HP);
        mode->Vision_ByteBits.shoot_infantr4 = judge_if_shoot_infantr4(game_robot_HP.blue_4_robot_HP);
        mode->Vision_ByteBits.shoot_Sentry   = judge_if_shoot_sentry(game_robot_HP.blue_7_robot_HP,
                                                                     game_robot_HP.blue_outpost_HP);
        mode->Vision_ByteBits.shoot_outpost  = judge_if_shoot_outpost(game_robot_HP.blue_outpost_HP);
        mode->Vision_ByteBits.shoot_base     = judge_if_shoot_base(game_robot_HP.blue_outpost_HP);
    }
    else
    {
        mode->Vision_ByteBits.shoot_Hero     = judge_if_shoot_hero(game_robot_HP.red_1_robot_HP);
        mode->Vision_ByteBits.shoot_Engineer = judge_if_shoot_Engineer(game_robot_HP.red_2_robot_HP);
        mode->Vision_ByteBits.shoot_infantr3 = judge_if_shoot_infantr3(game_robot_HP.red_3_robot_HP);
        mode->Vision_ByteBits.shoot_infantr4 = judge_if_shoot_infantr4(game_robot_HP.red_4_robot_HP);
        mode->Vision_ByteBits.shoot_Sentry   = judge_if_shoot_sentry(game_robot_HP.red_7_robot_HP,
                                                                     game_robot_HP.red_outpost_HP);
        mode->Vision_ByteBits.shoot_outpost  = judge_if_shoot_outpost(game_robot_HP.red_outpost_HP);
        mode->Vision_ByteBits.shoot_base     = judge_if_shoot_base(game_robot_HP.red_outpost_HP);
    }
}

/* ================================================================
 *              各兵种射击条件判断(static函数)
 * ================================================================ */

/**
  * @brief  击打英雄: 残血(<=60)冷却9s, 满血(>=200)冷却2s
  */
bool judge_if_shoot_hero(int16_t robot_hp)
{
    bool res;
    static uint16_t wait_tim3  = 0;
    static uint16_t wait_tim10 = 0;
    static uint16_t last_hp;

    if (robot_hp <= 60 && robot_hp > 0 && last_hp == 0)
        wait_tim10 = game_state.stage_remain_time;
    if (robot_hp >= 200 && last_hp == 0)
        wait_tim3 = game_state.stage_remain_time;

    if ((wait_tim10 - game_state.stage_remain_time < 9
      && wait_tim10 - game_state.stage_remain_time >= 0)
     || (wait_tim3 - game_state.stage_remain_time < 2
      && wait_tim3 - game_state.stage_remain_time >= 0))
        res = 0;
    else
        res = 1;

    last_hp = robot_hp;
    return res;
}

/**
  * @brief  击打工程: 残血(<=60)冷却9s, 满血(>=150)冷却3s
  */
bool judge_if_shoot_Engineer(int16_t robot_hp)
{
    bool res;
    static uint16_t wait_tim3  = 0;
    static uint16_t wait_tim10 = 0;
    static uint16_t last_hp;

    if (robot_hp <= 60 && robot_hp > 0 && last_hp == 0)
        wait_tim10 = game_state.stage_remain_time;
    if (robot_hp >= 150 && last_hp == 0)
        wait_tim3 = game_state.stage_remain_time;

    if ((wait_tim10 - game_state.stage_remain_time < 9
      && wait_tim10 - game_state.stage_remain_time >= 0)
     || (wait_tim3 - game_state.stage_remain_time < 3
      && wait_tim3 - game_state.stage_remain_time >= 0))
        res = 0;
    else
        res = 1;

    last_hp = robot_hp;
    return res;
}

/**
  * @brief  击打3号步兵: 残血(<=40)冷却9s, 满血(>=150)冷却3s
  */
bool judge_if_shoot_infantr3(int16_t robot_hp)
{
    bool res;
    static uint16_t wait_tim3  = 0;
    static uint16_t wait_tim10 = 0;
    static uint16_t last_hp;

    if (robot_hp <= 40 && robot_hp > 0 && last_hp == 0)
        wait_tim10 = game_state.stage_remain_time;
    if (robot_hp >= 150 && last_hp == 0)
        wait_tim3 = game_state.stage_remain_time;

    if ((wait_tim10 - game_state.stage_remain_time < 9
      && wait_tim10 - game_state.stage_remain_time >= 0)
     || (wait_tim3 - game_state.stage_remain_time < 3
      && wait_tim3 - game_state.stage_remain_time >= 0))
        res = 0;
    else
        res = 1;

    last_hp = robot_hp;
    return res;
}

/**
  * @brief  击打4号步兵: 残血(<=40)冷却9s, 满血(>=150)冷却3s
  */
bool judge_if_shoot_infantr4(int16_t robot_hp)
{
    bool res;
    static uint16_t wait_tim3  = 0;
    static uint16_t wait_tim10 = 0;
    static uint16_t last_hp;

    if (robot_hp <= 40 && robot_hp > 0 && last_hp == 0)
        wait_tim10 = game_state.stage_remain_time;
    if (robot_hp >= 150 && last_hp == 0)
        wait_tim3 = game_state.stage_remain_time;

    if ((wait_tim10 - game_state.stage_remain_time < 9
      && wait_tim10 - game_state.stage_remain_time >= 0)
     || (wait_tim3 - game_state.stage_remain_time < 3
      && wait_tim3 - game_state.stage_remain_time >= 0))
        res = 0;
    else
        res = 1;

    last_hp = robot_hp;
    return res;
}

/**
  * @brief  击打哨兵: 已死亡禁止, 满血(>=150)冷却2s
  */
bool judge_if_shoot_sentry(int16_t robot_hp, int16_t outpost_HP)
{
    bool res;
    static uint16_t wait_tim3  = 0;
    static uint16_t wait_tim10 = 0;
    static uint16_t last_hp;
    static uint8_t  if_die = 0;

    (void)outpost_HP;

    if (robot_hp <= 80 && robot_hp > 0 && last_hp == 0)
    {
        if_die = 1;
        wait_tim10 = game_state.stage_remain_time;
    }
    if (robot_hp > 80)
        if_die = 0;
    if (robot_hp >= 150 && last_hp == 0)
        wait_tim3 = game_state.stage_remain_time;

    if (if_die
     || (wait_tim3 - game_state.stage_remain_time < 2
      && wait_tim3 - game_state.stage_remain_time >= 0))
        res = 0;
    else
        res = 1;

    last_hp = robot_hp;
    return res;
}

/**
  * @brief  击打前哨站: 仅ENEMY_OUTPOST_POINT或CENTRL_HIGH_POINT时允许
  */
bool judge_if_shoot_outpost(uint16_t outpost_hp)
{
    (void)outpost_hp;
    return (decision.point == ENEMY_OUTPOST_POINT
         || decision.point == CENTRL_HIGH_POINT) ? 1 : 0;
}

/**
  * @brief  击打基地: 仅当前哨站已被摧毁
  */
bool judge_if_shoot_base(int16_t outpost_hp)
{
    return (outpost_hp > 0) ? 0 : 1;
}



