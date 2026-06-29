
/**
  ******************************************************************************
  * @file           : decision.h
  * @version        : v2.1
  * @author         : 周灿
  * @date           : 2025-8-1
  * @brief          : 哨兵电控决策模块头文�?
  * @attention      :
  *   坐标系�?�明�?
  *     1. 云台手标点坐�?     -> map_control_fill()
  *     2. 小地图哨兵路径提�?  -> Path_display
  *     3. 导航点推�?         -> Navigation_Tx_Send(&navigation_tx)
  *     4. 导航发来的当前位�?�?里程计坐标（相�?�于程序起�?�点，非零点�?
  *   建图前需根据红蓝方修�? RED_START_NAVIGATION �?
  ******************************************************************************
  */

#ifndef __DECISION_H
#define __DECISION_H
/* ============================== 头文件包�? ============================== */
#include "struct_typedef.h"
#include "stdbool.h"
#include "system.h"
#include "Nautilus_UI.h"

/* ======================== 导航点位索引定义（共14�?�? ======================== */
#define INIT_PACK_POINT                    0   /* 初�?�化放置�? */
#define WE_DEPOT_POINT                     1   /* 己方补给�? */
#define WE_FLYING_POINT                    2   /* 己方飞坡落点 */
#define WE_FORTRESS_POINT                  3   /* 己方堡垒 */
#define WE_PATROL_POINT                    4   /* 己方防守�? */
#define WE_FORTRESS_BEHIND_FRONT_POINT     5   /* 己方堡垒前方 */
#define WE_FORTRESS_BEHIND_RIGHT_POINT     6   /* 己方堡垒右侧 */
#define ENEMY_OUTPOST_POINT                7   /* 敌方前哨�? */
#define ENEMY_OUTPOST_PROTECT_POINT        8   /* 防�?�敌方英雄击打前哨站 */
#define ENEMY_BASE_PROTECT_POINT           9   /* 防�?�敌方英雄吊射基�? */
#define WE_OUTPOST_POINT                  10   /* 己方前哨�? */
#define ENEMY_FORTRESS_POINT              11   /* 敌方堡垒 */
#define CENTRL_HIGH_POINT                 12   /* 高地背敌�? */
#define MANUAL_POINT                      13   /* 云台手手动标�? */
#define DECISION_POSITION_NUM             14   /* 点位总数 */

/* ======================== 计时器阈�? ======================== */
#define TIM_3S    500    /* 3秒�?�数�? */
#define TIM_5S   1500    /* 5秒�?�数�? */
#define TIM_10S  2500    /* 10秒�?�数�? */

/* ======================== 场地关键坐标（红方�?��?�） ======================== */
/* 堡垒坐标 */
#define RED_FORTRESS_X                    10.5f
#define RED_FORTRESS_Y                     7.5f
#define BLUE_FORTRESS_X                   (21.0f - 10.5f)
#define BLUE_FORTRESS_Y                    7.5f

/* 己方U型弯区域 */
#define WE_U_START_X                       3.91f
#define WE_U_END_X                         5.05f
#define WE_U_START_Y                      -1.0f
#define WE_U_END_Y                         3.52f

/* 敌方U型弯区域 */
#define ENEMY_U_START_X                   22.66f
#define ENEMY_U_END_X                     24.5f
#define ENEMY_U_START_Y                   11.60f
#define ENEMY_U_END_Y                     16.0f

/* 己方�?�?荒地 */
#define WE_TOSS_START_X                   11.29f
#define WE_TOSS_END_X                     15.95f
#define WE_TOSS_START_Y                    2.62f
#define WE_TOSS_END_Y                      5.0f

/* 敌方�?�?荒地 */
#define ENEMY_TOSS_START_X                11.9f
#define ENEMY_TOSS_END_X                  16.7f
#define ENEMY_TOSS_START_Y                10.12f
#define ENEMY_TOSS_END_Y                  12.38f

/* ======================== 类型定义 ======================== */

/* 机器人�?�色枚举 */
typedef enum
{
    NO_CONTACT = 0,   /* �?连接 */
    red,              /* 红方 */
    blue,             /* 蓝方 */
} robot_color_t;

/* 决策模式枚举 */
typedef enum
{
    protect      = 0,   /* 保护模式：保持不�?，最简�? */
    extreme,            /* 激进模式：主动进攻敌方前哨�?/堡垒 */
    conservative,       /* 保守模式：分区赛决策+堡垒 */
    air_control,        /* 云台手接管控�? */
    flying,             /* 飞坡模式：打前哨�?+飞坡落点 */
    patrol,             /* 巡逻模式：打前哨站+保护 */
} decision_mode_t;

/* 视�?�打击�?�可位域（压缩存�?�? */
typedef __packed struct
{
    uint8_t shoot_all       : 1;   /* 全部射击 */
    uint8_t shoot_Hero      : 1;   /* 英雄机器�? */
    uint8_t shoot_Engineer  : 1;   /* 工程机器�? */
    uint8_t shoot_infantr3  : 1;   /* 3号�?�兵 */
    uint8_t shoot_infantr4  : 1;   /* 4号�?�兵 */
    uint8_t shoot_Sentry    : 1;   /* 哨兵 */
    uint8_t shoot_outpost   : 1;   /* 前哨�? */
    uint8_t shoot_base      : 1;   /* 基地 */
} Vision_ByteBits_t;

/* 判决条件结构体（1=满足�?0=不满足） */
typedef struct
{
    /* --- 到达与受�? --- */
    bool IF_Arrived;                 /* 哨兵�?否到达目标点 */
    bool IF_3s_NotHurted;            /* 3秒内�?受击 */
    bool IF_5s_NotHurted;            /* 5秒内�?受击 */
    bool IF_10s_NotHurted;           /* 10秒内�?受击 */

    /* --- �?标丢�? --- */
    bool IF_3s_NotFound;             /* 3秒内�?发现敌人 */
    bool IF_5s_NotFound;             /* 5秒内�?发现敌人 */
    bool IF_10s_NotFound;            /* 10秒内�?发现敌人 */

    /* --- 血量相�? --- */
    bool IF_HP_Less_50;              /* 血量低�?50 */
    bool IF_HP_Less_100;             /* 血量低�?100 */
    bool If_hp_less_200;             /* 血量低�?200 */
    bool IF_HP_recover;              /* 回�?�完成（满血�? */

    /* --- 基地/前哨站相�? --- */
    bool IF_base_armor_spred;        /* 己方基地护甲展开 */
    bool IF_outpost_destroyed;       /* 己方前哨站�??击毁 */
    bool IF_enemy_outpost_destroyed; /* 敌方前哨站�??击毁 */
    bool If_enemy_outpost_lock;      /* 敌方前哨站停�? */

    /* --- 发射与弹�? --- */
    bool IF_fire_lock;               /* 发射机构锁住 */
    bool IF_allowance_less_50;       /* 允�?�发弹量<50 */
    bool IF_allowance_less_100;      /* 允�?�发弹量<100 */

    /* --- 地形与运�? --- */
    bool If_on_toss;                 /* �?否在�?�?荒地�? */
    bool If_moving_v;                /* �?否�?�在过U型弯 */

    /* --- 战术条件 --- */
    bool If_need_to_enemy_fortress;  /* �?否需要上敌方堡垒 */
    bool If_stop_navi;               /* �?否停下来击打敌人 */
    bool If_chassis_weak;            /* 底盘虚弱模式（比赛末期） */
    bool If_get_allow_17;            /* �?否需要回补给区补�? */
    bool IF_fortress_allow_less_50;  /* 堡垒增益点弹�?<50 */
    bool IF_energy_Mechanism;        /* �?否给打�?�车让位 */
    bool IF_need_to_protect;         /* �?否需要去保护基地 */
    bool If_fortress_free;           /* 堡垒增益区空�? */
    bool If_chip_base;               /* 基地�?否�??吊射 */
    bool If_enemy_small_energy;      /* 敌方�?否开�?小能量机�? */
    bool If_close_to_enemy_out;      /* �?否靠近敌方前哨站 */
} Judge_condition_t;

/* 机器人基�?信息 */
typedef struct
{
    robot_color_t robot_color;   /* 机器人�?�色 */
} robot_data_t;

/* 哨兵�?主决策指令条�? */
typedef struct
{
    uint8_t  Die_cnt;                  /* 死亡计数 */
    uint8_t  If_revive;                /* �?否�?�条复活 */
    uint8_t  If_Immediately_Revive;    /* �?否立即�?�活 */
    uint16_t Exchange_Projectile_Num;  /* 哨兵兑换发弹�? */
    uint8_t  If_remote_exchange_HP;    /* 远程兑换血�? */
    bool     if_update;                /* 裁判系统�?否响应当前指�? */
} Cmd_condition_t;

/* 决策核心结构�? */
typedef struct
{
    Vision_ByteBits_t  Vision_ByteBits;       /* 视�?�打击�?�可�? */
    Judge_condition_t  Judge_condition;       /* 判决条件�? */
    decision_mode_t    decision_mode;         /* 当前决策模式 */
    decision_mode_t    last_decision_mode;    /* 上一次决策模�? */
    Cmd_condition_t    Cmd_condition;         /* 哨兵指令条件 */
    uint8_t            point;                 /* 当前�?标点位索�? */
    uint8_t            last_point;            /* 上一次点位索�? */
    uint8_t            if_point_new;          /* �?否为新点�? */
    robot_data_t       robot_data;            /* 机器人数�? */
    uint8_t            top_senior_priority;   /* �?瞄最高优先级�?�? */
    bool               If_point_change;       /* 决策点是否切�? */
    bool               keyboard_disable;      /* �?盘一�?失能 */
} decision_t;

/* ======================== 外部变量声明 ======================== */
extern decision_t    decision;
extern Sentry_cmd_t  Sentry_cmd_send;
extern uint8_t       if_navi_receive;
extern bool          IF_navi_target_renew;
extern float         Red_Navi_position[DECISION_POSITION_NUM][2];
extern float         Blue_Navi_position[DECISION_POSITION_NUM][2];

/* --- 射击相关外部变量 --- */
extern uint16_t  last_dart_time;
extern uint16_t  lock_dart_count;
extern uint8_t   if_random_dart;
extern uint16_t  Last_base_hurt_time;

/* ======================== 函数声明 ======================== */

/* --- FreeRTOS 主任�? --- */
void Auto_run(void const * argument);

/* --- 决策初�?�化与主�? --- */
void Decision_Init(decision_t *mode);
void Decison_State_Ctl(decision_t *mode);
void decision_point_chose(decision_t *mode);
void decision_point_fill(void);
void Judge_Continuous_Handle(Judge_condition_t *mode);

/* --- 模式选择 --- */
void AGV_auto_mode(decision_t *mode);

/* --- 各决策模式函�? --- */
void sentry_extreme_decision(decision_t *mode);
void sentry_conservative_decision(decision_t *mode);
void sentry_patrol_decision(decision_t *mode);
void sentry_flying_decision(decision_t *mode);
void sentry_protect_decision(decision_t *mode);
void sentry_air_control_decision(decision_t *mode);
void sentry_two_point_decision(decision_t *mode);
void sentry_test_decision(decision_t *mode);

/* --- 云台手控�? --- */
void air_control_ctl(decision_t *mode);
void map_control_fill(decision_t *mode);
void judge_if_keyboard_disable(decision_t *mode);

/* --- 射击决策 --- */
void sentry_shoot_decision(decision_t *mode);
void judge_if_shoot(decision_t *mode);
void judge_shoot_top_senior_priority(decision_t *mode);
bool judge_if_shoot_outpost(uint16_t outpost_hp);
bool judge_if_shoot_base(int16_t outpost_hp);

/* --- 底盘控制 --- */
void AGV_auto_chassis(decision_t *mode);

/* --- 裁判系统数据 --- */
void get_referr_data(void);
void Sentry_cmd_decision(decision_t *mode);

/* --- 判决条件判断函数 --- */
void judge_if_location_over(Judge_condition_t *mode);
void If_Point_arrived(void);
void IF_decision_point_change(Judge_condition_t *mode);
bool judge_if_nothurt(Judge_condition_t *mode);
void judge_if_not_found(Judge_condition_t *mode);
void judge_if_HP_less_50(Judge_condition_t *mode);
void judge_if_HP_less_100(Judge_condition_t *mode);
void judge_if_HP_less_200(Judge_condition_t *mode);
void judge_if_HP_recover(Judge_condition_t *mode);
void judge_if_fire_lock(Judge_condition_t *mode);
void judge_if_stop_navi(void);
void judge_if_chassis_weak(void);
void judge_if_need_allow_17(Judge_condition_t *mode);
void judge_if_allowance_less_50(Judge_condition_t *mode);
void judge_if_allowance_less_100(Judge_condition_t *mode);
void judge_if_fortress_allow_less_50(Judge_condition_t *mode);
void judge_if_on_toss(Judge_condition_t *mode);
void judge_if_moving_v(Judge_condition_t *mode);
void judge_if_close_to_enemy_out(Judge_condition_t *mode);
void judge_if_need_to_enemy_fortress(Judge_condition_t *mode);
void judge_if_need_to_protect(Judge_condition_t *mode);
void judge_base_armor_spred(Judge_condition_t *mode, int16_t base_HP);
void judge_if_outpost_destroyed(Judge_condition_t *mode, int16_t outpost_HP);
void judge_if_enemy_outpost_destroyed(Judge_condition_t *mode, int16_t enemy_outpost_HP);
void judge_if_chip_base(uint16_t base_hp, Judge_condition_t *mode);
void judge_if_enemy_outpost_lock(Judge_condition_t *mode);
void judge_if_fortress_free(Judge_condition_t *mode);
bool judge_if_enemy_small_energy(Judge_condition_t *mode, int16_t outpost_HP);

#endif /* __DECISION_H */

