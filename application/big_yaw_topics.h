/**
 * @file big_yaw_topics.h
 * @brief 集中定义所有 message_center 话题名称
 *
 * 每个话题对应一个跨任务共享的数据流。
 * 改造前这些数据通过 extern 全局变量直接访问；
 * 改造后通过 PubRegister/SubRegister + PubPushMessage/SubGetMessage 传递。
 *
 * 话题命名规则：用下划线分隔，描述数据流向
 *   - "_cmd" 后缀 → 命令（从决策/系统到执行模块）
 *   - "_feed" 后缀 → 反馈（从执行模块到决策/系统）
 *   - "_state" 后缀 → 状态数据（模块对外输出）
 *   - 无后缀 → 通用数据
 */

#ifndef BIG_YAW_TOPICS_H
#define BIG_YAW_TOPICS_H

/* ==================================================================
 *  Topic 清单
 *
 *  #   | 话题名              | 数据类型                  | 发布者       | 订阅者
 *  ----|--------------------|--------------------------|-------------|--------------------
 *   1  | sentry_system      | sentry_system_t          | system_task | chassis, big_yaw,
 *      |                    |                          |             | small_gimbal, transmit
 *   2  | ins_data           | INS_t                    | INSTask     | system, big_yaw,
 *      |                    |                          |             | transmit, decision
 *   3  | rc_ctrl            | RC_ctrl_t                | system_task | system(自用), big_yaw,
 *      |                    |                          | (桥接ISR)   | small_gimbal, decision,
 *      |                    |                          |             | chassis
 *   4  | gimbal_state       | gimbal_t                 | big_yaw     | motor, transmit
 *   5  | yaw_motor_data     | DM_motor_measure_t       | big_yaw     | motor, transmit
 *      |                    |                          | (Torque_SET)|
 *   6  | decision_data      | decision_t               | Auto_run    | system, big_yaw,
 *      |                    |                          |             | transmit, chassis, UI
 *   7  | vision_data        | receive_gimbal_data_t    | big_yaw     | decision
 *      |                    |                          | (桥接CAN)   |
 *   8  | chassis_state      | sentry_chassis_t         | chassis     | transmit
 *   9  | small_gimbal_state | small_gimbal_t           | small_gimbal| transmit
 *  10  | game_state         | game_state_t             | REFEREE     | system, decision,
 *      |                    |                          |             | big_yaw, chassis,
 *      |                    |                          |             | transmit
 *  11  | robot_status       | robot_status_t           | REFEREE     | system, chassis,
 *      |                    |                          |             | small_gimbal, big_yaw,
 *      |                    |                          |             | transmit, decision
 *  12  | power_heat         | power_heat_data_t        | REFEREE     | transmit, decision
 *  13  | nav_rx             | navigation_rx_t          | system_task | system(自用),
 *      |                    |                          | (桥接USB)   | chassis, decision,
 *      |                    |                          |             | big_yaw
 *  14  | nav_tx             | navigation_tx_t          | decision    | navigation 模块
 * ================================================================== */

/* ---------- 话题名称宏定义（避免拼写错误） ---------- */
#define TOPIC_SENTRY_SYSTEM      "sentry_system"
#define TOPIC_INS_DATA           "ins_data"
#define TOPIC_RC_CTRL            "rc_ctrl"
#define TOPIC_GIMBAL_STATE       "gimbal_state"
#define TOPIC_YAW_MOTOR_DATA     "yaw_motor_data"
#define TOPIC_DECISION_DATA      "decision_data"
#define TOPIC_VISION_DATA        "vision_data"
#define TOPIC_CHASSIS_STATE      "chassis_state"
#define TOPIC_SMALL_GIMBAL_STATE "small_gimbal_state"
#define TOPIC_GAME_STATE         "game_state"
#define TOPIC_ROBOT_STATUS       "robot_status"
#define TOPIC_POWER_HEAT         "power_heat"
#define TOPIC_NAV_RX             "nav_rx"
#define TOPIC_NAV_TX             "nav_tx"

#endif /* BIG_YAW_TOPICS_H */
