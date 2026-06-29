/**
 ******************************************************************************
 * @file    gimbal_big_yaw.c
 * @brief   big yaw gimbal control with pub/sub refactoring
 ******************************************************************************
 */

#include "gimbal_big_yaw.h"
#include "system.h"
#include "cmsis_os.h"
#include "referee.h"
#include "remote_control.h"
#include "can_receive.h"
#include "ins_task.h"
#include "bsp_pid.h"
#include "navigation.h"
#include "stdbool.h"
#include "message_center.h"
#include "big_yaw_topics.h"

/* ---- PID ---- */
pid_struct_t pid_yaw_angle;
pid_struct_t pid_yaw_speed;
pid_struct_t pid_vision_yaw_angle;
pid_struct_t pid_vision_yaw_speed;
pid_struct_t pid_navi_yaw_speed;

/* ---- state ---- */
com_mode_t gim_com;
com_mode_t last_gim;
gimbal_t gimbal;

int      big_yaw_lost_count;
static uint8_t  last_vision_mode;
static lock_state_t     Last_lock_state;
static Vision_look_state_t Last_look_state;

/* ---- message_center handles ---- */
static Subscriber_t *sentry_sys_sub = NULL;
static Subscriber_t *ins_sub = NULL;
static Subscriber_t *decision_sub = NULL;
static Subscriber_t *robot_status_sub = NULL;
static Subscriber_t *vision_sub = NULL;
static Publisher_t  *gimbal_pub = NULL;
static Publisher_t  *yaw_motor_pub = NULL;
static Publisher_t  *vision_pub = NULL;

/* local copies of subscribed data */
static sentry_system_t       sentry_sys_local;
static INS_t                 ins_local;
static decision_t            dec_local;
static robot_status_t        robot_status_local;
static receive_gimbal_data_t vision_local;

/* ---- internal declarations ---- */
void get_big_gimbal_com(void);
static void gimbal_curise_set(gimbal_t *mode);
void gimbal_vision_set(gimbal_t *mode);
static bool get_gimbal_response_state(gimbal_t *mode);
static float compare_yaw_add(float yaw_in);

/* ======================== Init ======================== */

void gimbal_pid_init(void)
{
    pid_init(&pid_yaw_angle,        1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_yaw_speed,        1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_navi_yaw_speed,   2.5f, 0.0f, 0.0f, 0.0f,  5.0f);
    pid_init(&pid_vision_yaw_angle, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_vision_yaw_speed, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
}

/* ======================== Main Task ======================== */

void big_yaw_run(void const *argument)
{
    (void)argument;
    gimbal.curise_direction = 1;
    gimbal_pid_init();

    /* register pub/sub */
    sentry_sys_sub  = SubRegister(TOPIC_SENTRY_SYSTEM, sizeof(sentry_system_t));
    ins_sub         = SubRegister(TOPIC_INS_DATA, sizeof(INS_t));
    decision_sub    = SubRegister(TOPIC_DECISION_DATA, sizeof(decision_t));
    robot_status_sub = SubRegister(TOPIC_ROBOT_STATUS, sizeof(robot_status_t));
    vision_sub      = SubRegister(TOPIC_VISION_DATA, sizeof(receive_gimbal_data_t));
    gimbal_pub      = PubRegister(TOPIC_GIMBAL_STATE, sizeof(gimbal_t));
    yaw_motor_pub   = PubRegister(TOPIC_YAW_MOTOR_DATA, sizeof(DM_motor_measure_t));
    vision_pub      = PubRegister(TOPIC_VISION_DATA, sizeof(receive_gimbal_data_t));

    for (;;)
    {
        /* fetch subscribed data into local copies */
        SubGetMessage(sentry_sys_sub, &sentry_sys_local);
        SubGetMessage(ins_sub, &ins_local);
        SubGetMessage(decision_sub, &dec_local);
        SubGetMessage(robot_status_sub, &robot_status_local);

        /* bridge: read vision data from raw global (CAN ISR) and publish */
        /* (also use the raw global directly for own computation) */
        PubPushMessage(vision_pub, (void *)&receive_gimbal_data);

        get_big_gimbal_com();

        switch (gim_com)
        {
            case com_err:
            {
                gimbal.yaw_set        = ins_local.YawTotalAngle;
                gimbal.yaw_vision_set  = ins_local.YawTotalAngle;
                gimbal.if_big_yaw_can  = 0;
                break;
            }
            case com_nom:
            {
                gimbal.if_big_yaw_can = 1;
                gimbal_mode_set(&gimbal);
                break;
            }
        }

        /* publish gimbal state and yaw_motor data */
        PubPushMessage(gimbal_pub, (void *)&gimbal);
        PubPushMessage(yaw_motor_pub, (void *)&yaw_motor);

        vTaskDelay(1);
    }
}

/* ======================== Communication Check ======================== */

void get_big_gimbal_com(void)
{
    big_yaw_lost_count++;

    if (sentry_sys_local.chassis_mode == no_move
        || get_if_communite_broke() == 1
        || robot_status_local.power_management_gimbal_output == 0
        || big_yaw_lost_count > 200)
    {
        gim_com = com_err;
    }
    else
    {
        gim_com = com_nom;
    }

    if (robot_status_local.power_management_gimbal_output == 0)
    {
        yaw_motor.Err = 0;
    }

    last_gim = gim_com;
}

/* ======================== Mode Dispatch ======================== */

void gimbal_mode_set(gimbal_t *mode)
{
    switch (sentry_sys_local.big_yaw_mode)
    {
        case big_yaw_off:
        {
            mode->yaw_set        = ins_local.YawTotalAngle;
            mode->yaw_vision_set  = ins_local.YawTotalAngle;
            break;
        }
        case big_yaw_rc:
        {
            gimbal_vision_no_mode();
            break;
        }
        case big_yaw_pc:
        {
            gimbal_vision_on_mode(mode);
            break;
        }
    }
}

/* ======================== RC Mode (no vision) ======================== */

void gimbal_vision_no_mode(void)
{
    gimbal.yaw_set -= sentry_sys_local.set_yaw_in;
    gimbal.yaw_vision_set = ins_local.YawTotalAngle;
    gimbal.if_update = 1;

    gimbal.big_yaw_speed_set = pid_calc(&pid_yaw_angle,
        ins_local.YawTotalAngle, gimbal.yaw_set);
    yaw_motor.Torque_SET = pid_calc(&pid_yaw_speed,
        ins_local.Gyro[2], gimbal.big_yaw_speed_set);
}

/* ======================== Vision Mode ======================== */

void gimbal_vision_on_mode(gimbal_t *mode)
{
    if (dec_local.point == WE_FORTRESS_POINT
        && dec_local.Judge_condition.IF_Arrived == 1)
    {
        mode->gimbal_mode = gimbal_vision_mode;
        gimbal_vision_set(mode);
        mode->if_update = 1;
    }
    else if (receive_gimbal_data.vision_state == vision_lost)
    {
        mode->gimbal_mode = gimbal_curise_mode;
        gimbal_curise_set(mode);
        mode->if_update = 1;
    }
    else
    {
        mode->gimbal_mode = gimbal_vision_mode;
        gimbal_vision_set(mode);
    }
}

static void gimbal_curise_set(gimbal_t *mode)
{
    mode->yaw_vision_set += mode->curise_direction * yaw_cusise_diff;
    mode->if_update       = 1;
    mode->get_current_yaw = ins_local.YawTotalAngle;
    gimbal.yaw_set        = ins_local.YawTotalAngle;

    gimbal.big_yaw_speed_set = pid_calc(&pid_vision_yaw_angle,
        ins_local.YawTotalAngle, gimbal.yaw_vision_set);
    yaw_motor.Torque_SET = pid_calc(&pid_vision_yaw_speed,
        ins_local.Gyro[2], gimbal.big_yaw_speed_set);
}

void gimbal_vision_set(gimbal_t *mode)
{
    gimbal.yaw_set = ins_local.YawTotalAngle;
    get_gimbal_response_state(mode);

    if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == left_lock
        && mode->if_update == 1)
    {
        mode->yaw_add         = receive_gimbal_data.small_yaw_add;
        mode->get_current_yaw = ins_local.YawTotalAngle;
        mode->curise_direction = -1 * direc;
        mode->if_update       = 0;
    }
    else if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == right_lock
        && mode->if_update == 1)
    {
        mode->yaw_add         = receive_gimbal_data.small_yaw_add;
        mode->get_current_yaw = ins_local.YawTotalAngle;
        mode->curise_direction = 1 * direc;
        mode->if_update       = 0;
    }
    else if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == no_block
        && mode->if_update == 1)
    {
        /* keep current target */
    }

    gimbal.big_yaw_speed_set = pid_calc(&pid_vision_yaw_angle,
        ins_local.YawTotalAngle, gimbal.yaw_vision_set);
    yaw_motor.Torque_SET = pid_calc(&pid_vision_yaw_speed,
        ins_local.Gyro[2], gimbal.big_yaw_speed_set);
}

/* ======================== Response Check ======================== */

static bool get_gimbal_response_state(gimbal_t *mode)
{
    if (fabsf(mode->yaw_vision_set - mode->get_current_yaw)
        < compare_yaw_add(mode->yaw_add) && mode->if_update == 0)
    {
        mode->yaw_vision_set += (mode->curise_direction == -1) ? -0.2f : 0.2f;
        mode->if_update = 0;
    }
    else
    {
        mode->if_update = 1;
    }
    return (bool)mode->if_update;
}

static float compare_yaw_add(float yaw_in)
{
    return (fabsf(yaw_in) < 50.0f) ? 50.0f : fabsf(yaw_in);
}

/* ======================== Navi Mode ======================== */

void gimbal_navi_set(gimbal_t *mode)
{
    (void)mode;
    yaw_motor.Torque_SET = pid_calc(&pid_navi_yaw_speed,
        ins_local.Gyro[2] * 1.0f, navigation_rx.navi_wz * 1.9f);
    gimbal.yaw_set        = ins_local.YawTotalAngle;
    gimbal.yaw_vision_set  = ins_local.YawTotalAngle;
}
