#include "bsp_transmit.h"
#include "cmsis_os.h"

#include "system.h"

#include "remote_control.h"
#include <stdlib.h>
#include "referee.h"
#include "stdbool.h"
#include "Nautilus_Vision.h"
#include "CAN_receive.h"
#include "decision.h"
#include "navigation.h"
#include "ins_task.h"
#include "bsp_pid.h"
#include "message_center.h"
#include "big_yaw_topics.h"

#define Navigation

sentry_system_t sentry_system;
robot_data_t robot_data;
pid_struct_t pid_chaais_Sx;
pid_struct_t pid_chaais_Sy;

/* message_center handles */
static Publisher_t  *sentry_pub = NULL;
static Publisher_t  *rc_pub = NULL;
static Subscriber_t *ins_sub = NULL;
static Subscriber_t *decision_sub = NULL;
static Subscriber_t *game_state_sub = NULL;
static Subscriber_t *robot_status_sub = NULL;

/* local copies of subscribed data */
static INS_t          ins_local;
static decision_t     dec_local;
static game_state_t   game_state_local;
static robot_status_t robot_status_local;

/* rc_ctrl bridge: system_run reads raw global (ISR->buffer) and publishes */
/* sentry_system local copy */
static sentry_system_t sys_local;

void system_run(void const *argument)
{
    (void)argument;
    pid_init(&pid_chaais_Sx, 0.1, 0, 0, 0, 1);
    pid_init(&pid_chaais_Sy, 0.1, 0, 0, 0, 1);

    /* register pub/sub */
    sentry_pub      = PubRegister(TOPIC_SENTRY_SYSTEM, sizeof(sentry_system_t));
    rc_pub          = PubRegister(TOPIC_RC_CTRL, sizeof(RC_ctrl_t));
    ins_sub         = SubRegister(TOPIC_INS_DATA, sizeof(INS_t));
    decision_sub    = SubRegister(TOPIC_DECISION_DATA, sizeof(decision_t));
    game_state_sub  = SubRegister(TOPIC_GAME_STATE, sizeof(game_state_t));
    robot_status_sub = SubRegister(TOPIC_ROBOT_STATUS, sizeof(robot_status_t));

    /* init phase: write power management directly (before anyone reads) */
    robot_status.power_management_chassis_output = 1;
    robot_status.power_management_gimbal_output = 1;
    robot_status.power_management_shooter_output = 1;

    sys_local.control_mode = rc_mode;

    for(;;)
    {
        /* fetch latest subscribed data into local copies */
        SubGetMessage(ins_sub, &ins_local);
        SubGetMessage(decision_sub, &dec_local);
        SubGetMessage(game_state_sub, &game_state_local);
        SubGetMessage(robot_status_sub, &robot_status_local);

        /* bridge: publish rc_ctrl from raw ISR buffer for other tasks */
        PubPushMessage(rc_pub, (void *)&rc_ctrl);

        #ifdef Game_prograss

        #else
        if (get_if_communite_broke() == 1 ||
            dec_local.keyboard_disable == 1 ||
            judg_if_imu_error())
        {
            remote_offline_set(&sys_local);
        }
        else
        {
            choose_control_mode(&sys_local);

            switch (sys_local.control_mode)
            {
                case rc_mode:
                {
                    AGV_mode_chose(&sys_local);
                    AGV_big_yaw(&sys_local);
                    shoot_mode_chose(&sys_local);
                    small_gimbal_mode_chose(&sys_local);
                    break;
                }
                case auto_mode:
                {
                    AGV_auto_chassis(&dec_local);
                    AUTO_big_yaw(&sys_local);
                    Auto_shoot_mode_chose(&sys_local);
                    Auto_small_gimbal_mode(&sys_local);
                    break;
                }
            }
        }
        #endif

        /* publish computed sentry_system */
        PubPushMessage(sentry_pub, (void *)&sys_local);

        vTaskDelay(1);
    }
}

uint8_t judg_if_imu_error()
{
    uint8_t if_imu_error;
    if (fabs(ins_local.Pitch) > 65 || fabs(ins_local.Roll) > 65)
    {
        if_imu_error = 1;
    }
    else
        if_imu_error = 0;

    return if_imu_error;
}

void Auto_shoot_mode_chose(sentry_system_t *mode)
{
    if (game_state_local.game_progress == 4)
    {
        if (rc_ctrl.rc.s_l != 2)
        {
            mode->shoot_mode = shoot_on;
            mode->trig_mode = trig_on;
        }
        else
        {
            mode->shoot_mode = shoot_no;
            mode->trig_mode = trig_off;
        }
    }
    else
    {
        mode->shoot_mode = shoot_no;
        mode->trig_mode = trig_off;
    }
}

uint8_t last_navi_seq;

void AGV_mode_chose(sentry_system_t *mode)
{
    switch (rc_ctrl.rc.s_l)
    {
        case 2:
        {
            mode->chassis_mode = no_move;
            mode->chassis_set.Vz_state = no_spine;
            break;
        }
        case 3:
        {
            mode->chassis_mode = normol_move;
            mode->chassis_set.Vz_state = no_spine;
            break;
        }
        case 1:
        {
            if (rc_ctrl.rc.s_r == 1)
            {
                if (dec_local.Judge_condition.IF_Arrived == 0)
                {
                    mode->chassis_mode = navigation_move;
                    mode->chassis_set.Vz_state = no_spine;
                }
                else
                {
                    mode->chassis_mode = normol_move;
                    mode->chassis_set.Vz_state = low_spine;
                }
            }
            else
            {
                mode->chassis_set.Vz_state = high_spine;
                mode->chassis_mode = normol_move;
            }
            break;
        }
    }
    chassis_speed_set(mode);
}

uint8_t if_lost_navi;
int16_t navi_weak_state;

void chassis_speed_set(sentry_system_t *mode)
{
    switch (mode->chassis_mode)
    {
        case no_move:
        {
            mode->chassis_set.set_vx = 0;
            mode->chassis_set.set_vy = 0;
            break;
        }
        case normol_move:
        {
            mode->chassis_set.set_vx = -rc_ctrl.rc.ch1 * 4.5 * WHEEL_FACTOR;
            mode->chassis_set.set_vy = -rc_ctrl.rc.ch0 * 4.5 * WHEEL_FACTOR;
            break;
        }
        case navigation_move:
        {
            mode->chassis_set.set_vx = -navigation_rx.navi_vx;
            mode->chassis_set.set_vy =  navigation_rx.navi_vy;
            break;
        }
    }

    navi_tx_count++;

    if (fabs(navigation_rx.navi_vx) < 0.15 && fabs(navigation_rx.navi_vy) < 0.15)
    {
        navi_weak_state++;
    }
    else
        navi_weak_state = 0;

    if (navi_tx_count > 500 || navi_weak_state > 100)
    {
        navigation_rx.if_lost_navi = 1;
    }
    else
    {
        navigation_rx.if_lost_navi = 0;
    }

    if (dec_local.Judge_condition.If_chassis_weak == 1)
    {
        mode->chassis_set.set_vx *= 0.33;
        mode->chassis_set.set_vy *= 0.33;
    }
    else
    {
        mode->chassis_set.set_vx = mode->chassis_set.set_vx;
        mode->chassis_set.set_vy = mode->chassis_set.set_vy;
    }
}

void AGV_big_yaw(sentry_system_t *mode)
{
    if (rc_ctrl.rc.s_l != 2)
    {
        switch (rc_ctrl.rc.s_r)
        {
            case 1:
            {
                if (rc_ctrl.rc.s_l == 1)
                {
                    mode->big_yaw_mode = big_yaw_pc;
                }
                else
                {
                    mode->big_yaw_mode = big_yaw_rc;
                }
                break;
            }
            case 3:
            {
                mode->big_yaw_mode = big_yaw_rc;
                break;
            }
            case 2:
            {
                mode->big_yaw_mode = big_yaw_rc;
                break;
            }
        }
        mode->set_yaw_in = rc_ctrl.rc.ch2 * big_yaw_kp;
    }
    else
    {
        mode->big_yaw_mode = big_yaw_off;
        mode->set_yaw_in = 0;
    }
}

void AUTO_big_yaw(sentry_system_t *mode)
{
    mode->set_yaw_in = rc_ctrl.rc.ch2 * big_yaw_kp;

    switch (rc_ctrl.rc.s_l)
    {
        case 1:
        {
            mode->big_yaw_mode = big_yaw_pc;
            break;
        }
        case 3:
        {
            mode->big_yaw_mode = big_yaw_pc;
            break;
        }
        case 2:
        {
            mode->big_yaw_mode = big_yaw_off;
            break;
        }
    }
}

void shoot_mode_chose(sentry_system_t *mode)
{
    switch (rc_ctrl.rc.s_r)
    {
        case 1:
        {
            if (mode->vision_mode == vision_off)
            {
                mode->shoot_mode = shoot_on;
                mode->trig_mode = trig_on;
            }
            else if (mode->vision_mode == vision_on)
            {
                #ifndef normal_test
                if (game_state_local.game_progress == 4)
                {
                    if (dec_local.Judge_condition.IF_Arrived == 1)
                    {
                        mode->shoot_mode = shoot_on;
                        mode->trig_mode = trig_on;
                    }
                    else
                    {
                        mode->shoot_mode = shoot_on;
                        mode->trig_mode = trig_on;
                    }
                }
                else
                {
                    mode->shoot_mode = shoot_on;
                    mode->trig_mode = trig_on;
                }
                #else
                mode->shoot_mode = shoot_on;
                mode->trig_mode = trig_on;
                #endif
            }
            break;
        }
        case 3:
        {
            mode->shoot_mode = shoot_no;
            mode->trig_mode = trig_off;
            break;
        }
        case 2:
        {
            mode->shoot_mode = shoot_no;
            mode->trig_mode = trig_off;
            break;
        }
    }
}

void remote_offline_set(sentry_system_t *mode)
{
    mode->chassis_set.set_vx = 0;
    mode->chassis_set.set_vy = 0;
    mode->set_yaw_in = 0;
    mode->chassis_mode = no_move;
    mode->set_yaw_in = 0;
    mode->shoot_mode = shoot_no;
}

int last_rc_wheel = 0;

void choose_control_mode(sentry_system_t *mode)
{
    if (rc_ctrl.rc.wheel < -400 && game_state_local.stage_remain_time < 420)
    {
        mode->control_mode = auto_mode;
    }
    else if (rc_ctrl.rc.wheel < 200 && last_rc_wheel > 200)
    {
        mode->control_mode = rc_mode;
    }
    last_rc_wheel = rc_ctrl.rc.wheel;
}

void Auto_small_gimbal_mode(sentry_system_t *mode)
{
    if (robot_status_local.power_management_gimbal_output == 0)
    {
        mode->small_gimbal_mode = small_gimbal_off;
        mode->if_small_pitch_can = 0;
        mode->vision_mode = vision_off;
    }
    else
    {
        if (rc_ctrl.rc.s_l != 2)
        {
            mode->small_gimbal_mode = small_gimbal_pc;
            mode->if_small_pitch_can = 1;
            mode->vision_mode = vision_on;
        }
        else
        {
            mode->small_gimbal_mode = small_gimbal_off;
            mode->if_small_pitch_can = 0;
            mode->vision_mode = vision_off;
        }
    }
}

void small_gimbal_mode_chose(sentry_system_t *mode)
{
    if (rc_ctrl.rc.s_l == 1)
    {
        switch (rc_ctrl.rc.s_r)
        {
            case 1:
            {
                if (game_state_local.game_progress == 4)
                {
                    if (dec_local.Judge_condition.IF_Arrived == 1)
                    {
                        mode->small_gimbal_mode = small_gimbal_pc;
                        mode->if_small_pitch_can = 1;
                        mode->vision_mode = vision_on;
                    }
                    else
                    {
                        mode->small_gimbal_mode = small_gimbal_pc;
                        mode->if_small_pitch_can = 1;
                        mode->vision_mode = vision_on;
                    }
                }
                else
                {
                    mode->small_gimbal_mode = small_gimbal_pc;
                    mode->if_small_pitch_can = 1;
                    mode->vision_mode = vision_on;
                }
                break;
            }
            case 3:
            {
                mode->small_gimbal_mode = small_gimbal_rc;
                mode->if_small_pitch_can = 1;
                mode->vision_mode = vision_off;
                break;
            }
            case 2:
            {
                mode->small_gimbal_mode = small_gimbal_off;
                mode->if_small_pitch_can = 0;
                mode->vision_mode = vision_off;
                break;
            }
        }
    }
    else if (rc_ctrl.rc.s_l == 3)
    {
        switch (rc_ctrl.rc.s_r)
        {
            case 1:
            {
                mode->small_gimbal_mode = small_gimbal_rc;
                mode->if_small_pitch_can = 1;
                break;
            }
            case 3:
            {
                mode->small_gimbal_mode = small_gimbal_rc;
                mode->if_small_pitch_can = 1;
                break;
            }
            case 2:
            {
                mode->small_gimbal_mode = small_gimbal_off;
                mode->if_small_pitch_can = 0;
                break;
            }
        }
        mode->vision_mode = vision_off;
    }
    else
    {
        mode->small_gimbal_mode = small_gimbal_off;
        mode->if_small_pitch_can = 0;
        mode->vision_mode = vision_off;
    }
    if (robot_status_local.power_management_gimbal_output == 0)
    {
        mode->small_gimbal_mode = small_gimbal_off;
        mode->if_small_pitch_can = 0;
        mode->vision_mode = vision_off;
    }
}
