#include "small_gimbal.h"
#include "cmsis_os.h"
#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include "bsp_transmit.h"
#include "referee.h"
#include "gimbal_big_yaw.h"
#include "can.h"
#include "can_receive.h"
#include "message_center.h"
#include "big_yaw_topics.h"

/* local data */
com_mode_t small_gimbal_com;
small_gimbal_t small_gimbal;

/* message_center */
static Subscriber_t *sentry_sys_sub = NULL;
static Subscriber_t *robot_status_sub = NULL;
static Subscriber_t *rc_ctrl_sub = NULL;
static Publisher_t  *small_gimbal_pub = NULL;

/* local copies */
static sentry_system_t sentry_sys_local;
static robot_status_t  robot_status_local;
static RC_ctrl_t       rc_ctrl_local;

void small_gimbal_run()
{
    /* register pub/sub */
    sentry_sys_sub   = SubRegister(TOPIC_SENTRY_SYSTEM, sizeof(sentry_system_t));
    robot_status_sub = SubRegister(TOPIC_ROBOT_STATUS, sizeof(robot_status_t));
    rc_ctrl_sub      = SubRegister(TOPIC_RC_CTRL, sizeof(RC_ctrl_t));
    small_gimbal_pub = PubRegister(TOPIC_SMALL_GIMBAL_STATE, sizeof(small_gimbal_t));

    for(;;)
    {
        /* fetch latest subscribed data */
        SubGetMessage(sentry_sys_sub, &sentry_sys_local);
        SubGetMessage(robot_status_sub, &robot_status_local);
        SubGetMessage(rc_ctrl_sub, &rc_ctrl_local);

        get_gimbal_com();
        switch (small_gimbal_com)
        {
            case com_nom:
            {
                system_mode_to_small(&small_gimbal);
                break;
            }
            case com_err:
            {
                small_gimbal.gimbal_pitch_in = 0;
                small_gimbal.gimbal_yaw_in   = 0;
                small_gimbal.pitch_can       = 0;
                break;
            }
        }

        /* publish small gimbal state */
        PubPushMessage(small_gimbal_pub, (void *)&small_gimbal);

        vTaskDelay(2);
    }
}

void system_mode_to_small(small_gimbal_t *mode)
{
    switch (sentry_sys_local.small_gimbal_mode)
    {
        case small_gimbal_pc:
        {
            mode->gimbal_pitch_in = 0;
            mode->gimbal_yaw_in   = 0;
            mode->pitch_can       = 1;
            break;
        }
        case small_gimbal_rc:
        {
            mode->gimbal_pitch_in = rc_ctrl_local.rc.ch3;
            mode->gimbal_yaw_in   = rc_ctrl_local.rc.ch2;
            mode->pitch_can       = 1;
            break;
        }
        case small_gimbal_off:
        {
            mode->gimbal_pitch_in = 0;
            mode->gimbal_yaw_in   = 0;
            mode->pitch_can       = 0;
            break;
        }
    }
}

void get_gimbal_com()
{
    if (robot_status_local.power_management_gimbal_output == 0 ||
        sentry_sys_local.chassis_mode == no_move ||
        get_if_communite_broke() == 1)
    {
        small_gimbal_com = com_err;
    }
    else
        small_gimbal_com = com_nom;
}
