#include  "chassis_control.h"
#include "cmsis_os.h"
#include "referee.h"
#include "gimbal_big_yaw.h"
#include "system.h"
#include "remote_control.h"
#include "navigation.h"
#include "message_center.h"
#include "big_yaw_topics.h"

/* local data (no longer global) */
sentry_chassis_t sentry_chassis;
com_mode_t chassis_com;

/* message_center */
static Subscriber_t *sentry_sys_sub = NULL;
static Subscriber_t *robot_status_sub = NULL;
static Publisher_t  *chassis_state_pub = NULL;

/* local copies of subscribed data */
static sentry_system_t sentry_sys_local;
static robot_status_t  robot_status_local;

void chassis_task_run()
{
    /* register pub/sub */
    sentry_sys_sub    = SubRegister(TOPIC_SENTRY_SYSTEM, sizeof(sentry_system_t));
    robot_status_sub  = SubRegister(TOPIC_ROBOT_STATUS, sizeof(robot_status_t));
    chassis_state_pub = PubRegister(TOPIC_CHASSIS_STATE, sizeof(sentry_chassis_t));

    for(;;)
    {
        /* fetch latest subscribed data */
        SubGetMessage(sentry_sys_sub, &sentry_sys_local);
        SubGetMessage(robot_status_sub, &robot_status_local);

        sentry_chassis.speed_max = 700;
        sentry_chassis.max_wz    = 2000;
        get_chassis_com();
        get_chassis_mode_set(&sentry_chassis);

        /* publish chassis state */
        PubPushMessage(chassis_state_pub, (void *)&sentry_chassis);

        vTaskDelay(1);
    }
}

void get_chassis_com()
{
    if (sentry_sys_local.chassis_mode == no_move ||
        get_if_communite_broke() == 1 ||
        robot_status_local.power_management_chassis_output == 0)
    {
        chassis_com = com_err;
    }
    else
        chassis_com = com_nom;
}

void get_chassis_mode_set(sentry_chassis_t *mode)
{
    switch (chassis_com)
    {
        case com_err:
        {
            mode->chassis_vx = 0;
            mode->chassis_vy = 0;
            mode->chassis_wz = 0;
            break;
        }
        case com_nom:
        {
            chassis_normol_set(mode);
            break;
        }
    }
}

void chassis_normol_set(sentry_chassis_t *mode)
{
    switch (sentry_sys_local.chassis_mode)
    {
        case no_move:
        {
            chassis_normol_move_set(mode);
            break;
        }
        case normol_move:
        {
            chassis_normol_move_set(mode);
            break;
        }
        case navigation_move:
        {
            chassis_spine_move_set(mode);
            break;
        }
    }
}

void chassis_no_move_set(sentry_chassis_t *mode)
{
    mode->chassis_vx = 0;
    mode->chassis_vy = 0;
    mode->chassis_wz = 0;
}

void chassis_normol_move_set(sentry_chassis_t *mode)
{
    mode->chassis_vx = sentry_sys_local.chassis_set.set_vx;
    mode->chassis_vy = sentry_sys_local.chassis_set.set_vy;
    mode->chassis_wz = 0;
}

void chassis_spine_move_set(sentry_chassis_t *mode)
{
    mode->chassis_vx = sentry_sys_local.chassis_set.set_vx * 0.25;
    mode->chassis_vy = sentry_sys_local.chassis_set.set_vy * 0.25;
    mode->chassis_wz = 1200;
}

void chassis_lock_move(sentry_chassis_t *mode)
{
    mode->chassis_vx = sentry_sys_local.chassis_set.set_vx;
    mode->chassis_vy = sentry_sys_local.chassis_set.set_vy;
    mode->chassis_wz = 0;
}

int chassis_speed_limit(int speed_init, int speed_max)
{
    if (speed_init > speed_max)
    {
        speed_init = speed_max;
    }
    else if (speed_init < -speed_max)
    {
        speed_init = -speed_max;
    }

    return speed_init;
}
