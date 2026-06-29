#include "system.h"
#include "motor.h"
#include "CAN_receive.h"
//#include "task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "CAN_transmit.h"
#include "gimbal_big_yaw.h"
#include "remote_control.h"
#include "message_center.h"
#include "big_yaw_topics.h"
#define gimbal_run
int ddddd;
int yaw_flag=1;

/* message_center subscribers */
static Subscriber_t *gimbal_state_sub = NULL;
static Subscriber_t *yaw_motor_sub = NULL;

/* local data copies */
static gimbal_t           gimbal_local;
static DM_motor_measure_t yaw_motor_local;

void motor_run()
{
    /* subscribe to gimbal_state and yaw_motor_data */
    gimbal_state_sub = SubRegister(TOPIC_GIMBAL_STATE, sizeof(gimbal_t));
    yaw_motor_sub    = SubRegister(TOPIC_YAW_MOTOR_DATA, sizeof(DM_motor_measure_t));

    for(;;)
    {
        /* fetch latest data into local copies */
        SubGetMessage(gimbal_state_sub, &gimbal_local);
        SubGetMessage(yaw_motor_sub, &yaw_motor_local);

        DM_start_run();
        dm_current_run();

        vTaskDelay(1);
    }
}


void DM_start_run()
{
 if(gimbal_local.if_big_yaw_can==1)
 {

   if(yaw_motor_local.Err==0)
   {
   pitch_Enable();

    yaw_flag=1;
   }
 }
 else if(gimbal_local.if_big_yaw_can==0)
 {

   if(yaw_motor_local.Err==1)
   {

   pitch_Unable();
   yaw_flag=0;
   }
 }

}


void pitch_Enable(void)
{

    for(int i=0;i<10;i++)
    {

   dm_motor_START(dm_yaw);
     vTaskDelay(1);
    }
}

void    pitch_Unable(void)
{

    dm_motor_STOP(dm_yaw);
    vTaskDelay(1);

}

void dm_current_run(void)
{
#ifdef gimbal_run
  dm_motor_Ctrl(dm_yaw,0,0,0,0,yaw_motor_local.Torque_SET);
  vTaskDelay(1);
  #else
  dm_motor_Ctrl(dm_yaw,0,0,0,0,0);

  #endif
}
