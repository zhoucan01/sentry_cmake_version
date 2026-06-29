/**
  ************************************* Copyright ******************************
  * FileName   : Sentry_cmd.c
  * Version    : v1.0
  * Author     : Wang Zipei
  * Date       : 2024-05-12
  * Description: sentry command / path display UI via referee system
  ******************************************************************************
 */
#include "Sentry_cmd.h"
#include "referee.h"
#include "navigation.h"
#include "cmsis_os.h"
#include "message_center.h"
#include "big_yaw_topics.h"

Sentry_cmd_t Sentry_cmd;
Sentry_cmd_Fill_t Sentry_cmd_Fill;
uint8_t IF_revive = 1;

Map_data_t Map_data;

/* message_center */
static Subscriber_t *decision_sub = NULL;
static Subscriber_t *robot_status_sub = NULL;

/* local copies */
static decision_t    dec_local;
static robot_status_t robot_status_local;

int8_t Map_diff_x, Map_diff_y;
float Map_diff_x_f, Map_diff_y_f;
uint16_t final_x, final_y;

void Path_display()
{
    static uint8_t data_pack[Map_Data_byte];
    uint8_t Map_Seq;

    /* data fill */
    Map_data.path_data.data.intention = 1;

    if (dec_local.robot_data.robot_color == red)
    {
        Map_data.path_data.data.start_position_x = (navigation_rx.current_x + Red_Navi_position[0][0]) * 10;
        Map_data.path_data.data.start_position_y = (navigation_rx.current_y + Red_Navi_position[0][1]) * 10;

        final_x = Red_Navi_position[dec_local.point][0] * 10;
        final_y = Red_Navi_position[dec_local.point][1] * 10;
    }
    else
    {
        Map_data.path_data.data.start_position_x = (28 - (navigation_rx.current_x + Red_Navi_position[0][0])) * 10;
        Map_data.path_data.data.start_position_y = (15 - (navigation_rx.current_y + Red_Navi_position[0][1])) * 10;

        final_x = (28 - Red_Navi_position[dec_local.point][0]) * 10;
        final_y = (15 - Red_Navi_position[dec_local.point][1]) * 10;
    }

    if (dec_local.point == MANUAL_POINT)
    {
        final_x = map_command.target_position_x * 10;
        final_y = map_command.target_position_y * 10;
    }

    Map_diff_x_f = (final_x - Map_data.path_data.data.start_position_x) / 2;
    Map_diff_y_f = (final_y - Map_data.path_data.data.start_position_y) / 2;

    Map_diff_x = (int8_t)Map_diff_x_f;
    Map_diff_y = (int8_t)Map_diff_y_f;

    for (int i = 0; i < 49; i++)
    {
        Map_data.path_data.data.delta_x[i] = 0;
        Map_data.path_data.data.delta_y[i] = 0;
    }
    Map_data.path_data.data.delta_x[47] = Map_diff_x;
    Map_data.path_data.data.delta_y[47] = Map_diff_y;

    Map_data.path_data.data.delta_x[48] = Map_diff_x;
    Map_data.path_data.data.delta_y[48] = Map_diff_y;

    Map_data.path_data.data.sender_id = robot_status_local.robot_id;

    Map_data.path_data.UI_Packhead.SOF = UI_SOF;
    Map_data.path_data.UI_Packhead.Seq = Map_Seq;
    Map_data.path_data.UI_Packhead.Data_Length = Map_Data_byte - 9;

    /* send data */
    memcpy(data_pack, &Map_data.path_data.UI_Packhead, 5);

    append_CRC8_check_sum(data_pack, 5);

    uint16_t CMD_ID = CONST__PATH_SEND_CMD_ID;
    memcpy(data_pack + 5, &CMD_ID, 2);

    memcpy(data_pack + 7, &Map_data.path_data.data, 105);

    append_CRC16_check_sum(data_pack, Map_Data_byte);

    HAL_UART_Transmit_DMA(&huart6, data_pack, Map_Data_byte);

    Map_Seq++;
}

uint16_t ooo;

void referee_transmit()
{
    vTaskDelay(500);

    /* register subscribers */
    decision_sub    = SubRegister(TOPIC_DECISION_DATA, sizeof(decision_t));
    robot_status_sub = SubRegister(TOPIC_ROBOT_STATUS, sizeof(robot_status_t));

    for(;;)
    {
        /* fetch latest subscribed data */
        SubGetMessage(decision_sub, &dec_local);
        SubGetMessage(robot_status_sub, &robot_status_local);

        Path_display();
        vTaskDelay(300);
        ooo++;

        if (sentry_info.sentry_if_free_revive == 1 && dec_local.Cmd_condition.If_revive == 1)
        {
            UI_SendSentry(&Sentry_cmd_send);
            vTaskDelay(100);
        }
    }
}
