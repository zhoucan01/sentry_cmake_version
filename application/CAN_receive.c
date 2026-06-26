/*
 * _______________#########_______________________ 
 * ______________############_____________________ 
 * ______________#############____________________ 
 * _____________##__###########___________________ 
 * ____________###__######_#####__________________ 
 * ____________###_#######___####_________________ 
 * ___________###__##########_####________________ 
 * __________####__###########_####_______________ 
 * ________#####___###########__#####_____________ 
 * _______######___###_########___#####___________ 
 * _______#####___###___########___######_________ 
 * ______######___###__###########___######_______ 
 * _____######___####_##############__######______ 
 * ____#######__#####################_#######_____ 
 * ____#######__##############################____ 
 * ___#######__######_#################_#######___ 
 * ___#######__######_######_#########___######___ 
 * ___#######____##__######___######_____######___ 
 * ___#######________######____#####_____#####____ 
 * ____######________#####_____#####_____####_____ 
 * _____#####________####______#####_____###______ 
 * ______#####______;###________###______#________ 
 * ________##_______####________####______________ 
 */
/*
 *   ��Ի:  
 *        д��¥��д�ּ䣬д�ּ������Ա��? 
 *        ������Աд�������ó��򻻾�Ǯ��  
 *        ����ֻ�����������������������ߣ�  
 *        ���������ո��գ����������긴�ꡣ  
 *        ��Ը�������Լ䣬��Ը�Ϲ��ϰ�ǰ��  
 *        ���۱�������Ȥ���������г���Ա��  
 *        ����Ц��߯��񲣬��Ц�Լ���̫����  
 *        ��������Ư���ã��ĸ���ó���Ա��?
 */

#include "main.h"
#include "CAN_receive.h"
#include "can.h"
#include "CAN_transmit.h"
#include "bsp_transmit.h"
#include "gimbal_big_yaw.h"
motor_data_t chassis_motor[4];
//motor_data_t yaw_motor;
//motor_data_t shoot_motor[2];
//motor_data_t trig_motor;
//motor_data_t steer_motor[4];
SuperCap_t SuperCAP;
DM_motor_measure_t pitch_motor;
DM_motor_measure_t yaw_motor;
receive_gimbal_data_t receive_gimbal_data=
{
  .vision_state=vision_lost,
}
;
void get_motor_measure(motor_measure_t *ptr, uint8_t data[8])                                                     
	{                                                                                    
		ptr->last_ecd = ptr->ecd;                                                    
		ptr->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                             
		ptr->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                       
		ptr->feedback_current = (uint16_t)((data)[4] << 8 | (data)[5]);                
		ptr->temperate = (data)[6];                                                 		
		ptr->ECD_angle = (ptr->ecd *360) / 8192;                                          
		if ((ptr->ecd - ptr->last_ecd) > 4096)                                       
		{                                                                               
			(ptr->round_cnt)--;                                                        
		}                                                                                
		else if (ptr->ecd - ptr->last_ecd < -4096)                                   
		{                                                                                
			(ptr->round_cnt)++;                                                        
		}                                                                                
		ptr->total_ecd = (ptr->round_cnt * 8192) + (ptr->ecd - ptr->offset_ecd); 
		ptr->total_angle = (ptr->round_cnt * 360) + (ptr->ECD_angle);                 
	}                                                                                  

//	int aaa = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	if(hcan->Instance == hcan1.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) == HAL_OK)
		{
			switch(rx_header.StdId)
			{
				
				case RECEIVE_ID_2:
        {
        get_gimbal_data_2(rx_data,&receive_gimbal_data);
         break;
        }
				  
				
				
				
				case 0x002:
				{
          big_yaw_lost_count=0;
					get_dm4310_motor_measure(&yaw_motor ,rx_data);
					break;
				}
				default:
				{
					
          break;
				}
			}	
		}
	}
	
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	if(hcan->Instance == hcan2.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&rx_header,rx_data ) == HAL_OK)
		{
			switch(rx_header.StdId)
			{
        default:
				{
					
          break;
				}
        
				}
		}

	}
}

const SuperCap_t *get_supercup_pointer(void)
{
	return &SuperCAP;
}


/** 
  * @brief        ��ȡ4310�������?
  * @param        {DM_motor_measure_t} *ptr   4310����ṹ��ָ��?
  * @param        {uint8_t} rx_data[]   can���ݵ�ַ
  * @return       {*}
  */
void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t data[])
{
  
  ptr->Err = data[0] >> 4;
	ptr->Position_int=(data[1]<<8)|data[2];
	ptr->Velocity_int =(data[3]<<4)|(data[4]>>4);
	ptr->Torque_int =((data[4]&0xF)<<8)|data[5];
	
	ptr->Position = uint_to_float(ptr->Position_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->Velocity = uint_to_float(ptr->Velocity_int, V_MIN, V_MAX, 12); // (-45.0,45)
	ptr->Torque = uint_to_float(ptr->Torque_int, T_MIN, T_MAX, 12); // (-18.0,18)
	ptr->Angle   = ptr->Position * 57.29577f;
	
	ptr->T_MOS = data[6];
	ptr->T_Rotor = data[7];
	
}




uint8_t get_gimbal_com_count=0;
void get_gimbal_data_2(uint8_t rx_data[8],receive_gimbal_data_t *data)
{

  int16_to_8 yaw_add,ecd_trans;
//  data->vision_state=vision_frount;
data->vision_state=rx_data[0];
  data->lock_state=rx_data[1];
  yaw_add.d[0]=rx_data[2];
  yaw_add.d[1]=rx_data[3];
  data->vision_armor_id=rx_data[4];
  
//  data->armor_dist=2;
  
  data->armor_dist=rx_data[5];
  ecd_trans.d[0]=rx_data[6];
  ecd_trans.d[1]=rx_data[7];
  
  data->current_transform_angle=ecd_trans.data;
  
  data->small_yaw_add=(float)yaw_add.data;
  
  get_gimbal_com_count=0;
  
}