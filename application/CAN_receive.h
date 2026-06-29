#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/***鐢垫満id***/
typedef enum
{
	/*底盘3508行进电机*/
	FR_3508_ID = 0X201,
	BR_3508_ID = 0X202,
	BL_3508_ID = 0X203,
	FL_3508_ID = 0X204,
	
	
	
	/*底盘舵向电机*/
	
	
	FR_6020_ID = 0x205,
	BR_6020_ID = 0x206,
	BL_6020_ID = 0x207,
	FL_6020_ID = 0x208,
	//BARREL_MOTOR_ID = 0X204, 


	/************************************浜戝彴yaw杞磇d*******************************/
	
	//PITCH_MOTOR_ID = 0x206,


    /***********************************瓒呯骇鐢靛?筰d*******************************/

} CAN_MotorID_e;


enum
{
	YAW = 0,
	PITCH,
};

enum
{
  RIGHT = 0,
  LEFT = 1,
};

enum
{
	FR = 0,
  BR,
	BL,
	FL,
};

typedef struct 
{
	    /* data */
    int32_t round_cnt;
    int32_t total_ecd;
    int32_t total_angle;
    uint32_t msg_cnt;

    int16_t ecd;              
    int16_t last_ecd;
    int16_t ECD_angle;	
    int16_t speed_rpm;        
    int16_t feedback_current; 
    uint8_t temperate;    

    uint16_t offset_ecd; 
}motor_measure_t;

typedef struct
{
	/* data */
    fp32 sp_tar;
    fp32 corder_tar;
    fp32 ins_tar;
    int16_t set_current;
	fp32 dm_current;
}motor_tar_t;

typedef struct
{
 motor_measure_t motor_measure;
 motor_tar_t motor_tar;
}motor_data_t;

typedef struct
{
	float C_Vol;	//鐢靛?圭數鍘?
	float C_Power;	//鐢靛?硅緭鍑哄姛鐜?
	float Mode_C;	//鐢靛?圭姸鎬佺爜
	float C_Current;	//鐢靛?硅緭鍑虹數娴?
	float PowerData[4];
} SuperCap_t;

/***************************************4310鐢垫満**************************************/
//Master_ID 鎺ユ敹鐢?
//#define YAW_MOTOR_ID		0x010	
#define PITCH_MOTOR_ID		0x000
//Slave_ID 鍙戦€佺敤
//#define YAW_MOTOR_slaveID	0x000
#define PITCH_MOTOR_slaveID	0x001


//鏍规嵁鐢佃皟涓?鐨勫弬鏁拌繘琛岃?剧疆
#define P_MIN    -3.141593f
#define P_MAX     3.141593f
#define V_MIN    -45.0f
#define V_MAX     45.0f
#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN    -18.0f
#define T_MAX     18.0f

//鍏宠妭鐢垫満鏁版嵁缁撴瀯浣?
typedef struct
{
int8_t Err;
	//杞?鎹㈠嚭鐨勫疄闄呭€?
	float Position;		//浣嶇疆_寮у害
	float Angle;		//浣嶇疆_瑙掑害
	float Velocity;		//閫熷害
	float Torque;		//鎵?鐭?
	//鐢佃皟浼犲洖鐨勫€?
	int Position_int;
	int Velocity_int;
	int Torque_int;
	uint8_t T_MOS;		//椹卞姩MOS娓╁害
	uint8_t T_Rotor;	//鐢垫満鍐呴儴绾垮湀鐨勫钩鍧囨俯搴?
	//鎺у埗
	float Torque_SET;
} DM_motor_measure_t;

typedef enum
{
  vision_frount=0,
  vision_back,
  vision_lost,
}vision_state_t;
typedef enum
{

 no_block=0,
  left_lock,
  right_lock,
}lock_state_t;
typedef enum
{
  no_look=0,
  small_look,
  big_look,
  together_look,
  
}Vision_look_state_t;
typedef struct
{

 int16_t small_yaw_add;
  lock_state_t lock_state;
int16_t current_transform_angle;
  vision_state_t vision_state;
  uint8_t Vision_look_state;
  uint8_t vision_armor_id;
 uint8_t armor_dist;
}receive_gimbal_data_t;



/**
*@note 第一个头接在can2上
*@note 第二个头接在can1上
*/
#define SEND_ID_HEAD1            0x301
#define SHOOT_SEND_ID_HEAD1      0x303
#define SEND_ID_HEAD_REMOTE1     0x305

#define SEND_ID_HEAD2            0x302
#define SHOOT_SEND_ID_HEAD2      0x304
#define SEND_ID_HEAD_REMOTE2     0x306


#define RECEIVE_ID_2     0x402
#define RECEIVE_ID_1     0x401
void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t rx_data[]);
const SuperCap_t *get_supercup_pointer(void);


extern receive_gimbal_data_t receive_gimbal_data;
void get_gimbal_data_1(uint8_t rx_data[8],receive_gimbal_data_t *data);
void get_gimbal_data_2(uint8_t rx_data[8],receive_gimbal_data_t *data);

extern DM_motor_measure_t yaw_motor;
extern uint8_t get_gimbal_com_count;
extern motor_data_t trig_motor;
extern motor_data_t chassis_motor[4];

extern DM_motor_measure_t pitch_motor;


#endif
