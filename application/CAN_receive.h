#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/***ç”µæœºid***/
typedef enum
{
	/*µ×ÅÌ3508ĞĞ½øµç»ú*/
	FR_3508_ID = 0X201,
	BR_3508_ID = 0X202,
	BL_3508_ID = 0X203,
	FL_3508_ID = 0X204,
	
	
	
	/*µ×ÅÌ¶æÏòµç»ú*/
	
	
	FR_6020_ID = 0x205,
	BR_6020_ID = 0x206,
	BL_6020_ID = 0x207,
	FL_6020_ID = 0x208,
	//BARREL_MOTOR_ID = 0X204, 


	/************************************äº‘å°yawè½´id*******************************/
	
	//PITCH_MOTOR_ID = 0x206,


    /***********************************è¶…çº§ç”µå?¹id*******************************/

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
	float C_Vol;	//ç”µå?¹ç”µå?
	float C_Power;	//ç”µå?¹è¾“å‡ºåŠŸç?
	float Mode_C;	//ç”µå?¹çŠ¶æ€ç 
	float C_Current;	//ç”µå?¹è¾“å‡ºç”µæµ?
	float PowerData[4];
} SuperCap_t;

/***************************************4310ç”µæœº**************************************/
//Master_ID æ¥æ”¶ç”?
//#define YAW_MOTOR_ID		0x010	
#define PITCH_MOTOR_ID		0x000
//Slave_ID å‘é€ç”¨
//#define YAW_MOTOR_slaveID	0x000
#define PITCH_MOTOR_slaveID	0x001


//æ ¹æ®ç”µè°ƒä¸?çš„å‚æ•°è¿›è¡Œè?¾ç½®
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

//å…³èŠ‚ç”µæœºæ•°æ®ç»“æ„ä½?
typedef struct
{
int8_t Err;
	//è½?æ¢å‡ºçš„å®é™…å€?
	float Position;		//ä½ç½®_å¼§åº¦
	float Angle;		//ä½ç½®_è§’åº¦
	float Velocity;		//é€Ÿåº¦
	float Torque;		//æ‰?çŸ?
	//ç”µè°ƒä¼ å›çš„å€?
	int Position_int;
	int Velocity_int;
	int Torque_int;
	uint8_t T_MOS;		//é©±åŠ¨MOSæ¸©åº¦
	uint8_t T_Rotor;	//ç”µæœºå†…éƒ¨çº¿åœˆçš„å¹³å‡æ¸©åº?
	//æ§åˆ¶
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
*@note µÚÒ»¸öÍ·½ÓÔÚcan2ÉÏ
*@note µÚ¶ş¸öÍ·½ÓÔÚcan1ÉÏ
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
