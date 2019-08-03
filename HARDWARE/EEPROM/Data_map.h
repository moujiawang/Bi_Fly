#ifndef __DATAMAP_H
#define __DATAMAP_H

#define  PAGE_Config    (0x08000000 + 62 * 1024) //将配置信息存放在第62页Flash


struct data_map{
	int16_t is_good;   					//数据是否有效
	int16_t dGx_offset;
	int16_t dGy_offset;
	int16_t dGz_offset;
	
	int16_t dMx_offset;
	int16_t dMy_offset;
	int16_t dMz_offset;
	float  dMx_scale;
	float  dMy_scale;
	float  dMz_scale;
	
//ROLL
	float Kp_ROLL_RATE;					
	float Ki_ROLL_RATE;
	float Kd_ROLL_RATE;
	float Limit_ROLL_RATE;       		//output limit MAX
	float LimitLow_ROLL_RATE;    		//output limit MIN
	float Kp_ROLL_ANGLE;					
	float Ki_ROLL_ANGLE;
	float Kd_ROLL_ANGLE;
	float Limit_ROLL_ANGLE;       		//output limit MAX
	float LimitLow_ROLL_ANGLE;    		//output limit MIN
//YAW
	float Kp_YAW_RATE;					
	float Ki_YAW_RATE;
	float Kd_YAW_RATE;
	float Limit_YAW_RATE;       		//output limit MAX
	float LimitLow_YAW_RATE;    		//output limit MIN
	float Kp_YAW_ANGLE;					
	float Ki_YAW_ANGLE;
	float Kd_YAW_ANGLE;
	float Limit_YAW_ANGLE;       		//output limit MAX
	float LimitLow_YAW_ANGLE;    		//output limit MIN
	float Kp_PITCH_RATE;	
//PITCH	
	float Ki_PITCH_RATE;
	float Kd_PITCH_RATE;
	float Limit_PITCH_RATE;       		//output limit MAX
	float LimitLow_PITCH_RATE;    		//output limit MIN
	float Kp_PITCH_ANGLE;					
	float Ki_PITCH_ANGLE;
	float Kd_PITCH_ANGLE;
	float Limit_PITCH_ANGLE;       		//output limit MAX
	float LimitLow_PITCH_ANGLE;    		//output limit MIN
	
};

extern struct data_map Config;


#endif

//------------------End of File----------------------------
