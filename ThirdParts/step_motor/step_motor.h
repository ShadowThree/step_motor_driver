/**
 *	@author	shadowthreed@gmail.com
 *	@date	20240426
 *
		How To Use;
			0. according your setuation, modify the macro define in step_motor.c;

			1. set the target positon:

					step_motor_set_tar_pos(step_motor_get_cur_pos() + 1000);	// forward 1000 steps

			2. call motor_next_step(); in HAL_TIM_PWM_PulseFinishedCallback();

					#include "step_motor.h"
					void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
					{
						if(htim == &htim13) {
							motor_next_step();
						}
					}

 */
#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include <stdint.h>

typedef enum
{
	MOTOR_DIR_IN = 0,
	MOTOR_DIR_OUT
} MOTOR_DIR_t;

typedef enum
{
	MOTOR_STA_STOP = 0,
	MOTOR_STA_ACCELE,
	MOTOR_STA_DECELE,
	MOTOR_STA_RUN
} MOTOR_STA_t;

typedef struct
{
	MOTOR_DIR_t dir;
	MOTOR_STA_t sta;
	int32_t cur_pos;
	int32_t tar_pos;
	uint16_t def_step_num; // default step for acceleration and deceleration
	uint16_t accele_step;
	uint32_t uniform_step;
	uint16_t decele_step;
	uint16_t speed;	   // current speed
	uint32_t cur_step; // current step for acceleration and deceleration
} MOTOR_TYPE_t;

extern MOTOR_TYPE_t motor;
extern volatile uint8_t isMotorRunning;

void step_motor_set_cur_pos(int32_t pos);
int32_t step_motor_get_cur_pos(void);
uint8_t step_motor_set_tar_pos(int32_t pos);
uint16_t motor_next_step(void);
void MOTOR_E_Stop(void);

#endif // __STEP_MOTOR_H__