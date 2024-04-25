#include "step_motor.h"
#include "arm_math.h"
#include "tim.h"
#include <stdint.h>

#if LOG_ENABLE
#include <dbger.h>
#define M_DBG(fmt, ...) LOG_DBG(fmt, ##__VA_ARGS__)
#define M_ERR(fmt, ...) LOG_ERR(fmt, ##__VA_ARGS__)
#else
#define M_DBG(fmt, ...)
#define M_ERR(fmt, ...)
#endif

#define abs(x) ((x) > 0 ? (x) : -(x))

#if CURVE_ACCELE_DECELE
#define _PI PI
#define COS(x) arm_cos_f32(x)
#endif // CURVE_ACCELE_DECELE

// default step number for acceleration and deceleration
#define DEFAULT_STEP_NUM 100

#define TIM_PWM_START() HAL_TIM_PWM_Start_IT(&htim13, TIM_CHANNEL_1)
#define TIM_PWM_STOP() 	HAL_TIM_PWM_Stop_IT(&htim13, TIM_CHANNEL_1)

#define MOTOR_SET_ENABLE(en)	HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, (GPIO_PinState)(en))
#define MOTOR_SET_DIR(dir)		HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, (GPIO_PinState)(dir))

#define MOTOR_SET_SPEED(spd)	__HAL_TIM_SET_AUTORELOAD(&htim13, spd)

MOTOR_TYPE_t motor = {.dir = MOTOR_DIR_IN,
                      .sta = MOTOR_STA_STOP,
                      .cur_pos = 0,
                      .tar_pos = 0,
                      .def_step_num = DEFAULT_STEP_NUM,
                      .accele_step = 0,
											.uniform_step = 0,
                      .decele_step = 0,
                      .cur_step = 0};

void step_motor_init(void) {}

int32_t step_motor_get_pos(void) { 
	M_DBG("motor info:\n");
	M_DBG("\tdir:						%d\n", motor.dir);
	M_DBG("\tsta: 					%d\n", motor.sta);
	M_DBG("\tcur_pos: 			%d\n", motor.cur_pos);
	M_DBG("\ttar_pos: 			%d\n", motor.tar_pos);
	M_DBG("\taccele_step: 	%d\n", motor.accele_step);
	M_DBG("\tuniform_step:	%d\n", motor.uniform_step);
	M_DBG("\tdecele_step: 	%d\n", motor.decele_step);
	M_DBG("\tcur_step:			%d\n\n", motor.cur_step);
	return motor.cur_pos;
}

uint8_t step_motor_set_pos(int32_t pos)
{
	if(motor.sta != MOTOR_STA_STOP) {
		M_ERR("Motor is running, can NOT set the new target position\n");
		return 1;
	}
	
	if(pos == motor.cur_pos) {
		M_DBG("target position is current position\n");
		return 0;
	}
	
	// MOTOR_POWER_ON();
	MOTOR_SET_ENABLE(1);
	
	if (pos > motor.cur_pos) {
		motor.dir = MOTOR_DIR_IN;
		MOTOR_SET_DIR(MOTOR_DIR_IN);
  } else {
		motor.dir = MOTOR_DIR_OUT;
		MOTOR_SET_DIR(MOTOR_DIR_OUT);
  }
	
  if (abs(motor.cur_pos - pos) > motor.def_step_num * 2) {
		motor.accele_step = motor.def_step_num;
		motor.uniform_step = abs(motor.cur_pos - pos) - motor.def_step_num * 2;
		motor.decele_step = motor.def_step_num;
  } else {
		motor.accele_step = abs(motor.cur_pos - pos) / 2;
		motor.uniform_step = 0;
		motor.decele_step = abs(motor.cur_pos - pos) - motor.accele_step;
  }
	
  if (motor.accele_step) {
		motor.sta = MOTOR_STA_ACCELE;
  } else {
		motor.sta = MOTOR_STA_DECELE;
  }
	motor.cur_step = 0;
	motor.tar_pos = pos;
	step_motor_get_pos();
	
	M_DBG("%s\n", (motor.sta == MOTOR_STA_ACCELE) ? "motor acceleration" : "motor deceleration");
	
	get_next_speed();
	
	return 0;
}

/**
 * @brief Get the next speed
 *
 * @return uint16_t range: 0~10000; 0: stop; 10000: max speed
 */
uint16_t get_next_speed(void)
{
	static uint16_t speed = 0;
	
	if(motor.dir == MOTOR_DIR_IN) {
		motor.cur_pos++;
	} else {
		motor.cur_pos--;
	}

	motor.cur_step++;
	if(motor.sta == MOTOR_STA_ACCELE) {
		if(motor.cur_step > motor.accele_step) {
			motor.cur_step = 1;
			if(abs(motor.cur_pos - motor.tar_pos) > motor.decele_step) {
				motor.sta = MOTOR_STA_RUN;
				speed = 10000;
				M_DBG("motor uniform motion\n");
			} else {
				motor.sta = MOTOR_STA_DECELE;
				M_DBG("motor deceleration\n");
			}
		} else {
			speed = (1 - COS(_PI * motor.cur_step / motor.accele_step)) / 2 * 10000;
		}
	}
	
	if(motor.sta == MOTOR_STA_RUN) {
		if(motor.cur_step > motor.uniform_step) {
			motor.cur_step = 1;
			motor.sta = MOTOR_STA_DECELE;
			M_DBG("motor deceleration\n");
		}
	}

	if(motor.sta == MOTOR_STA_DECELE) {
		if(motor.cur_step > motor.decele_step) {
			extern volatile uint16_t cnt_pulse;
			motor.sta = MOTOR_STA_STOP;
			motor.cur_step = 0;
			speed = 0;
			MOTOR_SET_ENABLE(0);
			// MOTOR_POWER_OFF();
			M_DBG("total pulse: %d\n", cnt_pulse);
		} else {
			speed = (1 + COS(_PI * (motor.cur_step - 1) / motor.decele_step)) / 2 * 10000;
		}
	}

	if(speed != 0) {
		//MOTOR_SET_SPEED(speed);
		MOTOR_SET_SPEED(65535);
		TIM_PWM_START();
	}
	M_DBG("pos,%4d, step,%3d, spd,%5d, \n", motor.cur_pos, motor.cur_step, speed);
	return speed;
}
