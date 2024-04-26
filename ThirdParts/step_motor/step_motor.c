#include "step_motor.h"
#include "arm_math.h"
#include "tim.h"
#include <stdint.h>

#define MOTOR_LOG_ENABLE 1
#define CURVE_ACCELE_DECELE 1

#if MOTOR_LOG_ENABLE
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
#define DEFAULT_STEP_NUM 200

// abstracted speed range(NOT important)
#define MIN_SPEED (0)
#define MAX_SPEED (10000)

#define TIM_PWM_START() HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1)
#define TIM_PWM_STOP() HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1)

#define MOTOR_POWER_ON()  // HAL_GPIO_WritePin(POWER_DRV_GPIO_Port, POWER_DRV_Pin, GPIO_PIN_SET)
#define MOTOR_POWER_OFF() // HAL_GPIO_WritePin(POWER_DRV_GPIO_Port, POWER_DRV_Pin, GPIO_PIN_RESET)

#define MOTOR_SET_ENABLE(en) // HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, (GPIO_PinState)(!en))
#define MOTOR_SET_DIR(dir)	 // HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, (GPIO_PinState)(dir))

#if CURVE_ACCELE_DECELE
// calculate abstracted speed
#define MOTOR_CAL_SPEED(step) ((1 - COS(_PI * (step) / motor.def_step_num)) / 2 * (MAX_SPEED - MIN_SPEED) + MIN_SPEED)
#endif // CURVE_ACCELE_DECELE

/**
 * @brief IMPORTANT! Mapping the abstracted speed to TIM PWM frequence
 *
 * 		timClk / (Prescaler+1) / (Period+1) / Motor_one_round_pulse * 60s = motor speed in round/min
 * eg: 84MHz / (20+1) / (999+1) / 1600pulse/round * 60s = 150round/min
 *
 *		(Period+1)		Speed(round/min)
 *		   250					  600
 *		   500					  300
 *		   750					  200
 *		  1000					  150
 *		  1250					  120
 *		  ...							...
 *		 20000						7.5
 */
#define MIN_TIM_PERIOD (500 - 1)   // max speed
#define MAX_TIM_PERIOD (20000 - 1) // min speed
#define MOTOR_SET_SPEED(spd) __HAL_TIM_SET_AUTORELOAD(&htim1, (MAX_TIM_PERIOD - (MAX_TIM_PERIOD - MIN_TIM_PERIOD) * (spd) / (MAX_SPEED - MIN_SPEED)))

MOTOR_TYPE_t motor = {
	.dir = MOTOR_DIR_IN,
	.sta = MOTOR_STA_STOP,
	.cur_pos = 0,
	.tar_pos = 0,
	.def_step_num = DEFAULT_STEP_NUM,
	.accele_step = 0,
	.uniform_step = 0,
	.decele_step = 0,
	.cur_step = 0,
	.speed = MIN_SPEED};

volatile uint8_t isMotorRunning = 0;
static volatile uint32_t cnt_pulse;

void motion_info(void)
{
	M_DBG("motion info:\n");
	M_DBG("\tdir:						%s\n", (motor.dir) ? "IN" : "OUT");
	M_DBG("\tcur_pos: 			%d\n", motor.cur_pos);
	M_DBG("\ttar_pos: 			%d\n", motor.tar_pos);
	M_DBG("\tmove_pos:			%d\n", motor.tar_pos - motor.cur_pos);
	M_DBG("\taccele_step: 	%d\n", motor.accele_step);
	M_DBG("\tuniform_step:	%d\n", motor.uniform_step);
	M_DBG("\tdecele_step: 	%d\n\n", motor.decele_step);
}

void step_motor_set_cur_pos(int32_t pos)
{
	motor.cur_pos = pos;
}

int32_t step_motor_get_cur_pos(void)
{
	return motor.cur_pos;
}

uint8_t step_motor_set_tar_pos(int32_t pos)
{
	M_DBG("\n\n/************** tarStep:%d ****************/\n", pos);

	if (motor.sta != MOTOR_STA_STOP)
	{
		M_ERR("Motor is running, can NOT set the new target position\n");
		return 1;
	}

	if (pos == motor.cur_pos)
	{
		M_DBG("target position is current position\n");
		return 0;
	}

	MOTOR_POWER_ON();
	MOTOR_SET_ENABLE(1);

	if (pos > motor.cur_pos)
	{
		motor.dir = MOTOR_DIR_IN;
		MOTOR_SET_DIR(MOTOR_DIR_IN);
	}
	else
	{
		motor.dir = MOTOR_DIR_OUT;
		MOTOR_SET_DIR(MOTOR_DIR_OUT);
	}

	if (abs(motor.cur_pos - pos) > motor.def_step_num * 2)
	{
		motor.accele_step = motor.def_step_num;
		motor.uniform_step = abs(motor.cur_pos - pos) - motor.def_step_num * 2;
		motor.decele_step = motor.def_step_num;
	}
	else
	{
		motor.decele_step = abs(motor.cur_pos - pos) / 2;
		motor.uniform_step = 0;
		motor.accele_step = abs(motor.cur_pos - pos) - motor.decele_step;
	}

	motor.sta = MOTOR_STA_ACCELE;
	motor.cur_step = 1;
	motor.tar_pos = pos;
	motor.speed = MOTOR_CAL_SPEED(1); // (1 - COS(_PI * 1 / motor.def_step_num)) / 2 * 10000;
	motor.cur_step = 1;
	motion_info();

	M_DBG("motor acceleration\n");
	isMotorRunning = 1;
	MOTOR_SET_SPEED(motor.speed);
	TIM_PWM_START();

	return 0;
}
/**
 * @brief Get the next speed
 *
 * @return uint16_t range: [MIN_SPEED, MAX_SPEED]; MIN_SPEED: stop; MAX_SPEED: max speed
 */
uint16_t motor_next_step(void)
{
	if (motor.dir == MOTOR_DIR_IN)
	{
		motor.cur_pos++;
	}
	else
	{
		motor.cur_pos--;
	}

	M_DBG("\tpos,%4d, step,%3d, spd,%5d, \n", motor.cur_pos, motor.cur_step, motor.speed);

	motor.cur_step++;
	if (motor.sta == MOTOR_STA_ACCELE)
	{
		if (motor.cur_step > motor.accele_step)
		{
			motor.cur_step = 1;
			if (motor.decele_step == 0)
			{
				motor.sta = MOTOR_STA_STOP;
				TIM_PWM_STOP();
				MOTOR_SET_ENABLE(0);
				MOTOR_POWER_OFF();
			}
			else if (abs(motor.cur_pos - motor.tar_pos) > motor.decele_step)
			{
				motor.sta = MOTOR_STA_RUN;
				motor.speed = MAX_SPEED;
				MOTOR_SET_SPEED(motor.speed);
				M_DBG("motor uniform motion\n");
			}
			else
			{
				motor.sta = MOTOR_STA_DECELE;
				M_DBG("motor deceleration\n");
			}
		}
		else
		{
			motor.speed = MOTOR_CAL_SPEED(motor.cur_step); // (1 - COS(_PI * motor.cur_step / motor.def_step_num)) / 2 * 10000;
			MOTOR_SET_SPEED(motor.speed);
			if (motor.cur_step == 1)
			{
				TIM_PWM_START();
			}
		}
	}

	if (motor.sta == MOTOR_STA_RUN)
	{
		if (motor.cur_step > motor.uniform_step)
		{
			motor.cur_step = 1;
			motor.sta = MOTOR_STA_DECELE;
			M_DBG("motor deceleration\n");
		}
	}

	if (motor.sta == MOTOR_STA_DECELE)
	{
		if (motor.cur_step > motor.decele_step)
		{
			motor.sta = MOTOR_STA_STOP;
			TIM_PWM_STOP();
			MOTOR_SET_ENABLE(0);
			MOTOR_POWER_OFF();
		}
		else
		{
			motor.speed = MOTOR_CAL_SPEED((motor.decele_step - motor.cur_step + 1)); // (1 - COS(_PI * (motor.decele_step - motor.cur_step + 1) / motor.def_step_num)) / 2 * 10000;
			MOTOR_SET_SPEED(motor.speed);
		}
	}

	cnt_pulse++;
	if (motor.sta == MOTOR_STA_STOP)
	{
		motor.accele_step = 0;
		motor.uniform_step = 0;
		motor.decele_step = 0;
		motor.cur_step = 0;
		motor.speed = MIN_SPEED;
		MOTOR_SET_SPEED(motor.speed);

		M_DBG("total pulse: %d\n", cnt_pulse);
		cnt_pulse = 0;
		isMotorRunning = 0;
	}

	return motor.speed;
}

void MOTOR_E_Stop(void)
{
	TIM_PWM_STOP();
	MOTOR_SET_ENABLE(0);
	MOTOR_POWER_OFF();

	M_DBG("Motor E-Stop\n");
	cnt_pulse = 0;
	isMotorRunning = 0;
	motor.sta = MOTOR_STA_STOP;
	motor.accele_step = 0;
	motor.uniform_step = 0;
	motor.decele_step = 0;
	motor.cur_step = 0;
	motor.speed = MIN_SPEED;
	MOTOR_SET_SPEED(motor.speed);
}
