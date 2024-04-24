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

#define TIM_PWM_START() HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1)
#define TIM_PWM_STOP() HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1)

MOTOR_TYPE_t motor = {.dir = MOTOR_DIR_IN,
                      .sta = MOTOR_STA_STOP,
                      .cur_pos = 0,
                      .tar_pos = 0,
                      .def_step_num = DEFAULT_STEP_NUM,
                      .accele_step = 0,
                      .decele_step = 0,
                      .cur_step = 0};

void step_motor_init(void) {}

int32_t step_motor_get_pos(void) { 
	M_DBG("motor info:\n");
	M_DBG("\tdir: %d\n", motor.dir);
	M_DBG("\tsta: %d\n", motor.sta);
	M_DBG("\tcur_pos: %d\n", motor.cur_pos);
	M_DBG("\ttar_pos: %d\n", motor.tar_pos);
	M_DBG("\taccele_step: %d\n", motor.accele_step);
	M_DBG("\tdecele_step: %d\n", motor.decele_step);
	M_DBG("\tcur_step: %d\n", motor.cur_step);
	return motor.cur_pos;
}

void step_motor_set_pos(int32_t pos) {
  switch (motor.sta) {
  case MOTOR_STA_STOP:
    if (motor.cur_pos == pos) {
      return;
    }
    if (pos > motor.cur_pos) {
      motor.dir = MOTOR_DIR_IN;
    } else {
      motor.dir = MOTOR_DIR_OUT;
    }
    if (abs(motor.cur_pos - pos) > motor.def_step_num * 2) {
      motor.accele_step = motor.def_step_num;
      motor.decele_step = motor.def_step_num;
    } else {
      motor.accele_step = abs(motor.cur_pos - pos) / 2;
      motor.decele_step = abs(motor.cur_pos - pos) - motor.accele_step;
    }
    if (motor.accele_step) {
      motor.sta = MOTOR_STA_ACCELE;
			M_DBG("Acceleration\n");
    } else {
      motor.sta = MOTOR_STA_DECELE;
			M_DBG("Deceleration\n");
    }
    break;
  case MOTOR_STA_ACCELE:
    break;
  case MOTOR_STA_DECELE:
    break;
  case MOTOR_STA_RUN:
    break;
  }

  if (motor.cur_pos == pos) {
    return;
  }

  motor.tar_pos = pos;
  TIM_PWM_START();
}

/**
 * @brief Get the next speed
 *
 * @return uint16_t range: 0~10000; 0: stop; 10000: max speed
 */
uint16_t get_next_speed(void) {
  switch (motor.sta) 
	{
  case MOTOR_STA_STOP:
    TIM_PWM_STOP();
    M_DBG("stop\n");
    break;
  case MOTOR_STA_ACCELE:
    if (++motor.cur_step == motor.accele_step) {
			M_DBG("Uniform motion\n");
      motor.sta = MOTOR_STA_RUN;
		}
    return (1 - COS(_PI * motor.cur_step / motor.accele_step)) / 2 * 10000;	
    break;
  case MOTOR_STA_DECELE:
    break;
  case MOTOR_STA_RUN:
    TIM_PWM_STOP();
    break;
  }
  return 0;
}
