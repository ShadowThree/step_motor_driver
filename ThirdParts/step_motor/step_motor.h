#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include <stdint.h>

#define LOG_ENABLE  1
#define CURVE_ACCELE_DECELE 1

typedef enum { MOTOR_DIR_IN = 0, MOTOR_DIR_OUT } MOTOR_DIR_t;

typedef enum {
  MOTOR_STA_STOP = 0,
  MOTOR_STA_ACCELE,
  MOTOR_STA_DECELE,
  MOTOR_STA_RUN
} MOTOR_STA_t;

typedef struct {
  MOTOR_DIR_t dir;
  MOTOR_STA_t sta;
  int32_t cur_pos;
  int32_t tar_pos;
  uint16_t def_step_num; // default step for acceleration and deceleration
  uint16_t accele_step;
	uint16_t uniform_step;
  uint16_t decele_step;
  uint16_t cur_step; // current step for acceleration and deceleration
} MOTOR_TYPE_t;

extern MOTOR_TYPE_t motor;

int32_t step_motor_get_pos(void);
uint8_t step_motor_set_pos(int32_t pos);
uint16_t get_next_speed(void);

#endif // __STEP_MOTOR_H__