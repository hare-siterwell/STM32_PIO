/**
 * @file bsp_tim_a4988.c
 * @brief 步进电机驱动
 */

#include "bsp_tim_a4988.h"

#define FRAC_MAX 30000 // 频率阈值
struct Motor motor1;

/**
 * @brief 初始化步进电机
 */
void motor_arg_init(void) {
  motor1.step_gpio_array = MOTOR1_STEP_GPIO_Port;
  motor1.step_gpio = MOTOR1_STEP_Pin;
  motor1.dir_gpio_array = MOTOR1_DIR_GPIO_Port;
  motor1.dir_gpio = MOTOR1_DIR_Pin;
  motor1.ms1_gpio_array = MOTOR1_MS1_GPIO_Port;
  motor1.ms1_gpio = MOTOR1_MS1_Pin;
  motor1.ms2_gpio_array = MOTOR1_MS2_GPIO_Port;
  motor1.ms2_gpio = MOTOR1_MS2_Pin;
  motor1.ms3_gpio_array = MOTOR1_MS3_GPIO_Port;
  motor1.ms3_gpio = MOTOR1_MS3_Pin;
}

/**
 * @brief 设置步进电机细分
 * @param motor 选择电机
 * @param divnum 细分,1/2/4/8/16
 */
void motor_divnum(struct Motor *motor, u16 divnum) {
  u8 ms1 = 0, ms2 = 0, ms3 = 0;
  switch (divnum) {
  case 2: // 2细分
    ms1 = 1;
    motor->divnum = 2;
    break;
  case 4: // 4细分
    ms2 = 1;
    motor->divnum = 4;
    break;
  case 8: // 8细分
    ms1 = ms2 = 1;
    motor->divnum = 8;
    break;
  case 16: // 16细分
    ms1 = ms2 = ms3 = 1;
    motor->divnum = 16;
    break;
  default: // 全细分
    motor->divnum = 1;
    break;
  }
  MS1_OUTPUT(ms1);
  MS2_OUTPUT(ms2);
  MS3_OUTPUT(ms3);
}

/**
 * @brief 启动步进电机
 * @param motor 选择电机
 * @param divnum 细分,1/2/4/8/16
 * @param step_move 运行脉冲数
 * @param step_spmax 最大速度
 * @param step_accel 加速度
 */
int motor_move(struct Motor *motor, u16 divnum, s32 step_move, u16 step_spmax,
               u16 step_accel) {
  if (!step_move || !step_spmax || !step_accel) {
    motor->state = stopped_state;
    STEP_TIM(0);
    return 1;
  }

  if (step_move < 0) { // 步数为负,反转
    DIR_OUTPUT(0);
    step_move = -step_move;
  } else {
    DIR_OUTPUT(1);
  }

  motor_divnum(motor, divnum);
  motor->step_move = step_move;
  motor->step_spmax = step_spmax > FRAC_MAX / motor->divnum
                          ? FRAC_MAX / motor->divnum
                          : step_spmax;
  motor->step_accel = step_accel;

  motor->step_count = 0;
  motor->step_acced = 0;
  motor->step_frac = 0;
  motor->speed_frac = 0;
  motor->step_speed = 0;

  motor->state = acceleration_state;
  STEP_TIM(1);
  return 0;
}

/**
 * @brief 加减速控制,中断里运行
 * @param motor 选择电机
 */
void motor_spta_algorithm(struct Motor *motor) {
  if (motor->state == stopped_state)
    return;

  // 拉低脉冲信号
  STEP_OUTPUT(0);

  motor->step_frac += motor->step_speed; // 叠加步数
  u32 carry = motor->step_frac / FRAC_MAX;
  if (carry) { // 溢出
    motor->step_frac -= FRAC_MAX;
    motor->step_count++;
    // 拉高脉冲信号产生一个步进脉冲
    STEP_OUTPUT(1);
  }

  // 根据电机的状态进行工作
  switch (motor->state) {
  case acceleration_state: // 加速阶段
    if (carry) {
      motor->step_acced++; // 记录加速的步数
    }

    motor->speed_frac += motor->step_accel * motor->divnum; // 叠加速度
    carry = motor->speed_frac / (FRAC_MAX * 2);
    if (carry) {
      motor->speed_frac -= FRAC_MAX * 2;
      motor->step_speed += carry;
    }

    if (motor->step_speed >= motor->step_spmax * motor->divnum) { // 达到匀速
      motor->step_speed = motor->step_spmax * motor->divnum;
      motor->state = uniform_state;
    }

    if (motor->step_move > 1) {
      if (motor->step_count >= motor->step_move / 2) { // 达到中值转为减速
        motor->state = deceleration_state;
      }
    } else if (motor->step_count) { // 只走1步
      motor->state = deceleration_state;
    }
    break;

  case uniform_state: // 匀速阶段
    if (motor->step_move - motor->step_count <= motor->step_acced) {
      motor->state = deceleration_state;
    }
    break;

  case deceleration_state: // 减速阶段
    motor->speed_frac += motor->step_accel * motor->divnum;
    carry = motor->speed_frac / (FRAC_MAX * 2);
    if (carry) {
      motor->speed_frac -= FRAC_MAX * 2;
      if (motor->step_speed >= carry) {
        motor->step_speed -= carry;
      }
    }

    if (motor->step_count >= motor->step_move) {
      motor->state = stopped_state;
      STEP_TIM(0);
    }
    break;

  default:
    break;
  }
}
