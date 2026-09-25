/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/module_motor.h
 * @Description:
 */
#ifndef _MODULE_MOTOR_H_
#define _MODULE_MOTOR_H_

#include "motor_def.h"
#include "motor_base.h"

#include "motor_dji.h"
#include "motor_damiao.h"
#include "motor_servo.h"
#include "motor_zdt.h"
#include "motor_lk.h"
#ifndef MOTOR_TASK_STACK_SIZE
#define MOTOR_TASK_STACK_SIZE 1024 /* 检测任务栈 (字节) */
#endif
#ifndef MOTOR_TASK_PRIORITY
#define MOTOR_TASK_PRIORITY 12 /* 检测任务优先级 */
#endif

/**
 * @brief 初始化电机模块
 */
void Module_Motor_Init(void);

/**
 * @brief   请求下发一次电机输出 (完全事件触发: 上层每控制周期调用一次)
 * @warning 使用本模块的工程必须保证有人周期性地调用它, 否则电机不会有任何输出。
 */
void Motor_RequestApply(void);

#endif /* _MODULE_MOTOR_H_ */
