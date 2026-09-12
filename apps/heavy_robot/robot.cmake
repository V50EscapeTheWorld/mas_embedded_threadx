# 先加载默认模板，再覆盖差异

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（按板型覆盖默认值）
set(MODULES_GIMBAL   OFFLINE MOTOR)

# OFFLINE 默认参数
set(OFFLINE_WATCHDOG_ENABLE 1)    # 开启看门狗
set(OFFLINE_BEEP_ENABLE     1)    # 开启蜂鸣器
set(OFFLINE_TASK_STACK_SIZE 1024) # 任务栈大小
set(OFFLINE_TASK_PRIORITY   6)    # 任务优先级

# MOTOR 默认参数
set(MOTOR_TASK_STACK_SIZE   1024)   # 任务栈大小 
set(MOTOR_TASK_PRIORITY     12)     # 任务优先级
