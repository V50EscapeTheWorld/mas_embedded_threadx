#include "module_motor.h"
#include "power_control.h"
#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "Module_Motor"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  motor_thread;
APPS_STACK_SECTION static uint8_t motor_thread_stack[MOTOR_TASK_STACK_SIZE];
static TX_EVENT_FLAGS_GROUP motor_apply_evt;
#define MOTOR_APPLY_TRIGGER_BIT 1U /* 标志位: 力矩已更新, 可下发 */

/**
 * @brief 请求下发一次电机输出 (由上层在力矩更新后调用)
 * @note  本函数只置标志, 不做任何频率限制; 下发频率 = 上层调用频率。
 *        同一控制周期内重复调用会合并成一次下发(标志是比特, 置两次仍是一次)。
 */
void Motor_RequestApply(void)
{
    (void)tx_event_flags_set(&motor_apply_evt, MOTOR_APPLY_TRIGGER_BIT, TX_OR);
}

static void motor_task_entry(ULONG thread_input)
{
    ULONG actual_flags;

    while (1)
    {
        /* 完全事件触发: 没人请求就一直阻塞(无超时兜底, 频率与掉线策略由上层决定) */
        (void)tx_event_flags_get(&motor_apply_evt, MOTOR_APPLY_TRIGGER_BIT, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

        /* 控制计算 */
        Motor_ControlAll();
        /* 功率限制 */
        PowerControl_Update();
        /* 输出应用 */
        Motor_ApplyAll();
    }
}

void Module_Motor_Init(void)
{
    if (tx_event_flags_create(&motor_apply_evt, "motor_apply") != TX_SUCCESS)
    {
        LOG_E("Failed to create motor apply event flags");
        return;
    }

    UINT ret = tx_thread_create(&motor_thread, "motor", motor_task_entry, 0, motor_thread_stack, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY,
                                MOTOR_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (ret != TX_SUCCESS)
    {
        LOG_E("Failed to create motor thread");
    }

    LOG_I("Motor module initialized");
}
