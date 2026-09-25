#include "robot_control.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "heavy_robot_def.h" 
#include "arm_func.h"
#include "module_motor.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];


static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;
    while (1)
    {
        #if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
        remote_ctrl_arm();
        #endif
        arm_func();
        /* 力矩已更新, 立刻触发下发 */
        Motor_RequestApply();
        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;
    
    if (!arm_init())
    {
        LOG_E("arm_init failed!");
        return;
    }

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0,
                              robot_control_thread_stack, 1024, 30, 30, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
