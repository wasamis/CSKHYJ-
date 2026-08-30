#include "ArmTask.hpp"

#include "arm.h"
#include "community.h"

/*
 * ============================================================
 * ArmTask
 * ============================================================
 *
 * This module is the adapter between Community and arm.c.
 *
 * Current Community interfaces used here:
 *
 *   0x05 -> Community_TakeArmGrabRequest()
 *          -> Arm_StartGrab()
 *
 *   0x0B -> Community_TakeArmBuildRequest()
 *          -> Arm_StartBuild()
 *
 *   0x07 -> Community_IsStopRequested()
 *          -> Arm_Stop()
 *
 * The old 0x01 wrist-parameter command is no longer used here.
 *
 * ArmTask does not directly control chassis motors.
 */

/*
 * Prevent repeated STOP debug output while 0x07 remains pending.
 *
 * Community owns the stop flag. ArmTask only observes it and stops
 * the arm; it does not clear the flag because the chassis layer may
 * also need to process the same STOP request.
 */
static uint8_t s_arm_stop_handled = 0U;

extern "C" void ArmTask_Init(void)
{
    /*
     * ArmTask owns initialization of the mechanical-arm module.
     *
     * Main only needs to call:
     *
     *     ArmTask_Init();
     *
     * once after TIM/UART/GPIO peripherals have been initialized.
     */
    Arm_Init();

    s_arm_stop_handled = 0U;

    Community_SendDebugString("ArmTask Init OK\r\n");
}

extern "C" void ArmTask_Process(void)
{
    /*
     * ========================================================
     * 1. Global STOP request
     * ========================================================
     *
     * Community 0x07 means "stop all".
     *
     * Do not call Community_ClearStopRequest() here.
     * Move/chassis may also need to see the same STOP request.
     */
    if (Community_IsStopRequested() != 0U)
    {
        if (s_arm_stop_handled == 0U)
        {
            Arm_Stop();
            s_arm_stop_handled = 1U;

            Community_SendDebugString("ARM STOP\r\n");
        }

        return;
    }

    /*
     * STOP has been cleared by the global/chassis task manager.
     * Allow the next STOP event to be handled again.
     */
    s_arm_stop_handled = 0U;

    /*
     * ========================================================
     * 2. Grab request: Community command 0x05
     * ========================================================
     *
     * The request flag is consumed here.
     * It does not enter the normal chassis mission queue.
     */
    if (Community_TakeArmGrabRequest() != 0U)
    {
        /*
         * Keep the behavior of the previous project:
         * once an arm action is requested, discard queued chassis
         * movement commands that have not started yet.
         *
         * This does not directly stop a chassis task that is already
         * active; global task arbitration should handle that if needed.
         */
        Community_ClearQueue();

        if (Arm_StartGrab() != 0U)
        {
            Community_SendDebugString("ARM GRAB START\r\n");
        }
        else
        {
            Community_SendDebugString("ARM GRAB BUSY\r\n");
        }

        return;
    }

    /*
     * ========================================================
     * 3. Build request: Community command 0x0B
     * ========================================================
     */
    if (Community_TakeArmBuildRequest() != 0U)
    {
        Community_ClearQueue();

        if (Arm_StartBuild() != 0U)
        {
            Community_SendDebugString("ARM BUILD START\r\n");
        }
        else
        {
            Community_SendDebugString("ARM BUILD BUSY\r\n");
        }

        return;
    }

    /*
     * ========================================================
     * 4. Build state machine is running
     * ========================================================
     */
    if (Arm_IsBuildBusy() != 0U)
    {
        Arm_Process();
        return;
    }

    /*
     * ========================================================
     * 5. Build just finished
     * ========================================================
     *
     * Current Community provides no dedicated build-finish frame,
     * so keep using Community_SendFinish() here.
     */
    if (Arm_TakeBuildFinished() != 0U)
    {
        Community_SendFinish();
        Community_SendDebugString("ARM BUILD DONE\r\n");

        return;
    }

    /*
     * ========================================================
     * 6. Grab state machine is running
     * ========================================================
     */
    if (Arm_IsBusy() != 0U)
    {
        Arm_Process();
        return;
    }

    /*
     * ========================================================
     * 7. Grab just finished
     * ========================================================
     *
     * Current Community behavior:
     *
     *   Community_SendArmFinish() -> arm-specific finish frame
     *   Community_SendFinish()    -> general all-done frame
     */
    if (Arm_TakeFinished() != 0U)
    {
        Community_SendArmFinish();
        Community_SendFinish();

        Community_SendDebugString("ARM GRAB DONE\r\n");

        return;
    }
}
