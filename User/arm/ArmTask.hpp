#ifndef ARM_TASK_HPP
#define ARM_TASK_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * Initialize the mechanical-arm task layer.
 *
 * This function also calls Arm_Init().
 * Call it once after CubeMX peripheral initialization.
 */
void ArmTask_Init(void);

/*
 * Non-blocking mechanical-arm task processing.
 *
 * Call repeatedly in the main loop.
 */
void ArmTask_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TASK_HPP */
