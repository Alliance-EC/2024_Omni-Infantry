#include "motor_task.h"

#include "dji_motor.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "servo_motor.h"

void MotorControlTask()
{
    DJIMotorControl();

    ServeoMotorControl();
}
