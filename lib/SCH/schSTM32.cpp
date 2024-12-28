#include <Arduino.h>
#include <schSTM32.h>

/* Used to display the error code */
tByte Error_code_G = 0;
HardwareTimer *MyTim = new HardwareTimer(TIM6);// declare object MyTim;


sTask SCH_tasks_G[SCH_MAX_TASKS];
/*--------------------------------------------------------*/
void SCH_Update(void) 
{
tByte Index;
//HAL_ResumeTick();
/* NOTE: calculations are in *TICKS* (not milliseconds) */
for (Index = 0; Index < SCH_MAX_TASKS; Index++)
{
/* Check if there is a task at this location */
if (SCH_tasks_G[Index].pTask)
{
if (--SCH_tasks_G[Index].Delay == 0)
{
/* The task is due to run */
SCH_tasks_G[Index].RunMe += 1; /* Inc. 'RunMe' flag */
if (SCH_tasks_G[Index].Period)
{
/* Schedule regular tasks to run again */
SCH_tasks_G[Index].Delay = SCH_tasks_G[Index].Period;
}
}
}
}

}

/*--------------------------------------------------------*/
void SCH_Init(uint32_t intervall, TimerFormat_t format)
{
tByte i;

  MyTim->setOverflow(intervall, format); 
  MyTim->attachInterrupt(SCH_Update);//configure the function Update_IT_callback as an interrupt for the Update event
 
for (i = 0; i < SCH_MAX_TASKS; i++)
{
SCH_Delete_Task(i);
}
/* SCH_Delete_Task() will generate an error code,
because the task array is empty.
-> reset the global error variable. */
Error_code_G = 0;
}


/*--------------------------------------------------------*-
SCH_Add_Task()
Causes a task (function) to be executed at regular
intervals, or after a user-defined delay.
-*--------------------------------------------------------*/
tByte SCH_Add_Task(void (* pFunction)(),const tWord DELAY,const tWord PERIOD)
{
tByte Index = 0;
/* First find a gap in the array (if there is one) */
while ((SCH_tasks_G[Index].pTask != 0) && (Index < SCH_MAX_TASKS))
{
Index++;
}
/* Have we reached the end of the list? */
if (Index == SCH_MAX_TASKS)
{
/* Task list is full
-> set the global error variable */
Error_code_G = ERROR_SCH_TOO_MANY_TASKS;
/* Also return an error code */
return SCH_MAX_TASKS;
}
/* If we're here, there is a space in the task array */
SCH_tasks_G[Index].pTask = pFunction;
SCH_tasks_G[Index].Delay = DELAY + 1;
SCH_tasks_G[Index].Period = PERIOD;
SCH_tasks_G[Index].RunMe = 0;
return Index; /* return pos. of task (to allow deletion) */
}

/*--------------------------------------------------------*-
SCH_Dispatch_Tasks()
This is the 'dispatcher' function. When a task (function)
is due to run, SCH_Dispatch_Tasks() will run it.
This function must be called (repeatedly) from the main loop.
-*--------------------------------------------------------*/
void SCH_Dispatch_Tasks(void)
{
tByte Index;
/* Dispatches (runs) the next task (if one is ready) */
for (Index = 0; Index < SCH_MAX_TASKS; Index++)
{
if (SCH_tasks_G[Index].RunMe > 0)
{
(*SCH_tasks_G[Index].pTask)(); /* Run the task */
SCH_tasks_G[Index].RunMe -= 1; /* Reduce RunMe count */
/* Periodic tasks will automatically run again
- if this is a 'one shot' task, delete it */
if (SCH_tasks_G[Index].Period == 0)
{
SCH_Delete_Task(Index);
}
}
}
/* Report system status */
SCH_Report_Status();
/* The scheduler enters idle mode at this point */
SCH_Go_To_Sleep();
}

/*--------------------------------------------------------*/
void SCH_Start(void)
{
 MyTim->resume();// start Timer
}

tByte SCH_Delete_Task(const tWord TASK_INDEX)
{
tByte Return_code;
if (SCH_tasks_G[TASK_INDEX].pTask == 0)
{
/* No task at this location...
-> set the global error variable */
Error_code_G = ERROR_SCH_CANNOT_DELETE_TASK;
/* ...also return an error code */
Return_code = RETURN_ERROR;
}
else
{
Return_code = RETURN_NORMAL;
}
SCH_tasks_G[TASK_INDEX].pTask = 0x0000;
SCH_tasks_G[TASK_INDEX].Delay = 0;
SCH_tasks_G[TASK_INDEX].Period = 0;
SCH_tasks_G[TASK_INDEX].RunMe = 0;
return Return_code; /* return status */
}

void SCH_Go_To_Sleep(void)
{
 //HAL_SuspendTick();
 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); 
}

/*--------------------------------------------------------*/
void SCH_Report_Status(void)
{
#ifdef SCH_REPORT_ERRORS
/* ONLY APPLIES IF WE ARE REPORTING ERRORS */
/* Check for a new error code */
if (Error_code_G != Last_error_code_G)
{
/* Negative logic on LEDs assumed */
Error_port = 255 - Error_code_G;
Last_error_code_G = Error_code_G;
if (Error_code_G != 0)
{
Error_tick_count_G = 60000;
}
else
{
Error_tick_count_G = 0;
}
}
else
{
if (Error_tick_count_G != 0)
{
if (--Error_tick_count_G == 0)
{
Error_code_G = 0; /* Reset error code */
}
}
}
#endif
}


