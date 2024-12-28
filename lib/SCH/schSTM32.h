#include <Arduino.h>

#define tWord uint16_t
#define tByte unsigned char
#define tLong uint32_t  
/* Store in DATA area, if possible, for rapid access
Total memory per task is 7 bytes */
typedef struct
{
/* Pointer to the task (must be a 'void (void)' function) */
void (* pTask)(void);
/* Delay (ticks) until the function will (next) be run
- see SCH_Add_Task() for further details */
tWord Delay;
/* Interval (ticks) between subsequent runs.
- see SCH_Add_Task() for further details */
tWord Period;
/* Incremented (by scheduler) when task is due to execute */
tByte RunMe;
} sTask;


/* The maximum number of tasks required at any one time
during the execution of the program
MUST BE ADJUSTED FOR EACH NEW PROJECT */
#define SCH_MAX_TASKS 8

#ifdef SCH_REPORT_ERRORS
/* The port on which error codes will be displayed
(ONLY USED IF ERRORS ARE REPORTED) */
#define Error_port PB
#endif

#define ERROR_SCH_TOO_MANY_TASKS 1
#define ERROR_SCH_CANNOT_DELETE_TASK 2
#define ERROR_SCH_WAITING_FOR_SLAVE_TO_ACK 3
#define ERROR_SCH_WAITING_FOR_START_COMMAND_FROM_MASTER 4
#define ERROR_SCH_ONE_OR_MORE_SLAVES_DID_NOT_START 5
#define ERROR_SCH_LOST_SLAVE 6
#define RETURN_ERROR 1
#define RETURN_NORMAL 0

extern void  SCH_Init(uint32_t intervall, TimerFormat_t format);
extern tByte SCH_Delete_Task(const tWord TASK_INDEX);
extern void  SCH_Start(void);
extern void  SCH_Dispatch_Tasks(void);
extern tByte SCH_Add_Task(void (* pFunction)(),const tWord DELAY,const tWord PERIOD);
extern void  SCH_Go_To_Sleep(void);
extern void  SCH_Report_Status(void);
