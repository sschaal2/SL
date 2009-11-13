/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_motor_servo_xeno.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  main program for the motor servo adjusted for xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_motor_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_man.h"
#include "SL_controller.h"

// external variables
extern int           motor_servo_errors;
extern int           motor_servo_initialized;
extern int           count_no_receive;
extern double        count_no_receive_total;
extern int           count_no_broadcast;
extern int           no_receive_flag;

// global functions 
void disable_motor_servo(void);

// local functions
static void status(void);
static void sim_stop(void);
static void reset(void);
static void motor_servo(void *dummy);

// local variables
static RTIME   real_time;
static RTIME   real_time_dt;
static RTIME   start_real_time;
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 75;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;
static int     delay_ns = FALSE;

// external functions


/*!*****************************************************************************
*******************************************************************************
\note  main
\date  Feb 1999
\remarks 

initializes everything and starts the servo loop

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings

******************************************************************************/
int 
main(int argc, char**argv)
{
  int  i, j;
  int  rc;
  char name[100];

  // initialize xenomai specific variables and real-time environment
  initXeno();

  // parse command line options
  parseOptions(argc, argv);

  // get the clock option, i.e., the motor servo acts as a real-time clock
  real_time_clock_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-rtc")==0) {
      real_time_clock_flag = TRUE;
      break;
    }
  }

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_motor_servo();
  read_whichDOFs(config_files[WHICHDOFS],"motor_servo");

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // add to man pages 
  addToMan("dms","disables the motor servo",dms);
  addToMan("status","displays status information about servo",status);
  addToMan("stop","kills the robot control",sim_stop);

  // reset motor_servo variables
  servo_enabled            = 1;
  motor_servo_calls        = 0;
  servo_time               = 0;
  motor_servo_time         = 0;
  motor_servo_rate         = servo_base_rate;
  motor_servo_errors       = 0;
  count_no_receive         = 0;
  count_no_receive_total   = 0;
  count_no_broadcast       = 0;
  
  changeCollectFreq(motor_servo_rate);
  if (real_time_clock_flag) {
    addVarToCollect((char *)&(real_time),"real_time","ns", INT,FALSE);
    addVarToCollect((char *)&(real_time_dt),"real_time_dt","ns", INT,FALSE);
  }
  updateDataCollectScript();
  setDefaultPosture();
  zero_integrator();
  
  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo",servo_name);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),motor_servo,NULL))) {
      printf("rt_task_spawn returned %d\n",rc);
    }

    // spawn command line interface thread
    spawnCommandLineThread(NULL);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // wait for the task to finish
    rt_task_join(&servo_ptr);
	
  } else {

    // spawn command line interface thread
    spawnCommandLineThread(NULL);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // run this servo
    motor_servo(NULL);

  }

  printf("Motor Servo Error Count = %d\n",motor_servo_errors);

  return TRUE;

}
 
/*!*****************************************************************************
 *******************************************************************************
 \note  motor_servo
 \date  Oct 2009
 \remarks 
 
 This program is clocked either directly by a clock, or by another
  process through a shared memory semaphore for synchronization
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  dummy: dummy argument
 
 ******************************************************************************/
static void
motor_servo(void *dummy) 

{
  RTIME last_real_time, current_real_time;
  unsigned long overruns;
  int rc;

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL))) 
    printf("rt_task_set_mode returned %d\n",rc);

  
  if (real_time_clock_flag) // make this a clocked task
    rt_task_set_periodic(NULL,TM_NOW,(int)(1000000000./(double)motor_servo_rate));
      
  // initialize time
  last_real_time   = rt_timer_read();
  start_real_time  = last_real_time;

  // run the servo loop
  while (servo_enabled) {

    // force delay ticks if user wishes
    if (delay_ns > 0)
      taskDelay(ns2ticks(delay_ns));

    if (real_time_clock_flag) { // motor_servo acts as real time clock
      rt_task_wait_period(&overruns);

      current_real_time = rt_timer_read();

      real_time      = current_real_time  - start_real_time;
      real_time_dt   = current_real_time  - last_real_time;
      last_real_time = current_real_time;

      motor_servo_errors += overruns;
	
      if (semGive(sm_motor_servo_sem) == ERROR)
	exit(-1);

    } 

    // wait to take semaphore
    if (semTake(sm_motor_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated");
      exit(-1);
    }
    
    // increment time
    ++motor_servo_calls;
    servo_time += 1./(double)motor_servo_rate;
    motor_servo_time = servo_time;

    // lock out the keyboard interaction 
    pthread_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_motor_servo())
      break;

    // continue keyboard interaction
    pthread_mutex_unlock( &mutex1 );

    // trigger the simulation servo
    semGive(sm_simulation_servo_sem);

  }  /* end servo while loop */


}

/*!*****************************************************************************
*******************************************************************************
\note  dms & disable_motor_servo
\date  December 1997
   
\remarks 

disables the motor servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     param   :

******************************************************************************/
void 
dms(void)
{
  disable_motor_servo();
}

void 
disable_motor_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Motor Servo Terminated\n");

    exit(-1);

  } else
    fprintf( stderr, "motor servo is not on!\n" );
}


/*!*****************************************************************************
*******************************************************************************
\note  status
\date  August 7, 1992
   
\remarks 

prints out all important variables

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
status(void)
{

  printf("\n");
  printf("            Controller Kind        = %d\n",controller_kind);
  printf("            Time                   = %f\n",servo_time);
  printf("            Servo Calls            = %ld\n",motor_servo_calls);
  printf("            Servo Rate             = %d\n",motor_servo_rate);
  printf("            Servo Errors           = %d\n",motor_servo_errors);
  printf("            Count No Receive       = %d\n",(int)count_no_receive_total);
  printf("            Count No Broadcast     = %d\n",count_no_broadcast);
  printf("            Servo Initialize       = %d\n",motor_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("            Power Status           = %d\n",power_on);
  printf("            Real Time Clock        = %d\n",real_time_clock_flag);
#ifdef __XENO__
  extern long count_xenomai_mode_switches;
  printf("            Xeonmai Mode Swiches   = %ld\n",count_xenomai_mode_switches);
#endif

  printf("\n");

}

/*!*****************************************************************************
*******************************************************************************
\note  stop
\date  August 7, 1992 
   
\remarks 

send pump into low pressure and terminate loops

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
sim_stop(void)
{
  stop("Simulation Triggered Stop");   /* for simulation environment only */
}
int
stop(char *msg)
{

  int i;

  beep(1);
  printf("%s\n",msg);
  fflush(stdout);
  if (real_robot_flag)
    dms();
  else
    reset();
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  reset
 \date  Nov. 2005
 
 \remarks 
 
 sends message to task_servo to reset
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
******************************************************************************/
static void 
reset(void) 
{
  int i,j;
  unsigned char buf[1];

  sendMessageTaskServo("reset",(void *)buf,0);
  setDefaultPosture();

}


