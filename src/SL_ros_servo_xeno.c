/*!=============================================================================
  ==============================================================================

  \ingroup SLros
  
  \file    SL_ros_servo_xeno.c

  \author  Stefan Schaal
  \date    July 19, 2010

  ==============================================================================
  \remarks

  main program for the ros servo adjusted for xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_ros_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"

#define TIME_OUT_NS  1000000000

// local variables
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 10;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;
static int     delay_ns = FALSE;

// global functions 

// local functions
static  void ros_servo(void *dummy);

/*!*****************************************************************************
*******************************************************************************
\note  main
\date  July 2010
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

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  if (!init_ros_servo())
    return FALSE;

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // reset ros_servo variables
  servo_enabled          = 1;
  ros_servo_calls        = 0;
  last_ros_servo_calls   = 0;
  ros_servo_time         = 0;
  ros_servo_errors       = 0;
  ros_servo_rate         = servo_base_rate/(double) task_servo_ratio;

  changeCollectFreq(ros_servo_rate);
  
  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo",servo_name);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),ros_servo,NULL))) {
      printf("rt_task_spawn returned %d\n",rc);
    }

    // spawn command line interface thread
    spawnCommandLineThread(initial_user_command);

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
    ros_servo(NULL);

  }


  printf("ROS Servo Error Count = %d\n",task_servo_errors);

  return TRUE;

}
 
/*!*****************************************************************************
*******************************************************************************
\note  ros_servo
\date  July 2010
\remarks 
 
This program is clocked by the motor servo and uses a shared
memory semaphore for synchronization
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]  dummy: dummy argument
 
******************************************************************************/
static void
ros_servo(void *dummy) 

{
  int rc;

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL)))
    printf("rt_task_set_mode returned %d\n",rc);
  
  // run the servo loop
  while (servo_enabled) {

    // force delay ticks if user wishes
    if (delay_ns > 0)
      taskDelay(ns2ticks(delay_ns));

    // wait to take semaphore 
    if (semTake(sm_ros_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated\n");
      return FALSE;
    }

    // adjust the servo time
    ++ros_servo_calls;
    ros_servo_time = servo_time = ros_servo_calls/(double)ros_servo_rate;

    // lock out the keyboard interaction 
    pthread_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_ros_servo())
      break;

    // continue keyboard interaction
    pthread_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

}


/*!*****************************************************************************
*******************************************************************************
\note  dts & disable_task_servo
\date  April 1999
   
\remarks 

disables the task servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
dts(void)
{
  disable_task_servo();
}

void 
disable_task_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Task Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "task servo is not on!\n" );
  
}

/*!*****************************************************************************
*******************************************************************************
\note  checkForMessages
\date  Nov. 2007
   
\remarks 

Messages can be given to the servo for hard-coded tasks.This allows
some information passing between the different processes on variables
of common interest, e.g., the endeffector specs, object information,
etc.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static int
checkForMessages(void)
{
  int i,j;
  char name[20];

  // check whether a message is available
  if (semTake(sm_task_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_task_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_task_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_task_message->name[i]);

  }

  // give back semaphore
  sm_task_message->n_msgs = 0;
  sm_task_message->n_bytes_used = 0;
  semGive(sm_task_message_sem);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  drs & disable_ros_servo
\date  July 2010
   
\remarks 

disables the ros servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void 
drs(void)
{
  disable_ros_servo();
}

void 
disable_ros_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("ROS Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "ROS Servo is not on!\n" );
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  checkForMessages
\date  Nov. 2007
   
\remarks 

Messages can be given to the servo for hard-coded tasks.This allows
some information passing between the different processes on variables
of common interest, e.g., the endeffector specs, object information,
etc.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int
checkForMessages(void)
{
  int i,j;
  char name[20];

  // check whether a message is available
  if (semTake(sm_ros_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_ros_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++ros_servo_errors;
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_ros_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_ros_message->name[i]);

    // act according to the message name

    // ---------------------------------------------------------------------------
    if (strcmp(name,"status") == 0) { 

      status();

    }


  }

  // give back semaphore
  sm_ros_message->n_msgs = 0;
  sm_ros_message->n_bytes_used = 0;
  semGive(sm_ros_message_sem);


  return TRUE;
}

