/*!=============================================================================
  ==============================================================================

  \ingroup SLros
  
  \file    SL_ros_servo_unix.c

  \author  Stefan Schaal
  \date    July, 2010

  ==============================================================================
  \remarks

  main program for the ros servo

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

#define TIME_OUT_NS  1000000

// global functions 
void ros_servo(void);

// local functions
static  int  checkForMessages(void);
static  void disable_ros_servo(void);
static  void drs(void);

// external functions

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
  int i, j;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  if (!init_ros_servo())
    return FALSE;

  // add to man pages 
  addToMan("drs","disables the ros servo",drs);

  // reset ros_servo variables
  servo_enabled          = 1;
  ros_servo_calls        = 0;
  ros_servo_time         = 0;
  ros_servo_errors       = 0;
  ros_servo_rate         = servo_base_rate/(double) ros_servo_ratio;

  // the user ROS as defined in initUserROS.c 
  initUserROS();

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop (this is the same as the task servo)
  while (servo_enabled) {

    // wait to take semaphore 
    switch (task_servo_ratio) {
    case R1TO1:
      if (semTake(sm_1to1_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO2:
      if (semTake(sm_1to2_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO3:
      if (semTake(sm_1to3_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO4:
      if (semTake(sm_1to4_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO5:
      if (semTake(sm_1to5_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R60HZ:
      if (semTake(sm_60Hz_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    default:
      if (semTake(sm_1to2_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");

    }

    // check for messages
    checkForMessages();

    // lock out the keyboard interaction 
    pthread_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_ros_servo())
      break;

    // continue keyboard interaction
    pthread_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("ROS Servo Error Count = %d\n",ros_servo_errors);

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
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_ros_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_ros_message->name[i]);

    // act according to the message name

  }

  // give back semaphore
  sm_ros_message->n_msgs = 0;
  sm_ros_message->n_bytes_used = 0;
  semGive(sm_ros_message_sem);


  return TRUE;
}

