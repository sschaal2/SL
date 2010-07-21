/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo_xeno.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks

  Initialization and semaphore communication for the vision servo
  under xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_vision_servo.h"

/* global variables */
extern int stereo_mode;

/* local variables */
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 10;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;
static int     delay_ns = FALSE;

/* global functions */
void status(void);
int  stop(char *msg);

/* local functions */
static void vision_servo(void *dummy);
static int  checkForMessages(void);


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

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_vision_servo();

  // add to man pages 
  addToMan("dvs","disables the vision servo",dvs);
  addToMan("status","displays status information about servo",status);

  // setup servo
  servo_enabled           = 1;
  vision_servo_calls      = 0;
  vision_servo_time       = 0;
  servo_time              = 0;
  vision_servo_errors     = 0;
  vision_servo_rate       = VISION_SERVO_RATE;
  no_hardware_flag        = TRUE;
  changeCollectFreq(vision_servo_rate);

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // initialize all vision variables to safe values
  init_vision_states();
  init_learning();

  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo",servo_name);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),vision_servo,NULL))) {
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
    vision_servo(NULL);

  }

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vision_servo
 \date  Oct 2009
 \remarks 
 
 This program is clocked by the motor servo and uses a shared
 memory semaphore for synchronization
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  dummy: dummary argument
 
 ******************************************************************************/
static void
vision_servo(void *dummy) 
{

  int i;
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
    if (semTake(sm_vision_servo_sem,WAIT_FOREVER) == ERROR)
      stop("semTake Time Out -- Servo Terminated");
   
    // check for messages
    checkForMessages();

    // lock out the keyboard interaction 
    pthread_mutex_lock( &mutex1 );

    // reset the blob status
    for (i=1; i<=max_blobs; ++i) {
      raw_blobs2D[i][1].status = FALSE;
      raw_blobs2D[i][2].status = FALSE;
    }

    // run the task servo routines
    if (!run_vision_servo())
      break;

    // continue keyboard interaction
    pthread_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

}

/*!*****************************************************************************
*******************************************************************************
\note  dvs & disable_vision_servo
\date  April 1999
   
\remarks 

disables the vision servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
dvs(void)
{
  disable_vision_servo();
}

void 
disable_vision_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Vision Servo Terminated\n");

    exit(1);
    
  } else
    fprintf( stderr, "vision servo is not on!\n" );
  
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
void
status(void)
{

  printf("\n");
  printf("            Time                   = %f\n",vision_servo_time);
  printf("            Servo Calls            = %d\n",vision_servo_calls);
  printf("            Servo Rate             = %d\n",vision_servo_rate);
  printf("            Servo Errors           = %d\n",vision_servo_errors);
  printf("            Servo Initialize       = %d\n",vision_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("           #frames read            = %d\n",count_all_frames);
  printf("           #frames lost            = %d\n",count_lost_frames);
  printf("            vision_pp              = %s\n",current_pp_name);
  printf("            No Hardware Flag       = %d\n",no_hardware_flag);
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

stops ongoing processing on this servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
stop(char *msg)
{

  int i;

  dvs();
  beep(1);
  printf("%s\n",msg);
  
  return TRUE;

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
  if (semTake(sm_vision_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;


  // receive the message
  if (semTake(sm_vision_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++vision_servo_errors;
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_vision_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_vision_message->name[i]);

    // act according to the message name

    // ---------------------------------------------------------------------------
    if (strcmp(name,"status") == 0) { 

      status();

    }


  }

  // give back semaphore
  sm_vision_message->n_msgs = 0;
  sm_vision_message->n_bytes_used = 0;
  semGive(sm_vision_message_sem);


  return TRUE;
}
