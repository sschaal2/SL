/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo_unix.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks

  Initialization and semaphore communication for the vision servo
  under unix

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_vision_servo.h"

#define TIME_OUT_NS  1000000

/* global variables */
extern int stereo_mode;

/* local variables */

/* global functions */
void status(void);
int  stop(char *msg);

/* local functions */

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
  int i, j;

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

  // initialize all vision variables to safe values
  init_vision_states();
  init_learning();

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    // wait to take semaphore 
    if (semTake(sm_60Hz_sem,WAIT_FOREVER) == ERROR)
      stop("semTake Time Out -- Servo Terminated");
   
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

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

  return TRUE;

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

