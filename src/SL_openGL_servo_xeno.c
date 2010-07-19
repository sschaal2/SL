/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL_servo_xeno.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  the main program for graphics visualization
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// openGL headers
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

// local headers
#include "SL.h"
#include "SL_common.h"
#include "SL_openGL_servo.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"

// global variables

// local variables
static RT_TASK servo_ptr;
static int     use_spawn = FALSE; // somehow openGL does not work in a spawned process
static int     servo_priority = 10;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;
static int     delay_ns=FALSE;
  
// local functions 
static void openGL_servo(void *dummy);
static  void dos(void);
static  void disable_openGL_servo(void);

  
/*!*****************************************************************************
*******************************************************************************
\note  main
\date  July 1998
 
\remarks 

 
entry program: the structure of the entire program is such that we assume 
some initialization functions, and then a final function that continuously
runs the simulation in a "clocked" fashion.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 
******************************************************************************/
int 
main(int argc, char**argv)

{
  int  i,j,n;
  int  rc;
  int  ans; 
  char name[100];


  // initialize xenomai specific variables and real-time environment
  initXeno();

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // get the stand alone flag
  stand_alone_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-sta")==0) {
      stand_alone_flag = TRUE;
      break;
    }
  }

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  if (!init_openGL_servo(argc,argv))
    return FALSE;

  // add to man pages 
  addToMan("dos","disables the openGL servo",dos);

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // make this process real-time
  if (use_spawn) { 

    sprintf(name,"%s_servo",servo_name);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),openGL_servo,NULL))) {
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
    openGL_servo(NULL);

  }

  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
 \note  openGL_servo
 \date  Oct 2009
 \remarks 
 
 This program is clocked by the motor servo and uses a shared
 memory semaphore for synchronization
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  dummy: dummary argument
 
 ******************************************************************************/
static void
openGL_servo(void *dummy) 

{
  int rc;

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL))) 
      printf("rt_task_set_mode returned %d\n",rc);

  // start the main loop
  servo_enabled = TRUE;
  glutMainLoop();

}

/*!*****************************************************************************
 *******************************************************************************
\note  dos & disable_openGL_servo
\date  July 2010
   
\remarks 

disables the openGL servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none
     
 ******************************************************************************/
void 
dos(void)
{
  disable_openGL_servo();
}

void 
disable_openGL_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("OpenGL Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "OpenGL Servo is not on!\n" );
  
}
