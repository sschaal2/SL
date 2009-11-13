/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo_xeno.c

  \author  Stefan Schaal
  \date    Oct 2009

  ==============================================================================
  \remarks

  main program for the task servo adjusted for xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "SL_common.h"
#include "SL_task_servo.h"
#include "SL_tasks.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"

#define TIME_OUT_NS  1000000000

// global variabes
char initial_user_command[100]="";

// local variables
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 25;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;
static int     delay_ns = FALSE;

// global functions 
void disable_task_servo(void);

// local functions
static  int  checkForMessages(void);
static  void freezeBaseToggle(void);
static  void task_servo(void *dummy);



// external functions
extern void initUserTasks(void);

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
  init_task_servo();
  read_whichDOFs(config_files[WHICHDOFS],"task_servo");

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // generic computations
  init_generic_tasks();

  // add to man pages 
  addToMan("dts","disables the task servo",dts);
  if (!real_robot_flag) {
    addToMan("reset","reset state of simulation",reset);
    addToMan("setG","set gravity constant",setG);
    addToMan("freezeBase","freeze the base at orgin",freezeBaseToggle);
  }

  // reset task_servo variables
  servo_enabled           = 1;
  task_servo_calls        = 0;
  task_servo_time         = 0;
  task_servo_errors       = 0;
  task_servo_rate         = servo_base_rate/(double) task_servo_ratio;

  setTaskByName(NO_TASK);
  changeCollectFreq(task_servo_rate);
  
  // the user tasks as defined in initUserTasks.c 
  initUserTasks();

  // reset the simulation
  if (!real_robot_flag)
    reset();

  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo",servo_name);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),task_servo,NULL))) {
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
    spawnCommandLineThread(initial_user_command);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // run this servo
    task_servo(NULL);

  }


  printf("Task Servo Error Count = %d\n",task_servo_errors);

  return TRUE;

}
 
/*!*****************************************************************************
*******************************************************************************
\note  task_servo
\date  Oct 2009
\remarks 
 
This program is clocked by the motor servo and uses a shared
memory semaphore for synchronization
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]  dummy: dummary argument
 
******************************************************************************/
static void
task_servo(void *dummy) 

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
    if (!run_task_servo())
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
\note  sendUserGraphics
\date  Nov. 2007
   
\remarks 

sends out information for user graphics to shared memory

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name   : name of graphics
\param[in]     buf    : byte buffer with information
\param[in]     n_bytes: number of bytest in buffer

******************************************************************************/
int
sendUserGraphics(char *name, void *buf, int n_bytes)
{
  int i,j;
  
  // send the user graphics data
  if (semTake(sm_user_graphics_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take user_graphics semaphore\n");
    return FALSE;
  }

  // send the name of user graphics
  strcpy(sm_user_graphics->name,name);

  // send the data
  memcpy(sm_user_graphics->buf,buf,n_bytes);

  // give semaphores
  semGive(sm_user_graphics_sem);
  semGive(sm_user_graphics_ready_sem);
  
  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  reset
\date  Nov. 2005
   
\remarks 

sends message to simulation_servo to reset

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
reset(void) 
{
  int i,j;
  float buf[N_CART+N_QUAT+1];
  unsigned char cbuf[(N_CART+N_QUAT)*sizeof(float)];

  j = 0;
  for (i=1; i<=N_CART; ++i)
    buf[++j] = freeze_base_pos[i];
    
  for (i=1; i<=N_QUAT; ++i)
    buf[++j] = freeze_base_quat[i];

  memcpy(cbuf,(void *)&(buf[1]),(N_CART+N_QUAT)*sizeof(float));
    
  sendMessageSimulationServo("reset",(void *)cbuf,(N_CART+N_QUAT)*sizeof(float));
  freeze();
  setDefaultPosture();

}


/*!*****************************************************************************
*******************************************************************************
\note  followBase
\date  Nov. 2005
   
\remarks 

sends message to openGL_servo to follow the base coordinates

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
followBase(void) 
{
  int i,j;
  unsigned char buf[1];

  sendMessageOpenGLServo("followBase",(void *)buf,0);
}


/*!*****************************************************************************
*******************************************************************************
\note  changeHideObject
\date  Nov. 2005
   
\remarks 

allows to hide an object from the openGL display

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name object
\param[in]     hide : TRUE/FALSE

******************************************************************************/
void 
changeHideObject(char *name, int hide) 
{
  int i,j;
  struct {
    int  hide;
    char obj_name[100];
  } data;
  unsigned char buf[sizeof(data)];

  data.hide = hide;
  strcpy(data.obj_name,name);

  memcpy(buf,&data,sizeof(data));
  sendMessageOpenGLServo("hideObject",(void *)buf,sizeof(data));
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

    // act according to the message name
    if (strcmp(name,"reset") == 0) {
      reset();
    }

  }

  // give back semaphore
  sm_task_message->n_msgs = 0;
  sm_task_message->n_bytes_used = 0;
  semGive(sm_task_message_sem);


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  sendUextSim
\date  Nov. 2005
   
\remarks 

sends simulated external force to simulation servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
sendUextSim(void) 
{
  int i,j;
  int count = 0;
  float buf[2*n_dofs*N_CART+1];
  unsigned char cbuf[(2*n_dofs*N_CART)*sizeof(float)];

  for (i=1; i<=n_dofs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      buf[++count] = uext_sim[i].f[j];
      buf[++count] = uext_sim[i].t[j];
    }
  }

  memcpy(cbuf,(void *)&(buf[1]),(2*n_dofs*N_CART)*sizeof(float));
    
  sendMessageSimulationServo("setUextSim",(void *)cbuf,(2*n_dofs*N_CART)*sizeof(float));

}


/*!*****************************************************************************
*******************************************************************************
\note  changeRealTime
\date  May 2008
   
\remarks 

sends messages to simulation servo to turn on/off real-time
simualtion

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     flag : TRUE/FALSE real time flag

******************************************************************************/
void 
changeRealTime(int flag) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = flag;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));
    
  sendMessageSimulationServo("changeRealTime",(void *)cbuf,sizeof(float));

}


/*!*****************************************************************************
*******************************************************************************
\note  setG
\date  Nov. 2005
\remarks 

set the gravity constant to an arbitrary value

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
setG(void)
{
  if (get_double("Enter new gravity constant",gravity,&gravity))
    setGravity(gravity);
}

/*!*****************************************************************************
*******************************************************************************
\note  setGravity
\date  July 2009
   
\remarks 

sends messages to simulation servo to adjust gravity constant

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     grav : gravity

******************************************************************************/
void 
setGravity(double grav) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = grav;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));

  sendMessageSimulationServo("setG",(void *)cbuf,sizeof(float));

}

/*!*****************************************************************************
*******************************************************************************
\note  hideWindowByName
\date  March 2003
   
\remarks 

allows toggling the hide status of a particular window

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name  : string containing the name of the window
\param[in]     hide  : TRUE/FALSE = hide or not

******************************************************************************/
void
hideWindowByName(char *name, int hide)
{
  int i;
  unsigned char cbuf[101];

  if (strlen(name) > sizeof(cbuf)-1) {
    printf("Too long name in hideWindowByName\n");
    return;
  }

  sprintf((char *)cbuf,"%s",name);
  cbuf[sizeof(cbuf)-1] = hide;

  printf("%s %d\n",cbuf,cbuf[100]);

  sendMessageOpenGLServo("hideWindowByName",(void *)cbuf,sizeof(cbuf));

}

/*!*****************************************************************************
*******************************************************************************
\note  freezeBase
\date  May 2008
   
\remarks 

sets the freezeBase flag in the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     flag : TRUE/FALSE for freeze base

******************************************************************************/
static void 
freezeBaseToggle(void) 
{

  if (freeze_base == 0) {
    freeze_base = TRUE;
    printf("Base is fixed at origin\n");
  } else {
    freeze_base = FALSE;
    printf("Base is floating\n");
  }

  freezeBase(freeze_base);
  
}

void 
freezeBase(int flag) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = freeze_base = flag;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));
    
  sendMessageSimulationServo("freezeBase",(void *)cbuf,sizeof(float));

}


