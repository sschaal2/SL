/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL_servo.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  the main functions for the openGL_servo
  
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
#include "SL_openGL.h"
#include "SL_common.h"
#include "SL_objects.h"
#include "utility.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "SL_userGraphics.h"

//! time out for semaphores
#define TIME_OUT_NS 1000000000

// global variables
long    openGL_servo_calls=0;
int     openGL_servo_rate = R60HZ;
int     openGL_servo_errors=0;
double  openGL_servo_time=0;
double  servo_time=0;
int     servo_enabled;
int     stand_alone_flag = FALSE;
  
// local functions 
static void status(void);
static void togglePause(void);

  
/*!*****************************************************************************
*******************************************************************************
\note  init_openGL_servo
\date  July 1998
 
\remarks 
 
initialization of the servo
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 
******************************************************************************/
int 
init_openGL_servo(int argc, char** argv)

{
  int i,j,n;
  int rc;
  int ans; 

  // servo name
  sprintf(servo_name,"openGL");

  // add to man pages 
  addToMan("status","displays status information about servo",status);
  addToMan("p","toggles pausing of the simulation",togglePause);

  /* inverse dynamics */
  if (!init_dynamics()) 
    return FALSE;
  
  // initialize kinematics
  init_kinematics();
  
  // initialize graphics
  if (!initGraphics(&argc, &argv))
    return FALSE;
  
  // generic initialization program
  if (!initSimulationGraphics(argc, argv)) 
    return FALSE;

  // initialize user specific graphics
  if (!initUserGraphics()) 
    return FALSE;

  // start the main loop
  servo_enabled = TRUE;
  
  return TRUE;
}


/*!*****************************************************************************
*******************************************************************************
\note  receive_sim_state
\date  Nov. 2007
   
\remarks 

recieves the entire joint_sim_state from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_sim_state(void)
{
  
  int i;

  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i)
    sm_joint_sim_state_data[i] = sm_joint_sim_state->joint_sim_state[i];
  
  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,FLOAT2DOUBLE);
    
  semGive(sm_joint_sim_state_sem);

  // base state
  if (semTake(sm_base_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  sm_base_state_data[1] = sm_base_state->state[1];
  
  cSL_Cstate(&base_state-1, sm_base_state_data, 1, FLOAT2DOUBLE);

  semGive(sm_base_state_sem);


  // base orient
  if (semTake(sm_base_orient_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  sm_base_orient_data[1] = sm_base_orient->orient[1];
  
  cSL_quat(&base_orient-1, sm_base_orient_data, 1, FLOAT2DOUBLE);

  semGive(sm_base_orient_sem);


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  receive_misc_sensors
\date  Nov. 2007
   
\remarks 

receives the entire misc_sim_sensors from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_misc_sensors(void)
{
  
  int i;

  if (n_misc_sensors <= 0)
    return TRUE;

  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_misc_sensors; ++i)
    misc_sim_sensor[i] = sm_misc_sim_sensor->value[i];
  
  semGive(sm_misc_sim_sensor_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  receive_contacts
\date  Nov. 2007
   
\remarks 

receive the contact forces from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_contacts(void)
{
  
  int i,j;

  if (semTake(sm_contacts_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=0; i<=n_links; ++i) {
    contacts[i].status = sm_contacts->contact[i].status;
    for (j=1; j<=N_CART; ++j)
      contacts[i].f[j] = sm_contacts->contact[i].f[j];
  }
  
  semGive(sm_contacts_sem);

  return TRUE;
}


/*!*****************************************************************************
*******************************************************************************
\note  status
\date  Nov 2007
   
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
  printf("            Real Time              = %f\n",openGL_servo_time);
  printf("            Servo Calls            = %ld\n",openGL_servo_calls);
  printf("            Servo Rate             = %d\n",openGL_servo_rate);
  printf("            Servo Errors           = %d\n",openGL_servo_errors);
  printf("            CLMCplot Mode          = %d\n",clmcplot_mode);
  printf("            Playback Mode          = %d\n",playback_mode);
  printf("            Pause Flag             = %d\n",pause_flag);
  printf("            Window Update Rate     = %5.2f\n",window_update_rate);
  printf("            Stand Alone Flag       = %d\n",stand_alone_flag);
  printf("            User Graphics          = %s\n",user_display_function_name);
#ifdef __XENO__
  extern long count_xenomai_mode_switches;
  printf("            Xeonmai Mode Swiches   = %ld\n",count_xenomai_mode_switches);
#endif

  printf("\n");

}

/*!*****************************************************************************
*******************************************************************************
\note  togglePause
\date  Nov. 2005
   
\remarks 

pauses the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
togglePause(void) 
{
  if (pause_flag==0){
    pause_flag=1;
    printf("Pausing the simulation\n");
  }
  else  {
    pause_flag=0;
    printf("Resuming the simulation\n");
    semFlush(sm_simulation_servo_sem);
  }
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
int
checkForMessages(void)
{
  int i,j,k;
  char name[20];

  // check whether a message is available
  if (semTake(sm_openGL_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_openGL_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take openGL message semaphore\n");
    return FALSE;
  }

  for (k=1; k<=sm_openGL_message->n_msgs; ++k) {

    // get the name of this message
    strcpy(name,sm_openGL_message->name[k]);
    
    // act according to the message name
    if (strcmp(name,"followBase") == 0) {
      
      followBaseByName("1", TRUE);
      
    } else if (strcmp(name,"hideObject") == 0) {
      struct {
	int  hide;
	char obj_name[100];
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      changeHideObjByName(data.obj_name, data.hide);
      
    } else if (strcmp(name,"hideWindowByName") == 0) {
      char cbuf[101];
      
      memcpy(&cbuf,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(cbuf));
      printf("%s %d\n",cbuf,cbuf[100]);
      hideWindowByName(cbuf, (int)(cbuf[sizeof(cbuf)-1]));
      
    }

  }

  // give back semaphore
  sm_openGL_message->n_msgs = 0;
  sm_openGL_message->n_bytes_used = 0;
  semGive(sm_openGL_message_sem);


  return TRUE;
}

