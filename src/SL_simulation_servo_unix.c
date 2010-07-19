/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_simulation_servo_unix.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  main program for running a numerical simulation of a robot, i.e.,
  this is where the equations of motion are integrated.

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_integrate.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_simulation_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_man.h"

#define TIME_OUT_NS  1000000

// global variables

// local variables
static int pause_flag = FALSE;

// global functions 

// local functions
static void dss(void);
static void disable_simulation_servo(void);

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
  int i, j;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initalize the servo
  if (!init_simulation_servo())
    return FALSE;

  // add to man pages 
  addToMan("dss","disables the simulation servo",dss);

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // boardcast the current state such that the motor servo can generate a command
  send_sim_state();
  send_misc_sensors();
  send_contacts();
  semGive(sm_motor_servo_sem);  

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);
  
  // run the servo loop
  while (servo_enabled) {

    // check for PAUSE
    if (semTake(sm_pause_sem,NO_WAIT) != ERROR) {
      if (pause_flag)
	pause_flag = FALSE;
      else
	pause_flag = TRUE;
    }

    if (pause_flag) {
      usleep(10000);
      continue;
    }

    // wait to take semaphore 
    if (semTake(sm_simulation_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated\n");
      return FALSE;
    }

    // advance the simulation servo
    ++simulation_servo_calls;
    servo_time += 1./(double)simulation_servo_rate;
    simulation_servo_time = servo_time;

    // lock out the keyboard interaction 
    pthread_mutex_lock( &mutex1 );

    // run the simulation servo routines
    if (!run_simulation_servo())
      break;

    // continue keyboard interaction
    pthread_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  return TRUE;

}
 
/*!*****************************************************************************
*******************************************************************************
\note  receive_des_commands
\date  Nov. 2007
   
\remarks 

reads the commands from the joint_sim_state shared memory
structure
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_des_commands(void)
{
  
  int i;
  extern double *controller_gain_th;
  extern double *controller_gain_thd;


  if (semTake(sm_des_commands_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i) {
    joint_sim_state[i].u   = (double) sm_des_commands->des_command[i].u - 
      (double) sm_des_commands->des_command[i].upd;
    joint_des_state[i].th  = (double) sm_des_commands->des_command[i].th;
    joint_des_state[i].thd = (double) sm_des_commands->des_command[i].thd;
  }
  
  // re-generate and add the PD command from the motor servo, as the
  // delay between motor servo and simulation servo can cause instabilities 
  // in case of stiff contacts -- the reason is that the state info on the 
  // motor servo is one tick delayed; the old PD command has been subtracted
  // from the total command above
  
  for (i=1; i<=n_dofs; ++i) {
    if (sm_des_commands->des_command[i].upd != 0.0) {
      joint_sim_state[i].u += (joint_des_state[i].th - joint_sim_state[i].th) * 
	controller_gain_th[i];
      joint_sim_state[i].u += (joint_des_state[i].thd - joint_sim_state[i].thd) * 
	controller_gain_thd[i];
    }

    if (fabs(joint_sim_state[i].u) > u_max[i]) {
      joint_sim_state[i].u = macro_sign(joint_sim_state[i].u) * u_max[i];
    }

  }
  
  semGive(sm_des_commands_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_sim_state
\date  Nov. 2007
   
\remarks 

sends the entire joint_sim_state to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_sim_state(void)
{
  
  int i;

  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,DOUBLE2FLOAT);
    
  for (i=1; i<=n_dofs; ++i)
    sm_joint_sim_state->joint_sim_state[i] = sm_joint_sim_state_data[i];
  
  semGive(sm_joint_sim_state_sem);


  // base state
  if (semTake(sm_base_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);

  sm_base_state->state[1] = sm_base_state_data[1];
  
  semGive(sm_base_state_sem);


  // base orient
  if (semTake(sm_base_orient_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_quat(&base_orient-1, sm_base_orient_data, 1, DOUBLE2FLOAT);

  sm_base_orient->orient[1] = sm_base_orient_data[1];
  
  semGive(sm_base_orient_sem);


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_misc_sensors
\date  Nov. 2007
   
\remarks 

sends the entire misc_sim_sensors to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_misc_sensors(void)
{
  
  int i;

  if (n_misc_sensors <= 0)
    return TRUE;

  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_misc_sensors; ++i)
    sm_misc_sim_sensor->value[i] = misc_sim_sensor[i];
  
  semGive(sm_misc_sim_sensor_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_contacts
\date  Nov. 2007
   
\remarks 

sends the contact forces to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_contacts(void)
{
  
  int i,j;

  if (semTake(sm_contacts_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=0; i<=n_links; ++i) {
    sm_contacts->contact[i].status = contacts[i].status;
    for (j=1; j<=N_CART; ++j)
      sm_contacts->contact[i].f[j] = contacts[i].f[j];
  }
  
  semGive(sm_contacts_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  dss & disable_simulation_servo
\date  July 2010
   
\remarks 

disables the simulation servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none
     
 ******************************************************************************/
static void 
dss(void)
{
  disable_simulation_servo();
}

static void 
disable_simulation_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Simulation Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "Simulation Servo is not on!\n" );
  
}


















