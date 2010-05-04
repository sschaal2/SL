/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_motor_servo.c

  \author  Stefan Schaal
  \date    1997

  ==============================================================================
  \remarks

  low level motor control for the robots and simulations

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_motor_servo.h"
#include "SL_collect_data.h"
#include "SL_controller.h"
#include "SL_shared_memory.h"
#include "SL_sensor_proc.h"
#include "SL_common.h"
#include "SL_filters.h"
#include "SL_oscilloscope.h"
#include "utility.h"


#define COUNT_NO_RECEIVE_MAX 30
#define TIME_OUT_NS          1000  //!< time out in nano seconds
#define TRANSIENT_TICKS 10         //!< servo ticks until filter convergence

/* variables for the motor servo */
int           motor_servo_errors;
long          motor_servo_calls=0;
int           motor_servo_initialized = FALSE;
int           motor_servo_rate;
double        servo_time=0;
double        motor_servo_time=0;
int           servo_enabled;
int           count_no_receive = 0;
double        count_no_receive_total = 0;
int           count_no_broadcast = 0;
int           no_receive_flag = TRUE;
int          *zero_ufb_P_flag;
int          *zero_ufb_D_flag;
int           real_time_clock_flag  = FALSE;


/* local variables */
static int       *joint_invalid;
static SL_Jstate *store_broadcast_joint_state; // stored state to detect canceling

/* global functions */
int  run_motor_servo(void);

/* local functions */
static int  receive_commands(void);
static int  broadcast_sensors(void);
static void triggerSynchronization(void);
static int  checkForMessages(void);


/*!*****************************************************************************
 *******************************************************************************
  \note  init_motor_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
void
init_motor_servo(void)
{
  int j, i;
  STATUS error;
  char   string[100];
  extern int    init_user_commands(void);
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    joint_invalid               = my_ivector(1,n_dofs);
    store_broadcast_joint_state = (SL_Jstate *) 
      my_calloc((unsigned long)(n_dofs+1),sizeof(SL_Jstate),MY_STOP);
    zero_ufb_P_flag             = my_ivector(1,n_dofs);
    zero_ufb_D_flag             = my_ivector(1,n_dofs);
  }
  
  if (motor_servo_initialized) {
    printf("Motor Servo is already initialized\n");
    return;
  }

  /* the default servo rate  and the name of the servo*/
  sprintf(servo_name,"motor");
  motor_servo_rate=servo_base_rate;
  
  /* vxworks specific initialization */
  
#ifdef VX
  printf("Init vxworks ...");
  if (!init_vxworks()) 
    return;
  printf("done\n");
#endif
  
  /* set oscilloscope to start value */
  setOsc(d2a_cm,0.0);

  /* initialize the controller */
  printf("Init controller ...");
  if (!init_controller())
    return;
  printf("done\n");

  /* init data collection */
  initCollectData(motor_servo_rate);

  /* initialize the sensory processing */
  printf("Init sensor processing ...");
  if (!init_sensor_processing())
    return;
  printf("done\n");

  /* initialize shared memories and shared semaphores */
  printf("Init shard memory ...");
  if (!init_shared_memory())
    return;
  printf("done\n");

   /* initialize system commands and user commands */
  printf("Init commands ...");
  if (!init_commands())
    return;
    
  if (!init_user_commands())
    return;

  printf("done\n");

  /* add variables to data collection */

  for (i=1; i<=n_dofs; ++i) {
    printf("%d...",i);
    fflush(stdout);
    sprintf(string,"%s_th",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].th),string,"rad", DOUBLE,FALSE);
    sprintf(string,"%s_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thd),string,"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thdd),string,"rad/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_u",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].u),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].load),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_des_th",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].th),string,"rad",DOUBLE,FALSE);
    sprintf(string,"%s_des_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thd),
		    string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_uff",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].uff),string,"Nm",DOUBLE,FALSE);
  }

  addVarToCollect((char *)&(motor_servo_errors),"MSErrors","-",INT,FALSE);
  addVarToCollect((char *)&(count_no_receive),"MSNoReceive","-",INT,FALSE);

  updateDataCollectScript();

  printf("done\n");

  /* add to man pages */

  /* initialize user specific things */
  if (!init_user_motor())
    return;

  /* if all worked out, we mark the servo as ready to go */
  motor_servo_initialized = TRUE;

  scd();

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_motor_servo
\date  Feb 1999
\remarks 

        set of subroutines executed in the motor servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
run_motor_servo(void)

{
  int    j,i;
  static int    last_tick=0;
  double dt;

  
  setOsc(d2a_cm,100.0);
  
  /* check for missed ISR ticks */
  
  dt = (double)(motor_servo_calls - last_tick)/(double)motor_servo_rate;
  if (dt == 0) /* this can only happen if weirdly disturbed */
    return FALSE;
  
  if (motor_servo_calls - last_tick > 1) {
    motor_servo_errors += motor_servo_calls - last_tick + 1;
  }
  last_tick = motor_servo_calls;
  
  /*********************************************************************
   * check for messages
   */

  checkForMessages();

  /*********************************************************************
   * read data from the robot: this returns all sensors in SI units
   */

  if (!read_sensors()) {
    stop("Problem when reading sensors"); 
    return FALSE;
    ;
  }

  setOsc(d2a_cm,90.0);
  
  /*********************************************************************
   * filtering and differentiation of the data
   */

  if (!process_sensors()) {
    stop("Problem when processing sensors");
    return FALSE;
  }

  setOsc(d2a_cm,80.0);
  
  /*************************************************************************
   * provide sensor readings in shared memory
   */

  if (!broadcast_sensors()) {
    stop("Problem when broadcasting sensor readings");
    return FALSE;
  }
  
  
  setOsc(d2a_cm,70.0);
  
  /*************************************************************************
   *  trigger synchronization processes
   */
  
  if (motor_servo_calls > TRANSIENT_TICKS) { // needed for filters to converge
    triggerSynchronization();
  }   
  
  setOsc(d2a_cm,50.0);
  
  /**********************************************************************
   * get desired values and feedforward commands
   */

  if (motor_servo_calls > TRANSIENT_TICKS) {
    if (!receive_commands()) {
      stop("Problem when receiving commands");
      return FALSE;
    }
  } else {
    for (i=1; i<=n_dofs; ++i) {
      joint_des_state[i].th  = joint_state[i].th;
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].uff = 0.0;
    }
  }

  setOsc(d2a_cm,40.0);
  
  /**********************************************************************
   * the feedforward/feedback controller
   */
  
  if (!generate_total_commands()) {
    stop("Problem when generating total command");
    return FALSE;
  }
  
  setOsc(d2a_cm,25.0);
  
  /**********************************************************************
   *  send commands to the robot
   */

  if (!send_commands()) {
    stop("Problem when sending command");
    return FALSE;
  }
  
  setOsc(d2a_cm,10.0);
  
  /*************************************************************************
   * collect data
   */

  writeToBuffer();

  setOsc(d2a_cm,0.0);
  
  /*************************************************************************
   * end of functions
   */

  return TRUE;

}
/*!*****************************************************************************
 *******************************************************************************
\note  receive_commands
\date  April 1999
   
\remarks 

        checks for new command input, copies those to local a structure,
	and counts the frequency of incoming commands
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_commands(void)
{
  
  int i;
  double rate = 0.999;
  int  wait_flag;

  if (real_time_clock_flag ) {
    wait_flag = NO_WAIT;
  } else {
    if (count_no_receive < task_servo_ratio-1) // force tight syncornization
      wait_flag = NO_WAIT;
    else
      wait_flag = WAIT_FOREVER;
  }

  if (semTake(sm_sjoint_des_state_ready_sem,wait_flag) == ERROR) {

    if (wait_flag == WAIT_FOREVER) // an error win WAIT_FOREVER must be terminated
      exit(-1);

    if (++count_no_receive < task_servo_ratio) {
      // if the motor servo run higher rate than the task servo, the desired
      // joint states gets integrated to be more accurate
      for (i=1; i<=n_dofs; ++i) {
	joint_des_state[i].th  += joint_des_state[i].thd  * 1./(double)motor_servo_rate;
	joint_des_state[i].thd += joint_des_state[i].thdd * 1./(double)motor_servo_rate;
      }
    }
    count_no_receive_total += 1./(double)task_servo_ratio;

  } else {

    if (semTake(sm_sjoint_des_state_sem,NO_WAIT) == ERROR) {
      
      ++count_no_receive;
      count_no_receive_total += 1.0;

    } else {

      count_no_receive = 0;
      count_no_receive_total = (int) count_no_receive_total;

      memcpy((void *)(&sm_sjoint_des_state_data[1]),
	     (const void*)(&sm_sjoint_des_state->sjoint_des_state[1]),
	     sizeof(SL_fSDJstate)*n_dofs);

      for (i=1; i<=n_dofs; ++i) {
	// check the joint status and only copy data if TRUE
	sm_sjoint_des_state->sjoint_des_state[i].status = FALSE;
	if (sm_sjoint_des_state_data[i].status) {
	  cSL_SDJstate(&(joint_des_state[i])-1,
		       &(sm_sjoint_des_state_data[i])-1,1,FLOAT2DOUBLE);
	}
	// check whether the user wants to overwrite local feedback servo
	if (joint_des_state[i].th == store_broadcast_joint_state[i].th)
	  zero_ufb_P_flag[i] = task_servo_ratio;
	if (joint_des_state[i].thd == store_broadcast_joint_state[i].thd)
	  zero_ufb_D_flag[i] = task_servo_ratio;
      }

      semGive(sm_sjoint_des_state_sem);

    }

  }

  /* check status of joints */
  for (i=1; i<=n_dofs; ++i) {

    /* if we do not receive enough input, we go into a default mode */
    if (!sm_sjoint_des_state_data[i].status) {
      if (++joint_invalid[i] > COUNT_NO_RECEIVE_MAX) {
	if (fabs(joint_des_state[i].th-joint_default_state[i].th) > 0.001) 
	  joint_des_state[i].th = joint_des_state[i].th * rate + 
	    (1.-rate) * joint_default_state[i].th;
	joint_des_state[i].thd = 0;
	joint_des_state[i].uff = 0;
      }
    } else {
      joint_invalid[i] = 0;
      sm_sjoint_des_state_data[i].status = FALSE;
    }

  }

  /* if we receive not information at all show it on the oscilloscope */

  if (count_no_receive > COUNT_NO_RECEIVE_MAX) {

    no_receive_flag = TRUE;

    setOsc(d2a_cm,10.0);

    /* after 0.5 hours, we automatically shut down the pump */

    if ((double)count_no_receive/(double)motor_servo_rate > 30.0*60.0) {
#ifdef VX
      user_kill();
#else
      exit(-1);
#endif
    }

  } else {

    no_receive_flag = FALSE;

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  broadcast_sensors
\date  April 1999
   
\remarks 

        just copies the sensor data to the share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
broadcast_sensors(void)
{
  
  int i,j;

  /* only write at the task servo rate to reduce VME traffic */
  if (motor_servo_calls%task_servo_ratio!=0)
    return TRUE;

  if (semTake(sm_joint_state_sem,NO_WAIT) == ERROR) {
    
    ++count_no_broadcast;
    
  } else {
    
    cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,DOUBLE2FLOAT);

    memcpy((void *)(&sm_joint_state->joint_state[1]),
	   (const void*)(&sm_joint_state_data[1]),
	   sizeof(SL_fJstate)*n_dofs);

    semGive(sm_joint_state_sem);

    // store the state that was currently boradcast in order to detect
    // if the user returns in a desired state exactly the same state
    // as desired signal -- this means the user wants to overwrite the
    // PD servo on the motor servo with his/her own signals -- note 
    // that the float2double converion is important as this clipping
    // will be done to the received state as well.

    cSL_Jstate(store_broadcast_joint_state,sm_joint_state_data,n_dofs,FLOAT2DOUBLE);

  }
  
  // the misc sensors
  if (n_misc_sensors > 0) {

    if (semTake(sm_misc_sensor_sem,NO_WAIT) == ERROR) {
      
      ++count_no_broadcast;
      
    } else {
      
      for (i=1; i<=n_misc_sensors; ++i)
	sm_misc_sensor_data[i] = (float) misc_sensor[i];
      
      memcpy((void *)(&sm_misc_sensor->value[1]),
	     (const void*)(&sm_misc_sensor_data[1]),
	     sizeof(float)*n_misc_sensors);
      
      semGive(sm_misc_sensor_sem);
      
    }

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  triggerSynchronization
\date  April 1999
   
\remarks 

        flushes the synchronization semaphores -- not that under SL
        Unix we use exact synchronization between task servo and motor
        servo, such that semGive is used. (Tight sync uses
        WAIT_FOREVER on the synchronization semaphores, such that one
        must not miss a sychronization signal (otherwise all processes
        would hang)
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static void
triggerSynchronization(void)
{
  
  int i;
  int iaux;
  static double last_openGL_time = 0.0;
  double current_time;
  
#ifdef VX
  semFlush(sm_1to1_sem);
#else
  semGive(sm_1to1_sem);
#endif

  // note: synchronizing on the remainder=1 allows starting all servos
  // immediately in the first run of the motor servo

  if (motor_servo_calls%2==1)
#ifdef VX
    semFlush(sm_1to2_sem);
#else
    semGive(sm_1to2_sem);
#endif

  if (motor_servo_calls%3==1)
#ifdef VX
    semFlush(sm_1to3_sem);
#else
    semGive(sm_1to3_sem);
#endif
  
  if (motor_servo_calls%4==1)
#ifdef VX
    semFlush(sm_1to4_sem);
#else
    semGive(sm_1to4_sem);
#endif

  if (motor_servo_calls%5==1)
#ifdef VX
    semFlush(sm_1to5_sem);
#else
    semGive(sm_1to5_sem);
#endif

  iaux = (int)(((double)motor_servo_rate)/60.0+0.5);
  if (motor_servo_calls%iaux==1)
    semFlush(sm_60Hz_sem);

#ifdef VX
  if (motor_servo_calls%iaux==1)
    semFlush(sm_openGL_servo_sem);
#else
  {
#ifdef __XENO__
    struct timespec t;
    
    clock_gettime(CLOCK_MONOTONIC,&t);
    current_time = (double) t.tv_sec + ((double)t.tv_nsec)/1.e9;
#else
    struct timeval t;
    
    gettimeofday(&t,NULL);
    current_time = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
#endif


    if (current_time-last_openGL_time >= 1./R60HZ) {
      semFlush(sm_openGL_servo_sem);
      last_openGL_time = current_time; 
    }
 
  }
#endif
  
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
  int i,j,k;
  char name[20];

  // check whether a message is available
  if (semTake(sm_motor_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_motor_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take motor message semaphore\n");
    return FALSE;
  }

  for (k=1; k<=sm_motor_message->n_msgs; ++k) {

    // get the name of this message
    strcpy(name,sm_motor_message->name[k]);

    // act according to the message name
    if (strcmp(name,"changePIDGains") == 0) {
      float buf[n_dofs*3+1];
      extern double *controller_gain_th;
      extern double *controller_gain_thd;
      extern double *controller_gain_int;
      
      memcpy(&(buf[1]),sm_motor_message->buf+sm_motor_message->moff[k],sizeof(float)*(3*n_dofs));

      for (i=1; i<=n_dofs; ++i) {
	controller_gain_th[i]  = (double) buf[i];
	controller_gain_thd[i] = (double) buf[i+n_dofs];
	controller_gain_int[i] = (double) buf[i+2*n_dofs];
      }
      
    }

    // see whether the user programmed a message interception
    userCheckForMessage(name,k);

  }

  // give back semaphore
  sm_motor_message->n_msgs = 0;
  sm_motor_message->n_bytes_used = 0;
  semGive(sm_motor_message_sem);


  return TRUE;
}

