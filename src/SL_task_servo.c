/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo.c

  \author  Stefan Schaal
  \date    1997

  ==============================================================================
  \remarks

  manages the control loop to run different tasks 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_shared_memory.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_filters.h"

#define TIME_OUT_NS  1000000000

/* variables for the task servo */
int    task_servo_errors;
long   task_servo_calls=0;
int    task_servo_initialized = FALSE;
double task_servo_time=0;
double servo_time=0;
int    servo_enabled;
int    task_servo_rate;
int    frame_counter = 0;
int    exit_on_stop = FALSE;

/* local variables */
static int servo_mode = MOTORSERVO;

/* global functions */
int  step(int jid, int iamp);

/* local functions */
static int  send_commands(void);
static int  send_invdyn(void);
static int  send_cartesian(void);
static int  receive_sensors(void);
static int  receive_blobs(void);
static void compute_kinematics(void);
static void sim_stop(void);
static void sim_step(void);

/*!*****************************************************************************
 *******************************************************************************
  \note  init_task_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
void
init_task_servo(void)
{
  int j, i;
  char   string[100];
  FILE  *in;
  extern void add_goto_task(void);
  extern void add_sine_task(void);
  extern void add_traj_task(void);
  extern void add_goto_cart_task(void);
  extern int  init_user_commands(void);

  if (task_servo_initialized) {
    printf("Task Servo is already initialized\n");
    return;
  }

#ifdef VX
  /* vxworks specific initialization */
  if (!init_vxworks()) 
    return;
#endif

  /* the servo name */
  sprintf(servo_name,"task");

  /* initialize the A/D board */
  initialize_ad(d2a_hwt);
  dta(d2a_hwt,d2a_bt,d2a_ct,0);
  
  /* initialize the tasks */
  initTasks();
  add_goto_task();
  add_sine_task();
  add_traj_task();
  add_goto_cart_task();

  /* initialize shared memories and shared semaphores */
  if (!init_shared_memory())
    return;

   /* initialize system commands and user commands */
  if (!init_commands())
    return;
    
  if (!init_user_commands())
    return;

  /* inverse dynamics */
  if (!init_dynamics()) 
    return;
  
  /* initialize kinematicss */
  init_kinematics();
  
  /* get the max, min, and offsets of the position sensors */
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    return;

  /* read the control gains and max controls  */
  if (!read_gains(config_files[GAINS],NULL,NULL,NULL))
    return;

  /* filtering */
  if (!init_filters())
    return;

  /* add variables to data collection */
  task_servo_rate=servo_base_rate/task_servo_ratio;
  initCollectData(task_servo_rate);

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
    sprintf(string,"%s_ufb",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].ufb),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].load),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_des_th",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].th),string,"rad",DOUBLE,FALSE);
    sprintf(string,"%s_des_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thd),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thdd),string,"rad/s^2",DOUBLE,FALSE);
    sprintf(string,"%s_uff",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].uff),string,"Nm",DOUBLE,FALSE);
  }

  for (i=1; i<=n_endeffs; ++i) {

    sprintf(string,"%s_x",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_y",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_z",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_X_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Y_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Z_]),string,"rad/s",DOUBLE,FALSE);

    sprintf(string,"%s_add",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_X_]),string,"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_bdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Y_]),string,"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_gdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Z_]),string,"rad/s2",DOUBLE,FALSE);

    sprintf(string,"%s_des_x",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_y",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_z",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_X_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Y_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Z_]),string,"rad/",DOUBLE,FALSE);

    sprintf(string,"%s_tx",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_ty",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tz",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_txd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tyd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tzd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_txdd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tydd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tzdd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_cons_x",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_X_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_y",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_Y_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_z",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_Z_]),string,"-",INT,FALSE);

    sprintf(string,"%s_cons_a",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_A_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_b",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_B_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_g",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_G_]),string,"-",INT,FALSE);

    sprintf(string,"%s_cfx",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_X_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_cfy",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_Y_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_cfz",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_Z_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_cta",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_A_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_ctb",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_B_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_ctg",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_G_]),string,"-",DOUBLE,FALSE);


  }

  for (i=1; i<=max_blobs; ++i) {

    sprintf(string,"%s_stat",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_x",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_y",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_z",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Z_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_xd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_X_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_yd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Y_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_zd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Z_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_xdd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_X_]),string,"m/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_ydd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_Y_]),string,"m/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_zdd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_Z_]),string,"m/s^2", DOUBLE,FALSE);

    sprintf(string,"%s_rstat1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_rstat2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_Y_]),string,"m", DOUBLE,FALSE);
  }

  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_sensor[i]),string,"-",DOUBLE,FALSE);
  }


  /* the center of gravity */
  addVarToCollect((char *)&(cog.x[_X_]),"cog_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.x[_Y_]),"cog_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.x[_Z_]),"cog_z","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog.xd[_X_]),"cog_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.xd[_Y_]),"cog_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.xd[_Z_]),"cog_zd","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog_des.x[_X_]),"cog_des_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.x[_Y_]),"cog_des_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.x[_Z_]),"cog_des_z","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog_des.xd[_X_]),"cog_des_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.xd[_Y_]),"cog_des_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.xd[_Z_]),"cog_des_zd","m",DOUBLE,FALSE);

  /* the state of the base */
  addVarToCollect((char *)&(base_state.x[_X_]),"base_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Y_]),"base_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Z_]),"base_z","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xd[_X_]),"base_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Y_]),"base_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Z_]),"base_zd","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xdd[_X_]),"base_xdd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Y_]),"base_ydd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Z_]),"base_zdd","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_orient.q[_Q0_]),"base_q0","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q1_]),"base_q1","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q2_]),"base_q2","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q3_]),"base_q3","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.qd[_Q0_]),"base_qd0","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q1_]),"base_qd1","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q2_]),"base_qd2","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q3_]),"base_qd3","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.ad[_A_]),"base_ad","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_B_]),"base_bd","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_G_]),"base_gd","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.add[_A_]),"base_add","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_B_]),"base_bdd","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_G_]),"base_gdd","-",DOUBLE,FALSE);



  addVarToCollect((char *)&(frame_counter),"frame_counter","-",INT,FALSE);

  printf("done\n");

  updateDataCollectScript();

  /* add to man pages */
  addToMan("status","displays information about the servo",status);
  addToMan("freeze","freeze the robot in current posture",freeze);
  addToMan("f","freeze the robot in current posture",f);
  addToMan("step","step commands to a joint",sim_step);
  addToMan("stop","kills the robot control",sim_stop);

  /* if all worked out, we mark the servo as ready to go */
  task_servo_initialized = TRUE;

  scd();

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_task_servo
\date  Feb 1999
\remarks 

        This program executes the sequence of task servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
run_task_servo(void)

{
  int    j,i;
  double dt;

  dta(d2a_hwt,d2a_bt,d2a_ct,0);
  
  /* adjust the servo time */
  ++task_servo_calls;
  task_servo_time = servo_time = task_servo_calls/(double)task_servo_rate;

  /* zero any external simulated forces */
  bzero((void *)uext_sim,sizeof(SL_uext)*(n_dofs+1));
  
  /*********************************************************************
   * receive sensory data
   */

  if (!receive_sensors()) {
    stop("Problem when receiving sensor data");
    return FALSE;
  }
  
  dta(d2a_hwt,d2a_bt,d2a_ct,500);

  if (task_servo_calls <= 1) { // initialize desired at first servo tick
    for (i=1; i<=n_dofs; ++i) {
      joint_des_state[i].th  = joint_state[i].th;
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].uff = 0.0;
    }
  }

  /*********************************************************************
   * compute useful kinematic variables
   */
  
  compute_kinematics();
  
  dta(d2a_hwt,d2a_bt,d2a_ct,1000);
  
  /**********************************************************************
   * send out the kinematic variables
   */
  
  if (!send_cartesian()) {
    stop("Problem when sending cartesian state");
    return FALSE;
  }
  
  dta(d2a_hwt,d2a_bt,d2a_ct,1500);
  
  
  /**********************************************************************
   * receive vision blobs
   */
  
  if (!receive_blobs()) {
    stop("Problem when receiving vision blobs");
    return FALSE;
  }
  
  dta(d2a_hwt,d2a_bt,d2a_ct,2000);
  
  
  /*********************************************************************
   * call the tasks
   */
  
#ifndef VX
  /* run general simulation specific task servo computations */
  if (!run_generic_tasks()) {
    stop("Problem in run_generic_tasks");
    return FALSE;
  }
#endif

  /* run user tasks */
  runTask();
  
  dta(d2a_hwt,d2a_bt,d2a_ct,3000);
  
  /**********************************************************************
   * send out the new commands
   */

  switch (servo_mode) {
  case INVDYNSERVO:
    if (!send_invdyn()) {
      stop("Problem when sending invdyn");
      return FALSE;
    }
    break;
  case CARTSERVO:
    break;
  default:
    if (!send_commands()) {
      stop("Problem when sending commands");
      return FALSE;
    }
  }

  dta(d2a_hwt,d2a_bt,d2a_ct,3500);
  
  
  /*************************************************************************
   * collect data
   */

  writeToBuffer();

  dta(d2a_hwt,d2a_bt,d2a_ct,4000);
  
  /*************************************************************************
   * end of program sequence
   */

  return TRUE;
  
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
  printf("            Time                   = %f\n",task_servo_time);
  printf("            Servo Calls            = %ld\n",task_servo_calls);
  printf("            Servo Rate             = %d\n",task_servo_rate);
  printf("            Servo Errors           = %d\n",task_servo_errors);
  printf("            Servo Initialize       = %d\n",task_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("            Task                   = %s\n",current_task_name);
  printf("            Vision Frame Counter   = %d\n",frame_counter);
  printf("            Servo Mode             = %d\n",servo_mode);
#ifdef __XENO__
  extern long count_xenomai_mode_switches;
  printf("            Xeonmai Mode Swiches   = %ld\n",count_xenomai_mode_switches);
#endif
  printf("\n");

}
/*!*****************************************************************************
 *******************************************************************************
\note  receive_sensors
\date  April 1999
   
\remarks 

        receives new sensory data 
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_sensors(void)
{
  
  int i;


  // the joint state
  if (semTake(sm_joint_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {

    ++task_servo_errors;
    return FALSE;

  } 


  memcpy((void *)(&sm_joint_state_data[1]),(const void*)(&sm_joint_state->joint_state[1]),
	 sizeof(SL_fJstate)*n_dofs);

  cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,FLOAT2DOUBLE);
  
  semGive(sm_joint_state_sem);


  // the misc sensors

  if (n_misc_sensors > 0) {
    
    if (semTake(sm_misc_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
      
      ++task_servo_errors;
      return FALSE;
      
    } 
    
    memcpy((void *)(&sm_misc_sensor_data[1]),(const void*)(&sm_misc_sensor->value[1]),
	   sizeof(float)*n_misc_sensors);
    
    for (i=1; i<=n_misc_sensors; ++i)
      misc_sensor[i] = (double) sm_misc_sensor_data[i];
    
    
    semGive(sm_misc_sensor_sem);

  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_commands
\date  April 1999
   
\remarks 

        just copies the sensor data to the share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_commands(void)
{
  
  int i,j;
  

  if (semTake(sm_sjoint_des_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++task_servo_errors;
    return FALSE;
  }

  /* check for joint limits */

  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      if (joint_des_state[i].th > joint_range[i][MAX_THETA])
	joint_des_state[i].th = joint_range[i][MAX_THETA];
      if (joint_des_state[i].th < joint_range[i][MIN_THETA]) 
	joint_des_state[i].th = joint_range[i][MIN_THETA];
    }
  }

  cSL_SDJstate(joint_des_state,sm_sjoint_des_state_data,n_dofs,DOUBLE2FLOAT);
    
  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      sm_sjoint_des_state_data[i].status = TRUE;
      sm_sjoint_des_state->sjoint_des_state[i] = sm_sjoint_des_state_data[i];
    }
  }

  semGive(sm_sjoint_des_state_sem);
  semGive(sm_sjoint_des_state_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_invdyn
\date  April 1999
   
\remarks 

        just copies the invdyn data to shared memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_invdyn(void)
{
  
  int i;

  if (semTake(sm_joint_des_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++task_servo_errors;
    return FALSE;
  }

  /* check for joint limits */

  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      if (joint_des_state[i].th > joint_range[i][MAX_THETA])
	joint_des_state[i].th = joint_range[i][MAX_THETA];
      if (joint_des_state[i].th < joint_range[i][MIN_THETA]) 
	joint_des_state[i].th = joint_range[i][MIN_THETA];
    }
  }

  cSL_DJstate(joint_des_state,sm_joint_des_state_data,n_dofs,DOUBLE2FLOAT);

  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      sm_joint_des_state->joint_des_state[i] = sm_joint_des_state_data[i];
    }
  }

  semGive(sm_joint_des_state_sem);
  semGive(sm_joint_des_state_ready_sem);
  
  return TRUE;
  
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
  dts();

#ifdef UNIX
  if (exit_on_stop)
    // this allows real robots to abort completely on stop()
    exit(-1);
  else {
    // the simulation just resets
    // extern void reset(void);
    // reset();
    ;
  }
#endif
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  freeze
\date  August 7, 1992 
   
\remarks 

       freezes an ongoing task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
f(void) {
  freeze();
}
void
freeze(void)
{
  int i;
  setTaskByName(NO_TASK);
  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].thd   = 0.0;
    joint_des_state[i].thdd = 0.0;
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  step
\date  April 1999
   
\remarks 

        simple steps to check out the gains
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     jid  : jointID
 \param[in]     amp  : amplitude

 ******************************************************************************/
static void
sim_step(void)  /* for simulation only */
{
  static int jid=1,iamp=5;

  get_int("Step which joint?",jid,&jid);
  get_int("Step amplitude as integer number (times 0.1 rad)",iamp,&iamp);
  step(jid,iamp);
}

int
step(int jid, int iamp)

{
  static int flag = 1;
  double amp;

  amp = (double)iamp/10.;

  if (jid < 1 || jid > n_dofs || strcmp(current_task_name,NO_TASK)!= 0 ||
      amp > 0.5 || amp < 0)
    return FALSE;

  if (flag) {
    printf("Step %s by amp = %f\n",joint_names[jid],amp);
    joint_des_state[jid].th += amp;
    flag = FALSE;
  } else {
    printf("Step %s by amp = %f\n",joint_names[jid],-amp);
    joint_des_state[jid].th -= amp;
    flag = TRUE;
  }

  scd();
  
  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  compute_kinematics
\date  June 99
   
\remarks 

       computes kinematic variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
static void
compute_kinematics(void)
{
  int i,j,r,o;
  static int firsttime = TRUE;
  static SL_DJstate *temp;
  static Matrix last_J;
  static Matrix last_Jbase;
  
  if (firsttime) {
    temp=(SL_DJstate *)my_calloc((unsigned long)(n_dofs+1),sizeof(SL_DJstate),MY_STOP);
    last_J      = my_matrix(1,6*n_endeffs,1,n_dofs);
    last_Jbase  = my_matrix(1,6*n_endeffs,1,2*N_CART);
  }

  /* compute the desired link positions */
  linkInformationDes(joint_des_state,&base_state,&base_orient,endeff,
		     joint_cog_mpos_des,joint_axis_pos_des,joint_origin_pos_des,
		     link_pos_des,Alink_des);

  /* the desired endeffector information */
  for (i=1; i<=N_CART; ++i) {
    for (j=1; j<=n_endeffs; ++j) {
      cart_des_state[j].x[i] = link_pos_des[link2endeffmap[j]][i];
    }
  }

  /* the desired quaternian of the endeffector */
  for (j=1; j<=n_endeffs; ++j) {
    linkQuat(Alink_des[link2endeffmap[j]],&(cart_des_orient[j]));
  }


  /* addititional link information */
  linkInformation(joint_state,&base_state,&base_orient,endeff,
		  joint_cog_mpos,joint_axis_pos,joint_origin_pos,
		  link_pos,Alink);

  /* create the endeffector information */
  for (i=1; i<=N_CART; ++i) {
    for (j=1; j<=n_endeffs; ++j) {
      cart_state[j].x[i] = link_pos[link2endeffmap[j]][i];
    }
  }

  /* the quaternian of the endeffector */
  for (j=1; j<=n_endeffs; ++j) {
    linkQuat(Alink[link2endeffmap[j]],&(cart_orient[j]));
  }

  /* the COG position */
  compute_cog();

  /* the jacobian */
  jacobian(link_pos,joint_origin_pos,joint_axis_pos,J);
  baseJacobian(link_pos,joint_origin_pos,joint_axis_pos,Jbase);

  jacobian(link_pos_des,joint_origin_pos_des,joint_axis_pos_des,Jdes);
  baseJacobian(link_pos_des,joint_origin_pos_des,joint_axis_pos_des,Jbasedes);

  /* numerical time derivative of Jacobian */
  if (!firsttime) {
    mat_sub(J,last_J,dJdt);
    mat_mult_scalar(dJdt,(double)task_servo_rate,dJdt);
    mat_sub(Jbase,last_Jbase,dJbasedt);
    mat_mult_scalar(dJbasedt,(double)task_servo_rate,dJbasedt);
  }
  mat_equal(J,last_J);
  mat_equal(Jbase,last_Jbase);

  /* compute the cartesian velocities and accelerations */

  for (j=1; j<=n_endeffs; ++j) {

    for (i=1; i<=N_CART; ++i) {

      cart_state[j].xd[i]     = 0.0;
      cart_state[j].xdd[i]    = 0.0;

      cart_orient[j].ad[i]     = 0.0;
      cart_orient[j].add[i]    = 0.0;

      cart_des_state[j].xd[i] = 0.0;
      cart_des_orient[j].ad[i] = 0.0;

      /* contributations from the joints */
      for (r=1; r<=n_dofs; ++r) {
	cart_state[j].xd[i]     += J[(j-1)*6+i][r] * joint_state[r].thd;
	cart_orient[j].ad[i]    += J[(j-1)*6+i+3][r] * joint_state[r].thd;

	cart_des_state[j].xd[i] += Jdes[(j-1)*6+i][r] *joint_des_state[r].thd;
	cart_des_orient[j].ad[i]+= Jdes[(j-1)*6+i+3][r] * joint_des_state[r].thd;

	cart_state[j].xdd[i]    += J[(j-1)*6+i][r] * joint_state[r].thdd + 
	  dJdt[(j-1)*6+i][r] * joint_state[r].thd;
	cart_orient[j].add[i]   += J[(j-1)*6+i+3][r] * joint_state[r].thdd + 
	  dJdt[(j-1)*6+i+3][r] * joint_state[r].thd;
      }

      /* contributations from the base */
      for (r=1; r<=N_CART; ++r) {
	cart_state[j].xd[i]     += Jbase[(j-1)*6+i][r] * base_state.xd[r];
	cart_orient[j].ad[i]    += Jbase[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_state[j].xd[i]     += Jbase[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_orient[j].ad[i]    += Jbase[(j-1)*6+i+3][3+r] * base_orient.ad[r];

	cart_des_state[j].xd[i]     += Jbasedes[(j-1)*6+i][r] * base_state.xd[r];
	cart_des_orient[j].ad[i]    += Jbasedes[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_des_state[j].xd[i]     += Jbasedes[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_des_orient[j].ad[i]    += Jbasedes[(j-1)*6+i+3][3+r] * base_orient.ad[r];

	cart_state[j].xdd[i]    += Jbase[(j-1)*6+i][r] * base_state.xdd[r] + 
	  dJbasedt[(j-1)*6+i][r] * base_state.xd[r];
	cart_orient[j].add[i]   += Jbase[(j-1)*6+i+3][r] * base_state.xdd[r] + 
	  dJbasedt[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_state[j].xdd[i]    += Jbase[(j-1)*6+i][3+r] * base_orient.add[r] + 
	  dJbasedt[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_orient[j].add[i]   += Jbase[(j-1)*6+i+3][3+r] * base_orient.add[r] + 
	  dJbasedt[(j-1)*6+i+3][3+r] * base_orient.ad[r];
      }

    }

    /* compute quaternion derivatives */
    quatDerivatives(&(cart_orient[j]));
    quatDerivatives(&(cart_des_orient[j]));
    for (r=1; r<=N_QUAT; ++r)
      cart_des_orient[j].qdd[r] = 0.0; // we don't have dJdes_dt so far

  }

  /* reset first time flag */
  firsttime = FALSE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  send_cartesian
\date  April 1999
   
\remarks 

        just copies the cartesian data to share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_cartesian(void)
{
  
  int i;
  
  if (semTake(sm_cart_states_sem,NO_WAIT) == ERROR) {
    
    ;

  } else {

    cSL_Cstate(cart_state,sm_cart_states_data,n_endeffs,DOUBLE2FLOAT);

    memcpy((void *)(&sm_cart_states->state[1]),(const void*)(&sm_cart_states_data[1]),
	   sizeof(SL_fCstate)*n_endeffs);

    semGive(sm_cart_states_sem);

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_blobs
\date  April 1999
   
\remarks 

        receives vision blob information
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_blobs(void)
{
  
  static int count = 0;
  int i,j,k,rc;
  char string[40];
  
  if (semTake(sm_vision_blobs_sem,NO_WAIT) == ERROR)
    {
      ;
    }
  else 
    {
      rc = sm_vision_blobs->frame_counter;
      if (frame_counter != rc)
	{
	  frame_counter = rc;
	  for (i=1; i<=max_blobs; ++i)
	    {
	      if (sm_vision_blobs->blobs[i].status)
		{
		  sm_vision_blobs_data[i] = sm_vision_blobs->blobs[i];
		} 
	      else
		sm_vision_blobs_data[i].status = FALSE;
	      sm_vision_blobs_data_aux[i] = sm_vision_blobs_aux->blobs[i];
	    }
	  cSL_VisionBlob(blobs,sm_vision_blobs_data,max_blobs,FLOAT2DOUBLE);
	  cSL_VisionBlobaux(raw_blobs2D,sm_vision_blobs_data_aux,max_blobs,
			    FLOAT2DOUBLE);

	}

      strcpy(sm_vision_blobs->pp_name,current_vision_pp);
      semGive(sm_vision_blobs_sem);
    }
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_raw_blobs
\date  April 1999
   
\remarks 

        just copies the raw blobs to share memory -- this is used
	to overwrite the vision info and allows simulation of visual
	blobs
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int 
send_raw_blobs(void)
{
  
  int i;
  
  if (semTake(sm_raw_blobs_sem,NO_WAIT) == ERROR) {
    return FALSE;
  }

  cBlob3D(raw_blobs,sm_raw_blobs_data,max_blobs,DOUBLE2FLOAT);

  for (i=1; i<=max_blobs; ++i) {
    if (sm_raw_blobs_data[i].status)
      sm_raw_blobs->blobs[i] = sm_raw_blobs_data[i];
  }
  
  semGive(sm_raw_blobs_sem);
  semGive(sm_raw_blobs_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_raw_blobs2D
\date  April 1999
   
\remarks 

        just copies the raw blobs2D to share memory -- this is used
	to overwrite the vision info and allows simulation of visual
	blobs
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int 
send_raw_blobs2D(void)
{
  
  int i;
  
  if (semTake(sm_raw_blobs2D_sem,NO_WAIT) == ERROR) {
    return FALSE;
  }

  cBlob2D(raw_blobs2D,sm_raw_blobs2D_data,max_blobs,DOUBLE2FLOAT);

  for (i=1; i<=max_blobs*2; ++i) {
    if (sm_raw_blobs2D_data[i].status)
      sm_raw_blobs2D->blobs[i] = sm_raw_blobs2D_data[i];
  }
  
  semGive(sm_raw_blobs2D_sem);
  semGive(sm_raw_blobs_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  setServoMode
\date  June 2000
   
\remarks 

        switches the task servo into different servo modes, e.g.,
        direct, invdyn, or cartdyn
        
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     type  :  servo type

 ******************************************************************************/
int 
setServoMode(int type)
{

  printf("setServoMode is depricated --- simply remove statement from your code\n");
  servo_mode = MOTORSERVO;
  
  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  changePDGains
\date  Nov. 2005
   
\remarks 

        sends the request to change the PD gains to the relevant servos

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     pGain : the proportional gains
 \param[in]     dGain : the derivative gains

 ******************************************************************************/
void 
changePDGains(double *pGain, double *dGain) 
{
  int i,j;
  float buf[2*n_dofs+1];
  unsigned char cbuf[(2*n_dofs)*sizeof(float)];

  for (i=1; i<=n_dofs; ++i) {
    buf[i] = pGain[i];
    buf[i+n_dofs] = dGain[i];
  }
    
  memcpy(cbuf,(void *)&(buf[1]),(2*n_dofs)*sizeof(float));
    
  sendMessageSimulationServo("changePDGains",(void *)cbuf,(2*n_dofs)*sizeof(float));
  sendMessageMotorServo("changePDGains",(void *)cbuf,(2*n_dofs)*sizeof(float));

}


