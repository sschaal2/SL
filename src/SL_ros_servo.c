/*!=============================================================================
  ==============================================================================

  \ingroup SLros
  
  \file    SL_ros_servo.c

  \author  Stefan Schaal
  \date    July 2010

  ==============================================================================
  \remarks

  manages the communcation to the ROS master node

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_ros_servo.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_oscilloscope.h"
#include "SL_kinematics.h"

#define TIME_OUT_NS  100000

/* variables for the task servo */
int    ros_servo_errors;
long   ros_servo_calls=0;
int    ros_servo_initialized = FALSE;
double ros_servo_time=0;
double servo_time=0;
int    servo_enabled;
int    ros_servo_rate;

/* local variables */

/* global functions */

/* local functions */
static int  receive_sensors(void);
static int  receive_des_commands(void);
static int  receive_blobs(void);
static void compute_kinematics(void);

/*!*****************************************************************************
 *******************************************************************************
  \note  init_ros_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
init_ros_servo(void)
{
  int j, i;
  char   string[100];

  if (ros_servo_initialized) {
    printf("Task Servo is already initialized\n");
    return;
  }

  // the servo name
  sprintf(servo_name,"ros");

  // set oscilloscope to start value
  setOsc(d2a_cr,0.0);

  // initialize shared memories and shared semaphores
  if (!init_shared_memory())
    return;

  // initialize kinematics
  init_kinematics();

  // initializes user specific issues
  init_user_ros();

  // add variables to data collection
  ros_servo_rate=servo_base_rate/task_servo_ratio;
  initCollectData(ros_servo_rate);

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

  // Cartesian variables
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

  }

  // misc sensors
  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_sensor[i]),string,"-",DOUBLE,FALSE);
  }


  // the state of the base
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

  printf("done\n");

  updateDataCollectScript();

  // add to man pages
  addToMan("status","displays information about the servo",status);

  // if all worked out, we mark the servo as ready to go
  ros_servo_initialized = TRUE;

  scd();

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_ros_servo
\date  Feb 1999
\remarks 

This program executes the sequence of ros servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
int
run_ros_servo(void)

{
  int    j,i;
  double dt;

  setOsc(d2a_cr,0.0);
  
  // adjust the servo time
  ++ros_servo_calls;
  ros_servo_time = servo_time = ros_servo_calls/(double)ros_servo_rate;


  /*********************************************************************
   * receive sensory data
   */

  if (!receive_sensors()) {
    printf("Problem when receiving sensor data\n");
    return FALSE;
  }
  
  setOsc(d2a_cr,10.0);

  /*********************************************************************
   * receive commands
   */

  if (!receive_des_commands()) {
    printf("Problem when receiving desired commands\n");
    return FALSE;
  }
  
  setOsc(d2a_cr,15.0);

  /*********************************************************************
   * compute useful kinematic variables
   */
  
  compute_kinematics();
  
  setOsc(d2a_cr,20.0);

  /**********************************************************************
   * do ROS communication
   */

  // TO BE FILLED IN

  setOsc(d2a_cr,80.0);
  
  /**********************************************************************
   * do user specific ROS functions
   */

  run_user_ros();

  setOsc(d2a_cr,90.0);
  


  /*************************************************************************
   * collect data
   */

  writeToBuffer();

  setOsc(d2a_cr,100.0);
  
  /*************************************************************************
   * end of program sequence
   */

  return TRUE;
  
}
 
/*!*****************************************************************************
 *******************************************************************************
\note  status
\date  July 2010
   
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
  printf("            Time                   = %f\n",ros_servo_time);
  printf("            Servo Calls            = %ld\n",ros_servo_calls);
  printf("            Servo Rate             = %d\n",ros_servo_rate);
  printf("            Servo Errors           = %d\n",ros_servo_errors);
  printf("            Servo Initialize       = %d\n",ros_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
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

    ++ros_servo_errors;
    return FALSE;

  } 


  memcpy((void *)(&sm_joint_state_data[1]),(const void*)(&sm_joint_state->joint_state[1]),
	 sizeof(SL_fJstate)*n_dofs);

  cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,FLOAT2DOUBLE);
  
  semGive(sm_joint_state_sem);


  // the misc sensors

  if (n_misc_sensors > 0) {
    
    if (semTake(sm_misc_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
      
      ++ros_servo_errors;
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
    mat_mult_scalar(dJdt,(double)ros_servo_rate,dJdt);
    mat_sub(Jbase,last_Jbase,dJbasedt);
    mat_mult_scalar(dJbasedt,(double)ros_servo_rate,dJbasedt);
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
\note  receive_des_commands
\date  Nov. 2007

\remarks 

receives the commands from the joint_des_state shared memory
structure


*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static int 
receive_des_commands(void)
{
  
  int i;

  if (semTake(sm_des_commands_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++ros_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].th  = (double) sm_des_commands->des_command[i].th;
    joint_des_state[i].thd = (double) sm_des_commands->des_command[i].thd;
    joint_des_state[i].uff = (double) sm_des_commands->des_command[i].uff;
    joint_sim_state[i].u   = (double) sm_des_commands->des_command[i].u;
  }
  
  semGive(sm_des_commands_sem);

  return TRUE;
}

