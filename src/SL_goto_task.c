/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_goto_task.c

  \author 
  \date   

  ==============================================================================
  \remarks

  a simple task for point to point movements in joint space

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_man.h"

/* local variables */
static SL_DJstate *joint_goto_state;
static SL_DJstate *joint_increment;
static SL_DJstate *joint_special_state;
static double goto_speed = 1.0;
static int n_steps;
static int n_goto_steps;
static int special_posture_flag = FALSE;

/* global functions */
void add_goto_task(void);

/* local functions */
static int init_goto_task(void);
static int run_goto_task(void);
static int change_goto_task(void);
static void sim_go(void);
 
/*!*****************************************************************************
 *******************************************************************************
\note  add_goto_task
\date  Feb 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_goto_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {

    firsttime = FALSE;

    joint_goto_state = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    joint_increment = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    joint_special_state = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);

    addTask("Goto Task", init_goto_task, run_goto_task, change_goto_task);
    addToMan("go0","go to default posture",go0);
    addToMan("go","go to a specific posture",sim_go);

  }

}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_goto_task
  \date  Dec. 1997

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_goto_task(void)
{
  int j, i;
  char string[100];
  double max_range=0;
  int ans;
  
  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Goto task can only be run if no other task is running!\n");
    return FALSE;
  }
  
  /* ask user where to go */
  
  if (!special_posture_flag) { 
    
    printf("\n");
    
    for (i=1; i<=n_dofs; ++i) {
      
      sprintf(string,"%s: target theta",joint_names[i]);
      joint_goto_state[i].th = joint_des_state[i].th;
      while (TRUE)  {
	if (!get_double(string,joint_goto_state[i].th,&joint_goto_state[i].th))
	  return FALSE;
	if (joint_goto_state[i].th > joint_range[i][MAX_THETA] ||
	    joint_goto_state[i].th < joint_range[i][MIN_THETA]) {
	  printf("Joint limits are from %5.3f to %5.3f\n",
		 joint_range[i][MIN_THETA],
		 joint_range[i][MAX_THETA]);
	  joint_goto_state[i].th = joint_des_state[i].th;
	  beep(1);
	} else {
	  break;
	}
      }
      
      if (fabs(joint_goto_state[i].th - joint_des_state[i].th) > max_range)
	max_range = fabs(joint_goto_state[i].th - joint_des_state[i].th);
      
      sprintf(string,"%s: target uff",joint_names[i]);
      joint_goto_state[i].uff = joint_des_state[i].uff;
      while (TRUE)  {
	if (!get_double(string,joint_goto_state[i].uff,
			&joint_goto_state[i].uff))
	  return FALSE;
	if (fabs(joint_goto_state[i].uff) > u_max[i]) {
	  printf("Too large torque, max = %f!\n",u_max[i]);
	  joint_goto_state[i].uff = joint_des_state[i].uff;
	  beep(1);
	} else {
	  break;
	}
      }
      
      joint_goto_state[i].thd = 0;
      
    }
    
  } else {  /* the default posture flag was set */
    
    max_range = 0;
    for (i=1; i<=n_dofs; ++i) {
      joint_goto_state[i] = joint_special_state[i];
      if (fabs(joint_goto_state[i].th - joint_des_state[i].th) > max_range)
	max_range = fabs(joint_goto_state[i].th - joint_des_state[i].th);
    }
    check_range(joint_goto_state);

  }
    
  n_steps = 0;
  n_goto_steps = max_range/goto_speed*task_servo_rate;
  if (n_goto_steps == 0) {
    for (i=1; i<=n_dofs; ++i) {
      if (fabs(joint_goto_state[i].uff - joint_des_state[i].uff) != 0)
	n_goto_steps = task_servo_rate;
    }
    if (n_goto_steps == 0) {
      special_posture_flag = FALSE;
      return TRUE;
    } 
  }
  
  printf("Goto Task: steps = %d, time = %f\n",
	 n_goto_steps,n_goto_steps/(double)task_servo_rate);

  for (i=1; i<=n_dofs; ++i) {
    joint_increment[i].th = (joint_goto_state[i].th - joint_des_state[i].th)/
      (double) n_goto_steps;
    joint_increment[i].uff = (joint_goto_state[i].uff - joint_des_state[i].uff)/
      (double) n_goto_steps;
    joint_increment[i].thd = 0;
  }

  if (!special_posture_flag) {
    ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
	return FALSE;
    }
    
    if (ans != 1) 
      return FALSE;
    
  } else {
    
    special_posture_flag = FALSE;
    
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  run_goto_task
  \date  Dec. 1997

  \remarks 

  run the task from the task servo: REAL TIME requirements!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
run_goto_task(void)
{
  int j, i;


  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].th   += joint_increment[i].th;
    joint_des_state[i].thd   = 0.0;
    joint_des_state[i].uff  += joint_increment[i].uff;
  }

  if (++n_steps >= n_goto_steps) 
    setTaskByName(NO_TASK);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_goto_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_goto_task(void)
{
  int j, i;

  get_double("Enter new goto speed",goto_speed,&goto_speed);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go
\date  
   
\remarks 

       a point to point movement

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     jID :  the joint ID

 ******************************************************************************/
static void
sim_go(void)  /* for simulation only */
{
  static int jID=0;
  
  if (!get_int("Which joint? (0=all)",jID,&jID))
    return;
  go(jID);
  
}

int
go(int jID)
{

  int j;
  char string[100];

  if (jID >=1 && jID <=n_dofs) {

    for (j=1; j<=n_dofs; ++j)
      joint_special_state[j] = joint_des_state[j];
    
    sprintf(string,"%s: target theta",joint_names[jID]);
    while (TRUE)  {
      if (!get_double(string,joint_special_state[jID].th,
		      &joint_special_state[jID].th))
	return FALSE;
      if (joint_special_state[jID].th > joint_range[jID][MAX_THETA] ||
	  joint_special_state[jID].th < joint_range[jID][MIN_THETA]) {
	printf("Joint limits are from %5.3f to %5.3f\n",
	       joint_range[jID][MIN_THETA],
	       joint_range[jID][MAX_THETA]);
	joint_special_state[jID].th = joint_des_state[jID].th;
	beep(1);
      } else {
	break;
      }
    }
    
    sprintf(string,"%s: target uff",joint_names[jID]);
    joint_special_state[jID].uff = joint_des_state[jID].uff;
    while (TRUE)  {
      if (!get_double(string,joint_special_state[jID].uff,
		      &joint_special_state[jID].uff))
	return FALSE;
      if (fabs(joint_special_state[jID].uff) > u_max[jID]) {
	printf("Too large torque, max = %f!\n",u_max[jID]);
	joint_special_state[jID].uff = joint_des_state[jID].uff;
	beep(1);
      } else {
	break;
      }
    }

    special_posture_flag = TRUE;
    
  } 

  return setTaskByName("Goto Task");

}

/*!*****************************************************************************
 *******************************************************************************
\note  go0
\date  
   
\remarks 

       a point to point movement to the default posture

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
go0(void)
{
  int i;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = joint_default_state[i];

  setTaskByName("Goto Task");

  return;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go0_wait
\date  
   
\remarks 

       go to the default posture, and returns only after default
       posture has been reached

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
int
go0_wait(void)
{

  int i;
  double last_time;
  double last_draw_time;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = joint_default_state[i];
  
  if (!setTaskByName("Goto Task")) {
    special_posture_flag = FALSE;
    return FALSE;
  }

  last_time = last_draw_time = task_servo_time;
  while (strcmp(current_task_name,NO_TASK) != 0) {
    if (task_servo_time - last_time > 5) {
      special_posture_flag = FALSE;
      printf("time out in go0_wait\n");
      return FALSE;
    }
    taskDelay(1);
  }
  
  return TRUE;

}



/*!*****************************************************************************
 *******************************************************************************
\note  go_target_wait
\date  
   
\remarks 

       go to the special posture, and returns only after 
       posture has been reached

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     target : the array of target states

 ******************************************************************************/
int
go_target_wait(SL_DJstate *target)
{

  int i;
  double last_time;
  double last_draw_time;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = target[i];


  if (!setTaskByName("Goto Task")) {
    special_posture_flag = FALSE;
    return FALSE;
  }

  last_time = last_draw_time = task_servo_time;
  while (strcmp(current_task_name,NO_TASK) != 0) {
    if (task_servo_time - last_time > 5) {
      special_posture_flag = FALSE;
      printf("time out in go_target_wait\n");
      return FALSE;
    }
    taskDelay(1);
  }
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go_target_wait_ID
\date  
   
\remarks 

       go to the special posture, and returns only after 
       posture has been reached -- uses inverse dynamics to
       compute the uff for the final posture

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     target : the array of target states

 ******************************************************************************/
int
go_target_wait_ID(SL_DJstate *target)
{

  int i;
  double last_time;

  /* compute inverse dynamics feedforward command */
  for (i=1; i<=n_dofs; ++i) {
    target[i].thd  = 0.0;
    target[i].thdd = 0.0;
  }

  SL_InverseDynamics(NULL,target,endeff);

  return go_target_wait(target);
  
}



