/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo.c

  \author  Stefan Schaal
  \date    June 1999

  ==============================================================================
  \remarks

  The program communicates with the QuickMag system and receives
  the (open loop) vision information. Since there is no data
  buffering in the QuickMag, it is necessary to poll the QuickMag
  connection all the time. The 712 transition module of the 
  QuickMag is used for communication.

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_vision_servo.h"
#include "SL_shared_memory.h"
#include "lwpr.h"
#include "SL_man.h"
#include "SL_common.h"

#define TIME_OUT_NS  1000000000

/* global variables */
int           servo_enabled;
double        servo_time;
double        vision_servo_time;
int           vision_servo_rate = R60HZ;
int           vision_servo_calls;
char          current_pp_name[100];
int           vision_servo_initialized = FALSE;
int           vision_servo_errors;
int           count_all_frames;
int           count_lost_frames;
int           *blob_is_endeffector;
int           learn_transformation_flag = FALSE;
int           raw_blob_overwrite_flag=FALSE;
int           no_hardware_flag = FALSE;

/* local variables */
static int models_read = FALSE;

/* local functions */
static int  broadcast_blobs(void);
static int  receive_cartesian(void);
static int  learn_transformation(void );
static int  check_raw_blob_overwrite(void);

/* global functions */
int  reset_learning( void );
int  save_learning( void );
int  set_learning( void );

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_servo
\date  June 1999
   
\remarks 

        initializes vision servo structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
init_vision_servo()
{
  int  j, i, k;
  char string[100];
  static int firsttime = TRUE;
  extern int  init_user_commands(void);
  
  if (vision_servo_initialized) {
    printf("Vision Servo is already initialized\n");
    return;
  }
  
  if (firsttime) {
    firsttime = FALSE;
    blob_is_endeffector = my_ivector(1,max_blobs);
  }

  /* the servo name */
  sprintf(servo_name,"vision");

#ifdef VX
  /* vxworks specific initialization */
  if (!init_vxworks()) 
    return;

  /* init the QuickMag communication */
  if (!init_vision_hardware())
    no_hardware_flag = TRUE;
#else
  no_hardware_flag = TRUE;
#endif
  
  /* initialize the A/D board */
  initialize_ad(d2a_hwv);
  dta(d2a_hwv,d2a_bv,d2a_cv,0);
  
  /* initialize shared memories and shared semaphores */
  if (!init_shared_memory())
    return;

   /* initialize system commands and user commands */
  if (!init_commands())
    return;

  if (!init_user_commands())
    return;

  /* init the vision processing */
  if (!init_vision_processing())
    return;

  /* init the coordinate transformations models */
  if (!init_learning())
    return;

  /* add variables to data collection */
  initCollectData(vision_servo_rate);

  for (i=1; i<=max_blobs; ++i) {
    printf("%d...",i);
    fflush(stdout);
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

  addVarToCollect((char *)&(count_all_frames),"frame_counter","-",INT,FALSE);

  printf("done\n");

  updateDataCollectScript();

  /* add to man pages */

  /* if all worked out, we mark the servo as ready to go */
  vision_servo_initialized = TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_vision_servo
\date  June 1999
   
\remarks 

        executes the sequence of vision servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
run_vision_servo(void)

{

  int i,j;


  /*************************************************************************
   * adjust the servo time (needs to come after acquire_blobs)
   */
  
  ++vision_servo_calls;
  vision_servo_time = vision_servo_calls/(double)vision_servo_rate;
  servo_time = vision_servo_time;

  /*************************************************************************
   * this allows to overwrite the blobs, e.g., by simulated information
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,1500);
  /* reset the blob status if there is no hardware */
  if (no_hardware_flag) {
    for (i=1; i<=max_blobs; ++i) {
      raw_blobs2D[i][1].status = 0;
      raw_blobs2D[i][2].status = 0;
    }
    count_all_frames = vision_servo_calls;
  }
  raw_blob_overwrite_flag = check_raw_blob_overwrite();
  
  /*************************************************************************
   * process the blobs (filtering, conversion into our coordinates
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,2000);
  process_blobs(raw_blobs2D);
  
  /*************************************************************************
   * broadcast the the final blob information in shared memory
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,2500);
  broadcast_blobs();
  
  /*************************************************************************
   * read the robot state
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,3000);
  receive_cartesian();
  
  /*************************************************************************
   * learn the mapping
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,3500);
  learn_transformation();
  
  /*************************************************************************
   * collect data
   */
  
  dta(d2a_hwv,d2a_bv,d2a_cv,4000);
  writeToBuffer();
  
  /*************************************************************************
   * end of program sequence
   */

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  broadcast_blobs
\date  April 1999
   
\remarks 

        copies filtered blob information to shared memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
broadcast_blobs(void)
{
  static int count = 0;
  int i,j,k;
  char string[40];
  
  if (semTake(sm_vision_blobs_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++vision_servo_errors;
    
  } else {

    cSL_VisionBlob(blobs,sm_vision_blobs_data,max_blobs,DOUBLE2FLOAT);

    cSL_VisionBlobaux(raw_blobs2D,sm_vision_blobs_data_aux,
		      max_blobs,DOUBLE2FLOAT);
    
    for (i=1; i<=max_blobs; ++i) {
      if (sm_vision_blobs_data[i].status) 
	sm_vision_blobs->blobs[i] = sm_vision_blobs_data[i];
      else
	sm_vision_blobs->blobs[i].status = sm_vision_blobs_data[i].status;

      sm_vision_blobs_aux->blobs[i] = sm_vision_blobs_data_aux[i];
    }
    
    /* check the post processing specifier */
    strcpy(string,sm_vision_blobs->pp_name);
    sm_vision_blobs->frame_counter = count_all_frames;
    if (strcmp(string,current_pp_name) != 0 && strcmp(string,"") != 0) {
      init_pp(string);
    }
    semGive(sm_vision_blobs_sem);

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_cartesian
\date  April 1999
   
\remarks 

        receives new sensory data 
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_cartesian(void)
{
  
  int i;

  if (!learn_transformation_flag)
    return TRUE;

  if (semTake(sm_cart_states_sem,NO_WAIT) == ERROR) {
    ++vision_servo_errors;
    return FALSE;
  } 

  memcpy((void *)(&sm_cart_states_data[1]),(const void*)(&sm_cart_states->state[1]),
	 sizeof(SL_fCstate)*n_endeffs);

  cSL_Cstate(cart_state,sm_cart_states_data,n_endeffs,FLOAT2DOUBLE);
  
  semGive(sm_cart_states_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  check_raw_blob_overwrite
\date  April 1999
   
\remarks 

        if semaphore is given, overwrite the raw blob information
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     returns TRUE only if the 3D blobs were overwritten -- for 2D blobs,
     no special action needs to be taken as processing just proceeds from
     this fake visual information

 ******************************************************************************/
static int 
check_raw_blob_overwrite(void)
{
  
  int i;
  static int last_frame_counter=0;
  int rc = FALSE;

  /* check whether there is an overwrite request */
  if (semTake(sm_raw_blobs_ready_sem,NO_WAIT) == ERROR) {
    return FALSE;
  } 

  /* check for 3D blob information */
  if (semTake(sm_raw_blobs_sem,ns2ticks(TIME_OUT_NS)) != ERROR) {
    
    for (i=1; i<=max_blobs; ++i) 
      if (sm_raw_blobs->blobs[i].status) {
	sm_raw_blobs_data[i] = sm_raw_blobs->blobs[i];
      }
    
    cBlob3D(raw_blobs,sm_raw_blobs_data,max_blobs,FLOAT2DOUBLE);
    
    if (count_all_frames > last_frame_counter) {
      last_frame_counter = count_all_frames;
    } else {
      ++count_all_frames; /* no vision input at the moment -> fake it */
      last_frame_counter = count_all_frames;
    }
  
    semGive(sm_raw_blobs_sem);

    rc = TRUE;
    
  } 

  /* check for 2D blob information */
  if (semTake(sm_raw_blobs2D_sem,ns2ticks(TIME_OUT_NS)) != ERROR) {

    for (i=1; i<=max_blobs*2; ++i) 
      if (sm_raw_blobs2D->blobs[i].status) {
	sm_raw_blobs2D_data[i] = sm_raw_blobs2D->blobs[i];
      }
    
    cBlob2D(raw_blobs2D,sm_raw_blobs2D_data,max_blobs,FLOAT2DOUBLE);
    
    if (count_all_frames > last_frame_counter) {
      last_frame_counter = count_all_frames;
    } else {
      if (!rc) { /* don't increment if already done for 3D blobs */
	++count_all_frames; /* no vision input at the moment -> fake it */
	last_frame_counter = count_all_frames;
      }
    }
    
    semGive(sm_raw_blobs2D_sem);

  } 



  return rc;
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_learning
\date  Feb 1999
\remarks 

initializes the learning models

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_learning( void )

{
  int i, j;

#ifdef VX  
  if (models_read) {
    ;
  } else {
    if (!readLWPRScript(BLOB2ROBOT_SCRIPT,0,BLOB2ROBOT)) {
      printf("Error when reading script file >%s<\n",BLOB2ROBOT_SCRIPT);
      return FALSE;
    }
  }

  for (i=1; i<=max_blobs; ++i) {
    blob_is_endeffector[i] = FALSE;
  }
#endif
  models_read = TRUE;

  return TRUE;
  
}    

/*!*****************************************************************************
 *******************************************************************************
\note  reset_learning
\date  Feb 1999
\remarks 

resets the learning model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
reset_learning( void )

{
  int i, j;

  if (servo_enabled) {
    printf("To reinitialize the learning the vision servo has to be stopped\n");
    return FALSE;
  }

#ifdef VX  
  if (models_read) {
    deleteLWPR(BLOB2ROBOT);
  }
  if (!readLWPRScript(BLOB2ROBOT_SCRIPT,1,BLOB2ROBOT)) {
      printf("Error when reading script file >%s<\n",BLOB2ROBOT_SCRIPT);
      return FALSE;
  }

  for (i=1; i<=max_blobs; ++i) {
    blob_is_endeffector[i] = FALSE;
  }
#endif
  models_read = TRUE;

  return TRUE;
  
}    

/*!*****************************************************************************
 *******************************************************************************
\note  save_learning
\date  Feb 1999
\remarks 

saves the learned model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
save_learning( void )

{
  int i, j;

  if (models_read) {
    writeLWPR(BLOB2ROBOT);
  }

  return TRUE;
  
}    

/*!*****************************************************************************
 *******************************************************************************
\note  set_learning
\date  Feb 1999
\remarks 

allows setting which endeffector corresponds to which blob

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
set_learning( void )

{
  int  i,j;
  char string[100];
  
  for (i=1; i<=max_blobs; ++i) {
    sprintf(string,"Which Endeffector Corresponds to %s",blob_names[i]);
    get_int(string, blob_is_endeffector[i],&(blob_is_endeffector[i]));
    if (blob_is_endeffector[i] < 1 || blob_is_endeffector[i] > n_endeffs)
      blob_is_endeffector[i] = FALSE;
  }
  
  return TRUE;
  
}    



/*!*****************************************************************************
 *******************************************************************************
\note  learn_transformation
\date  Feb 1999
\remarks 

learns the quickmag blob to robot cartesian space mapping

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static int
learn_transformation(void )

{
  int  i,j;
  char string[100];
  double x[2+2+1];
  double y[N_CART+1];
  int    rfID;
  
  if (semTake(sm_learn_blob2body_sem,NO_WAIT) == ERROR)
    return TRUE;

#ifdef VX
  for (i=1; i<=max_blobs; ++i) {
    if (blob_is_endeffector[i] != 0 && 
	raw_blobs2D[i][1].status &&
	raw_blobs2D[i][2].status ) {

      x[1] = raw_blobs2D[i][1].x[1];
      x[2] = raw_blobs2D[i][1].x[2];
      x[3] = raw_blobs2D[i][2].x[1];
      x[4] = raw_blobs2D[i][2].x[2];

      for (j=_X_ ; j<= _Z_; ++j) {
	y[j] = cart_state[blob_is_endeffector[i]].x[j];
      }
      addDataToLWPR(BLOB2ROBOT, x, x, y,FALSE,&rfID);
    }
  }
#endif  
  return TRUE;
  
}    



