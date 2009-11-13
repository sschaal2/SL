/*!=============================================================================
  ==============================================================================

  \file    SL_parm_estimate_body.h

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  estimates the inverse dynamics parameters
      
  July 2009: add base link system for estimation

  Note: this file was converted to a header file as it requires to include
        robot specific header files, such that it cannot be added to a SL
	library. SL_parm_estimate.c is just an empty file which includes
        this header.


  ============================================================================*/

/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "utility.h"
#include "utility_macros.h"
#include "statistics.h"
#include "mdefs.h"
#include "PE_declare.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"

#define DOWN_SAMPLE_DEFAULT  5
#define DEBUG        FALSE

enum ConstraintEstType {
  PROJECTION = 1,
  CFORCES,
  BASE_ONLY
};

/* local variables */
static double     sampling_freq;
static Matrix     ATA;
static Vector     ATb;
static Vector     beta;
static Vector     beta_old;
static Matrix     data_pos=NULL;
static Matrix     data_vel=NULL;
static Matrix     data_acc=NULL;
static Matrix     data_u=NULL;
static Matrix     data_bc_pos=NULL;
static Matrix     data_bc_vel=NULL;
static Matrix     data_bc_acc=NULL;
static Matrix     data_bo_q=NULL;
static Matrix     data_bo_ad=NULL;
static Matrix     data_bo_add=NULL;
static iMatrix    data_endeff_c=NULL;
static Matrix     data_endeff_f=NULL;
static int        n_rows;
static int        n_cols;
static double     sse[N_DOFS+2*N_CART+1];
static int        n_sse;
static SL_Jstate  state[N_DOFS+1];
static SL_DJstate invdyn_state[N_DOFS+1];
static SL_Cstate  basec[1]; 
static SL_quat    baseo[1]; 
static int        get_mse=2;
static int        use_commands = FALSE;
static FILE      *datafp=NULL;
static int        write_data = FALSE;
static int        down_sample = DOWN_SAMPLE_DEFAULT;
static int        filt_data = TRUE;
static int        got_all_joint_data = FALSE;
static int        got_all_base_data = FALSE;
static int        got_all_constraint_data = FALSE;
static int        got_all_contact_force_data = FALSE;
static int        use_floating_base = FALSE;
static int        constraint_estimation_type = PROJECTION;

#define LLSB(x)	((x) & 0xff)		/*!< 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)
#define LONGSWAP(x) ((LLSB(x) << 24) | \
		     (LNLSB(x) << 16)| \
		     (LNMSB(x) << 8) | \
		     (LMSB(x)))

/* global variables */
char   **argv_global;
int      argc_global;
char    *argv_prog_name;
int      servo_enabled = FALSE;
double   servo_time = 0;

/* global functions */

/* local functions */
static int add_to_regression(void);
static int read_file(char *fname);
static int filter_data(void);
static int regress_parameters(void);
static void do_math(SL_endeff *eff);

extern SL_link    links[N_DOFS+1];
 
/*!*****************************************************************************
 *******************************************************************************
\note  main
\date  02/25/92 
   
\remarks 

	entyr program
	
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     argc : standard arguments to give some initial input to the program
 \param[in]     arv  : s.o.

 ******************************************************************************/
int
main(int argc, char **argv)
{
  int   i,j,n;
  int  ans = 0;
  char fname[100]="ATA_ATb.mat";
  FILE *fid;
  
  /* copy the input arguments */

  argc_global    = argc;
  argv_global    = argv;
  argv_prog_name = argv[0];
  
  /* subtract 1 from argv to get rid of the 0-th argument which is the
     function name itself */
  
  argc_global -= 1;

  /* read the configration file names */
  real_robot_flag = TRUE;
  get_int("Which parameter files: RealRobot=1 Simulation=0?",real_robot_flag,&real_robot_flag);
  setRealRobotOptions();

  /* some handy matrices: note that these matrices need to be big enough
     to include the dummy DOFs */
  ATA      = my_matrix(1,(N_DOFS+1)*11,1,(N_DOFS+1)*11);
  ATb      = my_vector(1,(N_DOFS+1)*11);
  beta     = my_vector(1,(N_DOFS+1)*11);
  beta_old = my_vector(1,(N_DOFS+1)*11);

  /* initialize the dynamics calculations */
  if (!init_dynamics())
    exit(-1);

  /* initialize the kinematics calculations */
  init_kinematics();

  /* read the sensor offset to know about joint range values */
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    exit(-1);

  /* clear some matrices and vectors*/
  n_sse = 0;
  for (i=0; i<=N_DOFS_EST; ++i) {
    for (j=1; j<=6; ++j) {
      for (n=1; n<=6; ++n)
	Xinv[i][j][n]=0.0;
      st[i][j]=0.0;
    }
  }

  for (i=1; i<=N_DOFS+2*N_CART; ++i)
    sse[i]=0;

  for (i=0; i<=N_DOFS_EST; ++i)
    for (j=1; j<=11; ++j)
      for (n=1; n<=6; ++n)
	A[i][n][j]=0.0;

  for (i=1; i<=N_DOFS_EST+2*N_CART; ++i)
    for (j=1; j<=11*(N_DOFS_EST+1); ++j)
      K[i][j]=0.0;

  /* compute the parameter estimation matrices to make sure that the program can
     run without data files (the map array needs to be assinged) */
  do_math(endeff);

  /* copy the current RBD parameters into the beta_old vector */
  for (i=0; i<=N_DOFS; ++i) {
    beta_old[i*11+1] = links[i].m;
    beta_old[i*11+2] = links[i].mcm[_X_];
    beta_old[i*11+3] = links[i].mcm[_Y_];
    beta_old[i*11+4] = links[i].mcm[_Z_];
    beta_old[i*11+5] = links[i].inertia[1][1];
    beta_old[i*11+6] = links[i].inertia[1][2];
    beta_old[i*11+7] = links[i].inertia[1][3];
    beta_old[i*11+8] = links[i].inertia[2][2];
    beta_old[i*11+9] = links[i].inertia[2][3];
    beta_old[i*11+10]= links[i].inertia[3][3];
    beta_old[i*11+11]= links[i].vis;
  }

  /* calculate MSE */
  get_int("Calculate MSE & Parameters? Both=2, MSE=1, Parm=0",
	  get_mse,&get_mse);

  /* read the old ATA and ATb matrices? */
  if (!get_int("Read previous ATA and ATb matrices? Yes=1, No=0",ans,&ans))
    exit(-1);

  if (ans==1) {
    if (getFile(fname)) {
      fid = fopen(fname,"r");
      fread_mat(fid,ATA);
      fread_vec(fid,ATb);
      fclose(fid);
    }
  }

  if (!get_int("Use floating base estimation? Yes=1, No=0",
	       use_floating_base,&use_floating_base))
    exit(-1);

  if (use_floating_base) {
    if (!get_int("Which constraint estimation? Q-Project=1 C-Force=2 Base-Only=3",
		 constraint_estimation_type,&constraint_estimation_type))
      exit(-1);
  }

  if (!get_int("Use commands instead of load cells? Yes=1, No=0",
	       use_commands,&use_commands))
    exit(-1);

  if (!get_int("Write regression data? Yes=1, No=0",
	       write_data,&write_data))
    exit(-1);

  if (!get_int("Downsample using which data point?",down_sample,&down_sample))
    exit(-1);

  if (!get_int("Filter data?",filt_data,&filt_data))
    exit(-1);

  if (write_data)
    datafp = fopen("AbData.mat","w");


  /* loop through all the files */
  for (i=1; i<=argc_global; ++i) {

    printf("Processing %s\n",argv_global[i]);
    
    // read file
    if (!read_file(argv_global[i]))
      continue;

    // check whether this file allows for the desired estimation method
    if (use_floating_base) {
      if (constraint_estimation_type == BASE_ONLY) {
	if (! (got_all_base_data && got_all_contact_force_data)) {
	  printf("Missing variables for BASE-ONLY floating-base parameter estimation from this file\n");
	  continue;
	}
      } else {
	if (! (got_all_joint_data && got_all_base_data && got_all_constraint_data)) {
	  printf("Missing variables for floating-base parameter estimation from this file\n");
	  continue;
	}
	if (constraint_estimation_type == CFORCES) {
	  if (! (got_all_contact_force_data)) {
	    printf("Missing variables for floating-base parameter estimation with contact forces from this file\n");
	    continue;
	  }
	}
      }
    } else {
      if (!got_all_joint_data) {
	printf("Missing variables for parameter estimation from this file\n");
	continue;
      }
    }
    
    // filter
    if (filt_data)
      filter_data();

    // add data
    add_to_regression();
    
  }

  if (write_data)
    fclose(datafp);

  /* write out the ATA and ATb */

  fid = fopen(fname,"w");
  if (fid == NULL) {
    printf("Couldn't open file >%s< for write\n",fname);
    exit(-1);
  }
  fwrite_mat(fid,ATA);
  fwrite_vec(fid,ATb);
  fclose(fid);

  if (get_mse != 1)
    regress_parameters();

  return TRUE;
	
}

/*!*****************************************************************************
 *******************************************************************************
  \note  read_file
  \date  June 1999

  \remarks 

  read an MRDPLOT file and sort the variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
read_file(char *fname)
{
  int    j,i,r,k;
  static char string[100];
  FILE * fp;
  int    found = FALSE;
  char **vnames;
  char **units;
  Matrix buff;
  int    aux;

  /* clear the current parameters */
  if (data_pos != NULL) {
    my_free_matrix(data_pos,1,N_DOFS,1,n_rows);
    data_pos = NULL;
  }
  if (data_vel != NULL) {
    my_free_matrix(data_vel,1,N_DOFS,1,n_rows);
    data_vel = NULL;
  } 
  if (data_acc != NULL) {
    my_free_matrix(data_acc,1,N_DOFS,1,n_rows);
    data_acc = NULL;
  }
  if (data_u != NULL) {
    my_free_matrix(data_u,1,N_DOFS,1,n_rows);
    data_u = NULL;
  }
  if (data_bc_pos != NULL) {
    my_free_matrix(data_bc_pos,1,N_CART,1,n_rows);
    data_bc_pos = NULL;
  }
  if (data_bc_vel != NULL) {
    my_free_matrix(data_bc_vel,1,N_CART,1,n_rows);
    data_bc_vel = NULL;
  }
  if (data_bc_acc != NULL) {
    my_free_matrix(data_bc_acc,1,N_CART,1,n_rows);
    data_bc_acc = NULL;
  }
  if (data_bo_q != NULL) {
    my_free_matrix(data_bo_q,1,N_QUAT,1,n_rows);
    data_bo_q = NULL;
  }
  if (data_bo_ad != NULL) {
    my_free_matrix(data_bo_ad,1,N_CART,1,n_rows);
    data_bo_ad = NULL;
  }
  if (data_bo_add != NULL) {
    my_free_matrix(data_bo_add,1,N_CART,1,n_rows);
    data_bo_add = NULL;
  }
  if (data_endeff_c != NULL) {
    my_free_imatrix(data_endeff_c,1,N_ENDEFFS*2*N_CART,1,n_rows);
    data_endeff_c = NULL;
  }
  if (data_endeff_f != NULL) {
    my_free_matrix(data_endeff_f,1,N_ENDEFFS*2*N_CART,1,n_rows);
    data_endeff_f = NULL;
  }

  /* open the file, and parse the parameters */
  buff   = NULL;
  vnames = NULL;
  units  = NULL;
  if (!mrdplot_convert(fname, &buff, &vnames, &units, &sampling_freq, &n_cols, &n_rows))
    return FALSE;

  /* create the pos, vel, acc , uff matrices that define the trajectory */
  data_pos      = my_matrix(1,N_DOFS,1,n_rows);
  data_vel      = my_matrix(1,N_DOFS,1,n_rows);
  data_acc      = my_matrix(1,N_DOFS,1,n_rows);
  data_u        = my_matrix(1,N_DOFS,1,n_rows);
  data_bc_pos   = my_matrix(1,N_CART,1,n_rows);
  data_bc_vel   = my_matrix(1,N_CART,1,n_rows);
  data_bc_acc   = my_matrix(1,N_CART,1,n_rows);
  data_bo_q     = my_matrix(1,N_QUAT,1,n_rows);
  data_bo_ad    = my_matrix(1,N_CART,1,n_rows);
  data_bo_add   = my_matrix(1,N_CART,1,n_rows);
  data_endeff_c = my_imatrix(1,N_ENDEFFS*2*N_CART,1,n_rows);
  data_endeff_f = my_matrix(1,N_ENDEFFS*2*N_CART,1,n_rows);

  printf("Looking for variables ... ");

  got_all_joint_data         = TRUE;
  got_all_base_data          = TRUE;
  got_all_constraint_data    = TRUE;
  got_all_contact_force_data = TRUE;
  
  /* shuffle the matrices */
  for (i=1;i<=N_DOFS;++i) {
    
    sprintf(string, "%s_th",joint_names[i]);
    
    for (j=1;j<=n_cols;++j) {
      if (strcmp(string,vnames[j])==0) {
	
	found = TRUE;
	printf("\n%s ",string);
	
	/* fill the pos matrix using the right column from buffer*/
	for (r=1;r<=n_rows;++r)
	  data_pos[i][r] = buff[r][j];
	
	/* also check for velocity, acceleration, and uff information
	   use the same value of n-vars to fill the remaining matrices*/
	
	sprintf(string, "%s_thd",joint_names[i]);
	for (k=1;k<=n_cols;++k) {
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_vel[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	sprintf(string, "%s_thdd",joint_names[i]);
	for (k=1;k<=n_cols;++k) {
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_acc[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	
	if (use_commands) 
	  sprintf(string, "%s_u",joint_names[i]);
	else
	  sprintf(string, "%s_load",joint_names[i]);
	
	for (k=1;k<=n_cols;++k){
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_u[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	/* assume only one variable of each kind exists*/
	break;
      }
    }
    
    if (j > n_cols)
      got_all_joint_data = FALSE;
    
  }
  
  printf("\n\n");
  printf("-------> Status of joint data = %d\n\n",got_all_joint_data);
  
  /* look for base variables */
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"x"},{"y"},{"z"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_pos[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"xd"},{"yd"},{"zd"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_vel[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"xdd"},{"ydd"},{"zdd"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_acc[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  
  for (i=1; i<=N_QUAT; ++i) {
    char names[][20] = {{""},{"q0"},{"q1"},{"q2"},{"q3"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_q[i][r] = buff[r][j];
	if (i==N_QUAT)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"ad"},{"bd"},{"gd"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_ad[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"add"},{"bdd"},{"gdd"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_add[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }

  printf("\n");
  printf("-------> Status of base data = %d\n\n",got_all_base_data);

  for (i=1; i<=N_ENDEFFS; ++i) {
    char names[][20] = {{""},{"x"},{"y"},{"z"},{"a"},{"b"},{"g"}};

    for (k=1; k<=2*N_CART; ++k) {
      sprintf(string, "%s_cons_%s",cart_names[i],names[k]);
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  printf("%s ",string);
	  for (r=1;r<=n_rows;++r)
	    data_endeff_c[(i-1)*2*N_CART+k][r] = buff[r][j];
	  if (k==2*N_CART)
	    printf("\n");
	  break;
	}
      }
      if (j > n_cols)
	got_all_constraint_data = FALSE;
    }
  }

  printf("\n");
  printf("-------> Status of constraint data = %d\n\n",got_all_constraint_data);

  for (i=1; i<=N_ENDEFFS; ++i) {
    char names[][20] = {{""},{"cfx"},{"cfy"},{"cfz"},{"cta"},{"ctb"},{"ctg"}};

    for (k=1; k<=2*N_CART; ++k) {
      sprintf(string, "%s_%s",cart_names[i],names[k]);
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  printf("%s ",string);
	  for (r=1;r<=n_rows;++r)
	    data_endeff_f[(i-1)*2*N_CART+k][r] = buff[r][j];
	  if (k==2*N_CART)
	    printf("\n");
	  break;
	}
      }
      if (j > n_cols)
	got_all_contact_force_data = FALSE;
    }
  }

  printf("\n");
  printf("-------> Status of contact force data = %d\n\n",got_all_contact_force_data);

  printf("\n -------- Read %d rows with %d columns ---------\n",n_rows,n_cols);
  
  
  /* free up memory by deallocating resources */
  my_free_matrix (buff,1,n_rows,1,n_cols);
  for (i=1;i<=n_cols;++i){
    free(vnames[i] ); 
    free(units[i]) ;
  }
  
  free(units);
  free(vnames);
  
  return found;
  
}

/*!*****************************************************************************
 *******************************************************************************
  \note  filter_data
  \date  June 1999

  \remarks 

  differentiate and filter

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
filter_data(void)
{
  int j,i,r;

  for (i=1; i<=N_DOFS; ++i) {
    diff( data_vel[i], n_rows, 1./sampling_freq, data_acc[i]);
    filtfilt(data_acc[i],n_rows,10,data_acc[i]);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  add_to_regression
  \date  June 1999

  \remarks 

  recursively add data to the regression analysis

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
add_to_regression(void)
{
  int    j,i,m,n,r,s;
  int    count=0;
  int    flag;
  float  aux;
  int    n_cons;
  int    use_constraint;
  int    nr_K = 0;

  MY_MATRIX(Jc,1,2*N_CART*N_ENDEFFS,1,N_DOFS+2*N_CART);
  MY_MATRIX(Q,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_MATRIX(Qu,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_MATRIX(R,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_VECTOR(sv,1,N_DOFS+2*N_CART);
  MY_MATRIX(Kp,1,N_DOFS+2*N_CART,1,(N_DOFS+1)*11);
  MY_VECTOR(Yp,1,N_DOFS+2*N_CART);
  MY_VECTOR(cf,1,N_CART*2*N_ENDEFFS);

  printf("\nAdd data to regression ");

  for (i=1+100; i<=n_rows-100; i+=down_sample) {

    /* if data is too close to the joint range, ignore the data */
    flag = FALSE;
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      if (data_pos[j][i] > joint_range[j][MAX_THETA]-.05 ||
	  data_pos[j][i] < joint_range[j][MIN_THETA]+.05) {
	flag = TRUE;
      }
    }
    if (flag)
      continue;

    if (++count%1000==0) {
      printf("%d",count);
      fflush(stdout);
    } else if (count%100==0) {
      printf(".");
      fflush(stdout);
    }

    for (j=1; j<=N_DOFS; ++j) {
      invdyn_state[j].th   = state[j].th   = data_pos[j][i];
      invdyn_state[j].thd  = state[j].thd  = data_vel[j][i];
      invdyn_state[j].thdd = state[j].thdd = data_acc[j][i];
      state[j].u           = data_u[j][i];
      invdyn_state[j].uff  = 0;
   }

    for (j=1; j<=N_CART; ++j) {
      basec[0].x[j]   = data_bc_pos[j][i];
      basec[0].xd[j]  = data_bc_vel[j][i];
      basec[0].xdd[j] = data_bc_acc[j][i];
    }

    aux = 0;
    for (j=1; j<=N_QUAT; ++j) {
      baseo[0].q[j]   = data_bo_q[j][i];
      aux += baseo[0].q[j];
    }
    if (aux == 0) // not orientation in data
      baseo[0].q[_Q0_] = 1.0;

    for (j=1; j<=N_CART; ++j) {
      baseo[0].ad[j]  = data_bo_ad[j][i];
      baseo[0].add[j] = data_bo_add[j][i];
    }

    n_cons = 0;
    for (j=1; j<=N_ENDEFFS; ++j) {
      for (m=1; m<=2*N_CART; ++m) {
	endeff[j].c[m] = data_endeff_c[(j-1)*2*N_CART+m][i];
	n_cons += endeff[j].c[m];
      }
    }
    
    if (n_cons > 0) {
      use_constraint = TRUE;
    } else {
      use_constraint = FALSE;
    }

    for (j=1; j<=N_ENDEFFS*2*N_CART; ++j)
	cf[j] = data_endeff_f[j][i];
    
    /* compute the parameter estimation matrices */
    do_math(endeff);

    /* add viscous friction terms in the K-matrix: according to the model, viscous friction
       acts only idenpendently on each DOF */

    for (j=1; j<=N_DOFS; ++j) 
      K[map[j]][(map[j]+1)*11] = state[j].thd;

    /* ======================================================================================== */
    /* zero main regression matrices */
    vec_zero(Yp);
    mat_zero(Kp);
    
    /* project the K matrix if needed */
    if (use_constraint) {
      int nr,nc;

      computeConstraintJacobian(state, basec, baseo, endeff,Jc,&nr,&nc); 

      if (constraint_estimation_type == PROJECTION) {
	
	computeQR(Jc, nr, nc, Q, Qu, R, sv);
	
	// create a range space projection
	mat_mult_outer_size(Qu,N_DOFS+2*N_CART,N_DOFS+2*N_CART-n_cons,Qu);
	nr_K = N_DOFS+2*N_CART;
	
	// run the K matrix through this projection and remap
	for (j=1; j<=nr_K; ++j) {
	  for (m=0; m<=N_DOFS; ++m) {
	    for (r=1; r<=11; ++r ) {
	      
	      Kp[j][m*11+r] = 0.0;
	      
	      for (n=1; n<=N_DOFS+2*N_CART; ++n) {
		int map_n;
		
		if (n > N_DOFS)
		  map_n = N_DOFS_EST+(n-N_DOFS);
		else
		  map_n = map[n];
		
		Kp[j][m*11+r] += Qu[n][j]*K[map_n][map[m]*11+r];
	      }
	    }
	  }
	  
	  Yp[j] = 0.0;
	  for (n=1; n<=N_DOFS+2*N_CART; ++n) {
	    int map_n;
	    
	    if (n > N_DOFS)
	      map_n = N_DOFS_EST+(n-N_DOFS);
	    else
	      map_n = map[n];
	    
	    Yp[j] += Qu[n][j]*Y[map_n];
	  }
	  
	}

      } else if (constraint_estimation_type == CFORCES) {
	double Yc[2*N_CART+N_DOFS+1];

	for (j=1; j<=N_DOFS+6; ++j) {
	  Yc[j] = 0.0;
	  for (m=1; m<=2*N_CART*N_ENDEFFS; ++m)
	    Yc[j] += cf[m]*Jc[m][j];
	}

	nr_K = N_DOFS+2*N_CART;
	
	for (j=1; j<=nr_K; ++j) {
	  int map_j;
	  
	  if (j > N_DOFS)
	    map_j = N_DOFS_EST+(j-N_DOFS);
	  else
	    map_j = map[j];
	  
	  for (m=0; m<=N_DOFS; ++m)
	    for (r=1; r<=11; ++r)
	      Kp[j][m*11+r] = K[map_j][map[m]*11+r];
	  
	  Yp[j] = Y[map_j] + Yc[j];

	}

      } else if (constraint_estimation_type == BASE_ONLY) {
	double Yc[2*N_CART+1];

	for (j=1; j<=2*N_CART; ++j) {
	  Yc[j] = 0.0;
	  for (m=1; m<=2*N_CART*N_ENDEFFS; ++m)
	    Yc[j] += cf[m]*Jc[m][N_DOFS+j];
	}

	nr_K = 2*N_CART;
	
	for (j=1; j<=nr_K; ++j) {
	  int map_j;
	  
	  map_j = N_DOFS_EST+j;
	  
	  for (m=0; m<=N_DOFS; ++m)
	    for (r=1; r<=11; ++r)
	      Kp[j][m*11+r] = K[map_j][map[m]*11+r];
	  
	  Yp[j] = Y[map_j] + Yc[j];

	}


      }

    } else { // no constraints

      nr_K = N_DOFS+2*N_CART;

      for (j=1; j<=nr_K; ++j) {
	int map_j;
	
	if (j > N_DOFS)
	  map_j = N_DOFS_EST+(j-N_DOFS);
	else
	  map_j = map[j];

	for (m=0; m<=N_DOFS; ++m)
	  for (r=1; r<=11; ++r)
	    Kp[j][m*11+r] = K[map_j][map[m]*11+r];

	Yp[j] = Y[map_j];

      }
    }

    if (count == 1 && 0) {
      printf("nr_K=%d\n",nr_K);
      print_mat_size("Jc",Jc,n_cons,N_DOFS+6);
      print_mat("Q",Q);
      print_mat_size("Qu",Qu,N_DOFS+6,nr_K);
      print_mat_size("Kp",Kp,nr_K,(N_DOFS+1)*11);
      print_vec("beta_old",beta_old);
    }

    /*
    {
      FILE *fp = fopen("mist","w");
      for (j=1; j<=N_DOFS_EST+6; ++j) {
	for (m=1; m<=(N_DOFS_EST+1)*11; ++m) {
	  fprintf(fp,"% 5.2f ",K[j][m]);
      }
	fprintf(fp,"\n");
      }
      fprintf(fp,"\n");
      fclose(fp);
      getchar();
    }
    */

    /* predict the outcome with old parameters and update sse */
    if (get_mse != 0) {
      double pred[N_DOFS+2*N_CART+1];

      ++n_sse;

      for (j=1; j<=nr_K; ++j) {
	pred[j] = 0.0;
	for (m=1; m<=(N_DOFS+1)*11; ++m)
	  pred[j] += Kp[j][m]*beta_old[m];
      }

      for (j=1; j<=N_DOFS+2*N_CART; ++j) {
	sse[j] += sqr(Yp[j]-pred[j]);

      }

    }

    if (get_mse == 1)
      continue;
    
    /* add data to regression -------------------------------- */
    for (j=1; j<=nr_K; ++j) { // these are the # rows in the K matrix
      
      for (m=1; m<=(N_DOFS+1)*11; ++m)
	for (n=m; n<=(N_DOFS+1)*11; ++n)
	  ATA[m][n] += Kp[j][m]*Kp[j][n];
      
      for (m=1; m<=(N_DOFS+1)*11; ++m) {
	ATb[m] += Kp[j][m]*Yp[j];
	aux = Kp[j][m];
	if (write_data) {
#ifdef BYTESWAP
	  aux = byteswap_float(aux);
#endif
	  if (fwrite(&aux,sizeof(float),1,datafp)!= 1) {
	    printf( "cannot fwrite data.\n" );
	    exit(-1);
	  }
	}
      }

      if (write_data) {
	aux = Yp[j];
#ifdef BYTESWAP
	aux = byteswap_float(aux);
#endif
	if (fwrite(&aux,sizeof(float),1,datafp)!= 1) {
	  printf( "cannot fwrite data.\n" );
	  exit(-1);
	}
      }
    }
    
  }
      
  /* ATA is symmetric */
  for (m=1; m<=(N_DOFS+1)*11; ++m) {
    for (n=m; n<=(N_DOFS+1)*11; ++n) {
      ATA[n][m] = ATA[m][n];
    }
  }

  printf("done\n");

  /* print the current SSE */
  
  printf("\nMSE (STD) Statistics (based on %d data points):\n",count);

  if (constraint_estimation_type != BASE_ONLY) {

    for (j=1; j<=N_DOFS; j=j+3) {
      printf("%6s: % 7.2f (% 7.2f)   ",
	     joint_names[j],sse[j]/(double)n_sse,sqrt(sse[j]/(double)n_sse));
      if (j+1 > N_DOFS)
	break;
      printf("%6s: % 7.2f (% 7.2f)   ",
	     joint_names[j+1],sse[j+1]/(double)n_sse,sqrt(sse[j+1]/(double)n_sse));
      if (j+2 > N_DOFS)
	break;
      printf("%6s: % 7.2f (% 7.2f)\n",
	     joint_names[j+2],sse[j+2]/(double)n_sse,sqrt(sse[j+2]/(double)n_sse));
    }
    printf("\n");
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fx",sse[N_DOFS+1]/(double)n_sse,sqrt(sse[N_DOFS+1]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fy",sse[N_DOFS+2]/(double)n_sse,sqrt(sse[N_DOFS+2]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_fz",sse[N_DOFS+3]/(double)n_sse,sqrt(sse[N_DOFS+3]/(double)n_sse));
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_tx",sse[N_DOFS+4]/(double)n_sse,sqrt(sse[N_DOFS+4]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_ty",sse[N_DOFS+5]/(double)n_sse,sqrt(sse[N_DOFS+5]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_tz",sse[N_DOFS+6]/(double)n_sse,sqrt(sse[N_DOFS+6]/(double)n_sse));
    printf("\n");

  } else {

    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fx",sse[1]/(double)n_sse,sqrt(sse[1]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fy",sse[2]/(double)n_sse,sqrt(sse[2]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_fz",sse[3]/(double)n_sse,sqrt(sse[3]/(double)n_sse));
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_tx",sse[4]/(double)n_sse,sqrt(sse[4]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_ty",sse[5]/(double)n_sse,sqrt(sse[5]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_tz",sse[6]/(double)n_sse,sqrt(sse[6]/(double)n_sse));
    printf("\n");


  }
    
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  regress_parameters
  \date  June 1999

  \remarks 

  performs a SVD regression

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
regress_parameters(void)
{
  int j,i,m,n;
  FILE *fp;
  double svdr = 0.01;
  double svdt = 0.01;

  printf("\nStarting SVD ...");
  
  correct_command_svd(ATA, ATb, svdr, svdt, beta, (N_DOFS+1-N_DOFS_EST_SKIP)*11);
  
  printf("done\n");

  /* write the results to a file */
  fp = fopen("parameters.est","w");
  if (fp==NULL) {
    printf("Couldn't open parameter.est for write\n");
    exit(-1);
  }

  for (i=0; i<=N_DOFS; ++i) {
    fprintf(fp,"%s ",joint_names[i]);

    /* make the file look nice (Jorg, Sun Nov 21 20:31:20 PST 1999) */
    for (j=strlen(joint_names[i]); j<5; j++)
     fprintf(fp," ");

    for (m=1; m<=11; ++m) {
      fprintf(fp,"%9f ",beta[i*11+m]);
    }
    fprintf(fp,"\n");
  }

  fclose(fp);

  return TRUE;

}



/*!*****************************************************************************
 *******************************************************************************
  \note  do_math
  \date  June 1999

  \remarks 

  includes the math for parameter estimation -- it is useful to have this
  in a function for faster compilation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void
do_math(SL_endeff *eff) {

#include "PE_math.h"

}
