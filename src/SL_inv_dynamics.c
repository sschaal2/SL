/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_inv_dynamics.c

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks

  the inverse dynamics function and several other dynamics functions

  Note: the body was moved to a header file as this file needs robot specific
        header files.

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "mdefs.h"

// global variables
int    freeze_base               = FALSE;
double freeze_base_pos[N_CART+1] = {0.0,0.0,0.0,0.0};
double freeze_base_quat[N_QUAT+1] = {0.0,1.0,0.0,0.0,0.0};

/*!*****************************************************************************
*******************************************************************************
\note  init_dynamics
\date  Sept 2010
\remarks 

initializes all inverse dynamics meethods

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
init_dynamics( void )

{
  int    i, j,n;
  FILE  *in;
  char   string[100];
  double quat[N_QUAT+1];
  double pos[N_CART+1];
  double aux;

  // read link parameters
  if (!read_link_parameters(config_files[LINKPARAMETERS]))
    return FALSE;

  // the the default endeffector parameters
  setDefaultEndeffector();

  // initialize the base variables
  bzero((void *)&base_state,sizeof(base_state));
  bzero((void *)&base_orient,sizeof(base_orient));
  base_orient.q[_Q0_] = 1.0;

  if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_pos",N_CART,pos)) {
    for (i=1; i<=N_CART; ++i)
      freeze_base_pos[i] = base_state.x[i] = pos[i];
  }
  
  if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_quat",N_QUAT,quat)) {
    aux = 0.0;
    for (i=1; i<=N_QUAT; ++i)
      aux += sqr(quat[i]);
    aux = sqrt(aux);

    for (i=1; i<=N_QUAT; ++i) 
      freeze_base_quat[i] = base_orient.q[i] = quat[i]/(aux + 1.e-10);
  }
  

  return TRUE;
  
}    

// add the robot specific code
#include "SL_inv_dynamics_body.h"

