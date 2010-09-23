/*!=============================================================================
  ==============================================================================

  \file    SL_inv_dynamics_body.h

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  the inverse dynamics function and several other dynamics functions

  Note: this file was converted to a header file as it requires to include
        robot specific header files, such that it cannot be added to a SL
	library. SL_inv_dynamics.c is just an empty file which includes
        this header.


  ============================================================================*/

/* global variables */

/* local variables */
#include "InvDyn_declare.h"

static SL_DJstate state[N_DOFS+1];
static SL_endeff *eff; 

/* global functions */

/* local functions */
/* external variables */


/*!*****************************************************************************
 *******************************************************************************
\note  SL_InverseDynamics
\date  June 1999
   
\remarks 

        computes the inverse dynamics torques

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] cstate  : the current state (pass NULL to use only desired state)
 \param[in,out] lstate  : the desired state
 \param[in]     endeff  : the endeffector parameters

 ******************************************************************************/
void 
SL_InverseDynamics(SL_Jstate *cstate,SL_DJstate *lstate,SL_endeff *leff)

{
  int i;

  /* this makes the arguments global variables */
  eff    = leff;

  /* create a mixed desired/current state for proper inverse dynamics */
  for (i=1; i<=N_DOFS; ++i) {
    state[i]     = lstate[i];
    if (cstate != NULL) {
      state[i].th  = cstate[i].th;
      state[i].thd = cstate[i].thd;
    }
  }

#include "InvDyn_math.h"

  /* add the friction term */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += links[i].vis*state[i].thd;
    if (i > N_DOFS-N_DOFS_EST_SKIP)
      state[i].uff = 0.0;
    lstate[i].uff = state[i].uff;
  }
}

#include "InvDyn_functions.h"
