/*!=============================================================================
  ==============================================================================

  \file    SL_inv_dynamics_art_body.h

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  articulated body inverse dynamics, suitable for floating base inverse
  dynamics, but much more expensive to compute that normal inverse
  dynamics algorithms.

  Note: this file was converted to a header file as it requires to include
        robot specific header files, such that it cannot be added to a SL
	library. SL_inv_dynamics_art.c is just an empty file which includes
        this header.

  ============================================================================*/

/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "SL.h"
#include "SL_user.h"
#include "mdefs.h"
#include "SL_dynamics.h"
#include "SL_integrate.h"

/* global variables */ 

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  SL_InverseDynamicsArt
\date  June 1999
   
\remarks 

        computes the torques for all joints, and the acceleration of the
        base

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state   : the state containing th, thd, thdd, and receiving the
                  appropriate u
 \param[in,out] cbase   : the position state of the base
 \param[in,out] obase   : the orientational state of the base
 \param[in]     ux      : the external forces acting on each joint, in local 
                  (link) coordinates, e.g., as measured by a local force
                  or torque cell
 \param[in]     endeff  : the endeffector parameters

 ******************************************************************************/
#include "InvDynArt_declare.h"

static SL_DJstate  state[N_DOFS+1];
static SL_endeff  *eff;
static SL_Cstate  *basec;
static SL_quat    *baseo;
static SL_uext    *uex;

void 
SL_InverseDynamicsArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_Cstate *cbase,
		      SL_quat *obase, SL_uext *ux, SL_endeff *leff)

{
  int i,j;
  static int firsttime = TRUE; 

  /* initializations */
  if (firsttime) {
    firsttime = FALSE;
  }

  /* the following assignments make the arguments global variables */ 

  /* create a mixed desired/current state for proper inverse dynamics */
  for (i=1; i<=N_DOFS; ++i) {
    state[i]     = lstate[i];
    if (cstate != NULL) {
      state[i].th  = cstate[i].th;
      state[i].thd = cstate[i].thd;
    }
  }

  eff    = leff;
  basec  = cbase;
  baseo  = obase;
  uex    = ux;
  
#include "InvDynArt_math.h"

  /* add the friction term */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += links[i].vis*state[i].thd;
    if (i > N_DOFS-N_DOFS_EST_SKIP)
      state[i].uff = 0.0;
    lstate[i].uff = state[i].uff;
  }

} 


#include "InvDynArt_functions.h"
