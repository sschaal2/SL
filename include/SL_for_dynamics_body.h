/*!=============================================================================
  ==============================================================================

  \file    SL_for_dynamics_body.h

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  the forward dynamics function

  Note: this file was converted to a header file as it requires to include
        robot specific header files, such that it cannot be added to a SL
	library. SL_for_dynamics.c is just an empty file which includes
        this header.

  ============================================================================*/

/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_dynamics.h"
#include "SL_integrate.h"
#include "mdefs.h"


/* global variables */ 
Matrix rbdInertiaMatrix;
Vector rbdCplusGVector;

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  SL_ForwardDynamics
\date  June 1999
   
\remarks 

        computes the forward dynamics accelerations

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state   : the state containing th, thd, thdd, and receiving the
                          appropriate u
 \param[in,out] cbase   : the position state of the base
 \param[in,out] obase   : the orientational state of the base
 \param[in]     ux      : the external forces acting on each joint, 
                          in world coordinates, e.g., as computed from contact 
                          forces
 \param[in]     endeff  : the endeffector parameters

 ******************************************************************************/
#include "ForDyn_declare.h"

static SL_Jstate  *state;
static SL_endeff  *eff;
static SL_Cstate  *basec;
static SL_quat    *baseo;
static SL_uext    *uex;

void 
SL_ForwardDynamics(SL_Jstate *lstate,SL_Cstate *cbase,
		   SL_quat *obase, SL_uext *ux, SL_endeff *leff)

{
  int i,j;
  static int firsttime = TRUE; 
  static double **Hmat; 
  static double  *cvec; 
  static double  *ucvec; 
  double fbase[2*N_CART+1];  
  
  /* this makes the arguments global variables */ 
  state  = lstate;
  eff    = leff;
  basec  = cbase;
  baseo  = obase;
  uex    = ux;
  
  if (firsttime) {
    firsttime = FALSE;
    rbdInertiaMatrix = Hmat = my_matrix(1,N_DOFS+6,1,N_DOFS+6);
    rbdCplusGVector  = cvec = my_vector(1,N_DOFS+6);
    ucvec   = my_vector(1,N_DOFS+6);
    for (i=1; i<=N_DOFS+6; ++i) {
      cvec[i]=0.0;
      ucvec[i]=0.0;
      for (j=1; j<=N_DOFS+6; ++j)
	 Hmat[i][j] = 0.0;
    } 
  }
  
  /* subtract the friction term temporarily */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].u -= links[i].vis*state[i].thd;
  }

#include "ForDyn_math.h"

  /* add back the friction term */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].u += links[i].vis*state[i].thd;
  }

} 


#include "ForDyn_functions.h"
