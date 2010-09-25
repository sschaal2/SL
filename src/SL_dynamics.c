/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_dynamics.c

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks

  shared function used for forward and inverse dynamics

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "utility_macros.h"

// global variables
int    freeze_base               = FALSE;
double freeze_base_pos[N_CART+1] = {0.0,0.0,0.0,0.0};
double freeze_base_quat[N_QUAT+1] = {0.0,1.0,0.0,0.0,0.0};
#include "Floating_Base.h"

// local variables
static int forward_dynamics_comp_flag = FALSE;

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

  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"use_comp_inertia_fordyn",&n)) {
    if (n == 1)
      forward_dynamics_comp_flag = TRUE;
    else
      forward_dynamics_comp_flag = FALSE;
  }
  
  return TRUE;
  
}    

/*!*****************************************************************************
*******************************************************************************
\note  SL_InvDyn
\date  Sept 2010

\remarks 

Standard Newton Euler inverse dynamics, which switches automatically between
floating base and fixed base robots

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters
\param[in]     cbase   : the position state of the base
\param[in]     obase   : the orientational state of the base

Returns:
The appropriate feedforward torques are added in the uff component of the lstate
structure.

******************************************************************************/
void 
SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
	  SL_Cstate *cbase, SL_quat *obase)
{

  if (floating_base_flag)
    SL_InvDynArt(cstate, lstate, leff, cbase, obase);
  else
    SL_InvDynNE(cstate, lstate, leff, cbase, obase);
}

/*!*****************************************************************************
*******************************************************************************
\note  SL_InverseDynamics
\date  Sept 2010

\remarks 

depricated function for inverse dynamics computation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in,out] cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters

******************************************************************************/
void 
SL_InverseDynamics(SL_Jstate *cstate,SL_DJstate *lstate,SL_endeff *leff)
{
  static int firsttime = TRUE;
  static int counter = 0;

  if (firsttime) {
    printf("SL_InverseDynamics is depricated -- use SL_InvDyn instead\n");
    if (++counter > 10)
      firsttime = FALSE;
  }

  SL_InvDyn(cstate, lstate, leff, &base_state, &base_orient);

}

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
void 
SL_ForwardDynamics(SL_Jstate *lstate,SL_Cstate *cbase,
		   SL_quat *obase, SL_uext *ux, SL_endeff *leff)
{
  static int firsttime = TRUE;
  static int counter = 0;

  if (firsttime) {
    printf("SL_ForwardDynamics is depricated -- use SL_ForDyn, SL_ForDynArt, or SL_ForDynComp instead\n");
    if (++counter > 10)
      firsttime = FALSE;
  }

  SL_ForDyn(lstate, cbase, obase, ux, leff);

}

/*!*****************************************************************************
 *******************************************************************************
\note  SL_ForDyn
\date  Sept 2010
   
\remarks 

computes the forward dynamics accelerations according to the default method,
which can be composite inertia or articulated body inertia.

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
void 
SL_ForDyn(SL_Jstate *lstate,SL_Cstate *cbase,
	  SL_quat *obase, SL_uext *ux, SL_endeff *leff)

{
  MY_MATRIX(rbdM,1,n_dofs+6,1,n_dofs+6);
  MY_VECTOR(rbdCG,1,n_dofs+6);

  if (forward_dynamics_comp_flag)
    SL_ForDynComp(lstate, cbase, obase, ux, leff, rbdM, rbdCG);
  else 
    SL_ForDynArt(lstate, cbase, obase, ux, leff);

}
