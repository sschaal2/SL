/*!=============================================================================
  ==============================================================================

  \file    SL_dynamics.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_dynamics.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_dynamics_
#define _SL_dynamics_

/* shared functions */


#ifdef __cplusplus
extern "C" {
#endif

  int  init_dynamics(void);
  void setDefaultEndeffector(void);
  
  void SL_ForDyn(SL_Jstate *lstate,SL_Cstate *cbase,
		 SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_ForDynArt(SL_Jstate *lstate,SL_Cstate *cbase,
		    SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_ForDynComp(SL_Jstate *lstate,SL_Cstate *cbase,
		     SL_quat *obase, SL_uext *ux, SL_endeff *leff,
		     Matrix rbdM, Vector rbdCG);
  void SL_ForwardDynamics(SL_Jstate *lstate,SL_Cstate *cbase,
			  SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_InverseDynamics(SL_Jstate *cstate,SL_DJstate *state,SL_endeff *endeff);
  void SL_InverseDynamicsArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_Cstate *cbase,
			     SL_quat *obase, SL_uext *ux, SL_endeff *leff);

  void SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		 SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynNE(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		   SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		    SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynNEBase(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		       SL_Cstate *cbase, SL_quat *obase, double *fbase);

  // external variables 
  extern int    freeze_base;
  extern double freeze_base_pos[];
  extern double freeze_base_quat[];
  extern const int floating_base_flag;

  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_dynamics_ */
