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

/* external variables */

extern Matrix rbdInertiaMatrix;
extern Vector rbdCplusGVector;

extern int    freeze_base;
extern double freeze_base_pos[];
extern double freeze_base_quat[];

/* shared functions */

#ifdef __cplusplus
extern "C" {
#endif

int  init_dynamics(void);
void setDefaultEndeffector(void);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_dynamics_ */
