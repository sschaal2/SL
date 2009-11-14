/*!=============================================================================
  ==============================================================================

  \file    SL_oscilloscope.h

  \author  Peter Pastor, Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  declarations needed by SL_oscilloscope.c

  ============================================================================*/

#ifndef _SL_oscilloscope_
#define _SL_oscilloscope_

// left-overs from dta.h -- to be removed
#define DT1401       1
#define PMC16LCAIO   2


#ifdef __cplusplus
extern "C" {
#endif
  
  // global functions 

  void 
  dta(int hardware, int board, int channel, int param);
  
  int
  initialize_ad(int hardware);

  // external variables


#ifdef __cplusplus
}
#endif

#endif
