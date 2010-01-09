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

#ifdef __cplusplus
extern "C" {
#endif
  
  // global functions 
  int
  setOsc(int channel, double pval);

  void
  setD2AFunctioni(int (*fptr)(int,double));


  // external variables


#ifdef __cplusplus
}
#endif

#endif