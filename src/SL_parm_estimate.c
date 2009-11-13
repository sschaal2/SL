/*!=============================================================================
  ==============================================================================

  \ingroup SLparameterEstimation

  \file    SL_parm_estimate.c

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  estimates the inverse dynamics parameters
      
  July 2009: add base link system for estimation

  Note: the body was moved to a header file as this file needs robot specific
        header files.

  ============================================================================*/

#include "SL_parm_estimate_body.h"
