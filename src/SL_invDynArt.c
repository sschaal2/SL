/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_invDynArt.c

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks

  articulated body inverse dynamics, suitable for floating base inverse
  dynamics, but much more expensive to compute that normal inverse
  dynamics algorithms.

  Note: the body was moved to a header file as this file needs robot specific
        header files.

  ============================================================================*/

#include "SL_invDynArt_body.h"
