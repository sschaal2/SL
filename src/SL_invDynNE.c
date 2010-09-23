/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_invdynNE.c

  \author  Stefan Schaal
  \date    Sept 2010

  ==============================================================================
  \remarks

  Newton Euler inverse dynamics for fixed base robotic systems. Two versions
  are implemented, the standard NE algorithm with base state velocity and
  accelerations equal to zero, and a special version that takes the base 
  velocity and acceleration into account. The latter treats the fixed base
  robot as if the base were full actuated as well, and thus requires a desired
  acceleration for the base as input. This special version returns the commands
  needed at the DOFs and base to realize the given acceleration.


  Note: the body was moved to a header file as this file needs robot specific
        header files.

  ============================================================================*/

#include "SL_invDynNE_body.h"
