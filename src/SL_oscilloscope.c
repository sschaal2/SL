/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_oscilloscope.c

  \author  Stefan Schaal
  \date    Nov 2009

  ==============================================================================
  \remarks

  Manages visualization of data traces on an oscilloscope, which can be a
  real physical oscilloscope or a graphics simulation of an oscilloscope.
  Additionally, for graphics oscilloscope, other data traces can be visualized.
  
  ============================================================================*/

// system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_oscilloscope.h"

// global variabes

// local variabes
static int (*d2a_function)(int,double) = NULL;

// global functions

// local functions

/*!*****************************************************************************
*******************************************************************************
\note  setOsc
\date  Nov. 2009
 
\remarks 
 
set the oscilloscope on a particular channel to a particiular value,
expressed as a percentage value of the max range of the D/A converter.
Using percentage values frees the user from knowing the resolution of the
D/A converter.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   channel : which channel to use
\param[in]   pval    : [0 to 100] percentage value of max range to be displayed.
 
******************************************************************************/
int
setOsc(int channel, double pval)
{
  int rc;

  // if the user provide a special oscilloscope function
  if (d2a_function != NULL) {
    return (*d2a_function)(channel,pval);
  }

  // the graphics oscilloscope is the default -- just add the data
  // and a timestamp to the appropriate shared memory structure

  // to be done yet


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  setD2AFunction
\date  Nov. 2009
 
\remarks 
 
Provides a function pointer which is used to display D2A debugging signals.
Usually, this allows re-routing the D2A debugging signals to a D/A signal
converter, which is part of some DAQ board.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   fptr : function pointer
 
******************************************************************************/
void
setD2AFunctioni(int (*fptr)(int,double))
{
  int rc;

  d2a_function = fptr;

}


