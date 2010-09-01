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
#include "SL_shared_memory.h"
#include "SL_common.h"

// defines

// global variabes

// local variabes
static int (*d2a_function)(int,double) = NULL;

// data collection for oscilloscope
static SL_oscEntry   oscbuf[OSC_BUFFER_SIZE+1];        //<! the ring buffer of all entries
static int           current_osc_index = 1;            //<! index of latest entry
static int           n_osc_entries = 0;                //<! number of entries in buffer
static int           osc_enabled = FALSE;              //<! is the oscilloscope active?

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
#define TIME_OUT_NS 100000
int
setOsc(int channel, double pval)
{
  int    rc;
  double ts;
  char   string[40];
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    if (read_parameter_pool_int(config_files[PARAMETERPOOL],"osc_enabled", &rc))
      osc_enabled = macro_sign(abs(rc));
  }

  // if the user provide a special oscilloscope function
  if (d2a_function != NULL) {
    if (semTake(sm_oscilloscope_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
      return FALSE;
    } else {
      rc = (*d2a_function)(channel,pval);
      semGive(sm_oscilloscope_sem);
      return rc;
    }
  }

  if (!osc_enabled)
    return TRUE;

  // the graphics oscilloscope is the default -- just add the data
  // in the appropriate structure in memory

#ifdef __XENO__
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC,&t);
  ts = (double) t.tv_sec + ((double)t.tv_nsec)/1.e9;
#else
  struct timeval t;
  gettimeofday(&t,NULL);
  ts = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
#endif

  sprintf(string,"D2A_%s",servo_name);
  addEntryOscBuffer(string, pval, ts, 0);

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
setD2AFunction(int (*fptr)(int,double))
{
  int rc;

  d2a_function = fptr;

}

/*!*****************************************************************************
*******************************************************************************
\note  addEntryOscBuffer
\date  Aug 2010
 
\remarks 
 
Adds an entry to the oscilloscope ring buffer. As this is a ringer buffer,
the oldest value is overwritten if the buffer size is reached
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   name : name of entry
\param[in]      v : value of entry
\param[in]     ts : time stamp of entry (in seconds)
\param[in]    pID : plot ID, i.e., in which plot his is visualized
 
******************************************************************************/
void
addEntryOscBuffer(char *name, double v, double ts, int pID)
{

  if (!osc_enabled) 
    return;

  if (++current_osc_index > OSC_BUFFER_SIZE)
    current_osc_index = 1;

  strncpy(oscbuf[current_osc_index].name,name,sizeof(oscbuf[current_osc_index].name)-1);
  oscbuf[current_osc_index].v       = (float) v;
  oscbuf[current_osc_index].ts      = ts;
  oscbuf[current_osc_index].plotID  = pID;

  if (++n_osc_entries > OSC_BUFFER_SIZE) {
    n_osc_entries = OSC_BUFFER_SIZE;
  }

}

/*!****************************************************************************
******************************************************************************
\note  sendOscilloscopeData
\date  Aug. 2010

\remarks

copies all local oscilloscope data to shared memory structure

*****************************************************************************
Function Parameters: [in]=input,[out]=output

none

*****************************************************************************/
#define TIME_OUT_SHORT_NS   10000  //!< time out in nano seconds
void 
sendOscilloscopeData(void)
{
  
  int i,j,r;
  int count;
  
  if (!osc_enabled) 
    return;

  // try to take semaphore with very short time-out -- we don't care if we 
  // cannot copy the data to shared memory every time, as this is not a time 
  // critical operation
  if (semTake(sm_oscilloscope_sem,ns2ticks(TIME_OUT_SHORT_NS)) == ERROR)
    return;

  // copy first-in-first-out data into shared memory as long as there is enough
  // space
  if (sm_oscilloscope->n_entries+n_osc_entries > OSC_SM_BUFFER_SIZE) { // cannot copy all
    count = OSC_SM_BUFFER_SIZE - sm_oscilloscope->n_entries;
  } else 
    count = n_osc_entries;

  j = sm_oscilloscope->n_entries;
  for (i=1; i<=count; ++i) {
    r = current_osc_index - n_osc_entries + i;
    if ( r > OSC_BUFFER_SIZE)
      r -= OSC_BUFFER_SIZE;
    if ( r < 1)
      r += OSC_BUFFER_SIZE;
    sm_oscilloscope->entries[j+i] = oscbuf[r];
  }

  n_osc_entries -= count;
  sm_oscilloscope->n_entries += count;
  
  // give back semaphore
  semGive(sm_oscilloscope_sem);
  
}
