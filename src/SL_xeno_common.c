/*!=============================================================================
  ==============================================================================

  \file    SL_xeno_common.c

  \author  Stefan Schaal
  \date    Nov 2009

  ==============================================================================
  \remarks

  File includes functions that are shared by many processors under
  xenomai

  ============================================================================*/

// system headers
#include "SL_system_headers.h"
#include <execinfo.h>
#include <getopt.h>


// private includes
#include "SL.h"
#include "utility.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"

// xenomai people recommend an aperiodic clock
#define   XENO_CLOCK_PERIOD TM_ONESHOT  //!< base clock rate in nano seconds

// global variables
long count_xenomai_mode_switches = -1;

// external variables

// local variabes

// global functions

// local functions
static void 
action_upon_switch(int sig __attribute__((unused)));

/*!*****************************************************************************
 *******************************************************************************
\note  initXeno
\date  Oct. 2009

\remarks 

xenomai specific initializations

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
initXeno(char *task_name)
{
  int rc;
  struct sigaction sa;

  // lock all of the pages currently and pages that become
  // mapped into the address space of the process
  mlockall(MCL_CURRENT | MCL_FUTURE);

  sl_rt_mutex_init(&mutex1);

  //become a real-time process
  char name[100];
  sprintf(name, "x%s_main_%d", task_name, parent_process_id);
  rt_task_shadow(NULL, name, 0, 0);

  // gsutanto:
  // Only monitor task servo here, because otherwise the overall system becomes too sensitive:
  // openGL servo continuously has real-time violations (because it is NOT real-time),
  // cluttering the real-time violation indications of the task servo.
  // It is sufficient to just activate mode-switches interrupt for the task servo only,
  // assuming SL users only program additional tasks to be called from task servo,
  // and do not tamper with any other servos (i.e. the motor servo is NOT modified and completely real-time-safe).
  // You may see an example of real-time violation detection in the task servo here:
  // https://atlas.is.localnet/confluence/display/AMDW/Automatic+Pin-Pointing+of+Real-Time+Violation+in+SL%27s+Task+Servo
  if (strcmp(task_name, "task") == 0) { // only setup the real-time mode-switch interrupt handler for the task servo
  // what to do when mode switches happen
  signal(SIGDEBUG, action_upon_switch);
  }

  // start the non real-time printing library
  rt_print_auto_init(1);

  // get the timer info
  if ((rc=rt_timer_set_mode((RTIME) XENO_CLOCK_PERIOD)))
    printf("rc_timer_set_mode returned %d\n",rc);

  // check what we got
  RT_TIMER_INFO info;
  rt_timer_inquire(&info);
  if (info.period == TM_ONESHOT)
    printf("Timer Period = TM_ONESHOT\n");
  else
    printf("Timer Period = %ld [ns]\n",(long) info.period);

}


/*!*****************************************************************************
 *******************************************************************************
\note  action_upon_switch
\date  Oct. 2009

\remarks 

what to do when mode switches occur

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static void 
action_upon_switch(int sig __attribute__((unused)))
{
  void *bt[32];
  int nentries;

  // increment mode swich counter
  ++count_xenomai_mode_switches;

  /* Dump a backtrace of the frame which caused the switch to
     secondary mode: 
     (please un-comment the following block for real-time violation monitoring) */
  /*
  nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
  backtrace_symbols_fd(bt,nentries,fileno(stdout));

  getchar();*/

}
