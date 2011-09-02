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
static void warn_upon_switch(int sig, siginfo_t *si, void *context);


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

  // what to do when mode switches happen
  if(!strcmp(task_name, "motor"))
  {
    printf("motor signaling.");
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = warn_upon_switch;
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGDEBUG, &sa, NULL);

  }
  else if(!strcmp(task_name, "task"))
  {
    printf("task signaling.");
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = warn_upon_switch;
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGDEBUG, &sa, NULL);

  }
  else
  {
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
     secondary mode: */
  /*
  nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
  backtrace_symbols_fd(bt,nentries,fileno(stdout));

  getchar();
  */

}


static const char *reason_str[] = {
    [SIGDEBUG_UNDEFINED] = "undefined",
    [SIGDEBUG_MIGRATE_SIGNAL] = "received signal",
    [SIGDEBUG_MIGRATE_SYSCALL] = "invoked syscall",
    [SIGDEBUG_MIGRATE_FAULT] = "triggered fault",
    [SIGDEBUG_MIGRATE_PRIOINV] = "affected by priority inversion",
    [SIGDEBUG_NOMLOCK] = "missing mlockall",
    [SIGDEBUG_WATCHDOG] = "runaway thread",
};

static void warn_upon_switch(int sig, siginfo_t *si, void *context)
{
    unsigned int reason = si->si_value.sival_int;
    void *bt[32];
    int nentries;

    printf("\nSIGDEBUG received, reason %d: %s\n", reason,
           reason <= SIGDEBUG_WATCHDOG ? reason_str[reason] : "<unknown>");
    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
}
