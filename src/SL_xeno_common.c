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
action_upon_switch(int sig, siginfo_t *si, void *context);

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

  // Note that only pthread_mutex_init() may be used to initialize a mutex,
  // using the static initializer PTHREAD_MUTEX_INITIALIZER is not supported by Xenomai.
  pthread_mutexattr_t attr;
  pthread_mutexattr_setpshared(&attr,PTHREAD_PROCESS_SHARED);
  pthread_mutexattr_setprotocol(&attr,PTHREAD_PRIO_INHERIT);
  pthread_mutex_init(&mutex1,&attr);

  //become a real-time process
  char name[100];
  sprintf(name, "x%s_main", task_name);
  rt_task_shadow(NULL, name, 0, 0);

  // start the non real-time printing library
  rt_print_auto_init(1);

  // what to do when mode switches happen
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = action_upon_switch;
  sa.sa_flags = SA_SIGINFO;
  sigaction(SIGDEBUG, &sa, NULL);
  //signal(SIGDEBUG, action_upon_switch);

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
static const char *reason_str[] = {
                                   [SIGDEBUG_UNDEFINED] = "undefined",
                                   [SIGDEBUG_MIGRATE_SIGNAL] = "received signal",
                                   [SIGDEBUG_MIGRATE_SYSCALL] = "invoked syscall",
                                   [SIGDEBUG_MIGRATE_FAULT] = "triggered fault",
                                   [SIGDEBUG_MIGRATE_PRIOINV] = "affected by priority inversion",
                                   [SIGDEBUG_NOMLOCK] = "missing mlockall",
                                   [SIGDEBUG_WATCHDOG] = "runaway thread",
};

static void 
action_upon_switch(int sig, siginfo_t *si, void *context)

{
  unsigned int reason = si->si_value.sival_int;
  void *bt[32];
  int nentries;

  // increment mode swich counter
  if ( (reason >= SIGDEBUG_MIGRATE_SIGNAL) && (reason <=  SIGDEBUG_MIGRATE_PRIOINV) )
  {
    ++count_xenomai_mode_switches;
    //rt_printf("modeswitch\n");
    printf("\nSIGDEBUG received, reason %d: %s\n", reason,
           reason <= SIGDEBUG_WATCHDOG ? reason_str[reason] : "<unknown>");
    //rt_printf("\nRTprintf SIGDEBUG received, reason %d: %s\n", reason,
    //          reason <= SIGDEBUG_WATCHDOG ? reason_str[reason] : "<unknown>");
  }
  else
  {
    extern RT_TASK servo_ptr;
    //stop the task that is causing trouble
    rt_task_suspend(&servo_ptr);

    printf("\nSIGDEBUG received, reason %d: %s\n", reason,
           reason <= SIGDEBUG_WATCHDOG ? reason_str[reason] : "<unknown>");
    nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt,nentries,fileno(stderr));
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
    fflush(stdout);
    fflush(stderr);

    //now we resume the task
    //rt_task_resume(&servo_ptr);
  }


  //printf("\nSIGDEBUG received, reason %d: %s\n", reason,
  //	 reason <= SIGDEBUG_WATCHDOG ? reason_str[reason] : "<unknown>");
  /* Dump a backtrace of the frame which caused the switch to
     secondary mode: */



  /* Dump a backtrace of the frame which caused the switch to
     secondary mode: */
  /*
  nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
  backtrace_symbols_fd(bt,nentries,fileno(stdout));

  getchar();
   */

}
