/*!=============================================================================
  ==============================================================================

  \file    SL_unix_common.h

  \author 
  \date   

  ==============================================================================
  \remarks

  declarations needed by SL_unix_common.c

  ============================================================================*/

#ifndef _SL_unix_common_
#define _SL_unix_common_

#include "sys/time.h"
#include "unistd.h"
#include "pthread.h"

#ifdef __cplusplus
extern "C" {
#endif

  // external variables 
  extern pthread_mutex_t mutex1;
  extern int (*window_check_function)(char *);

  // global functions 
  void  spawnCommandLineThread(char *initial_command);
  void  addCommand(char *name, void (*fptr)(void));
  void  installSignalHandlers(void);
  void  printSLBanner(void);
  void  printPrompt(void);
  void  parseOptions(int argc, char**argv);
  
#ifdef __cplusplus
}
#endif

#endif
