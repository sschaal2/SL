/*!=============================================================================
  ==============================================================================

  \file    SL_main.c

  \author  Stefan Schaal
  \date    Jan 1999

  ==============================================================================
  \remarks
  
  Entry program for dynamics simulations
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"
#include "sys/wait.h"
#include "unistd.h"
#include <X11/Xlib.h>

/* openGL headers */
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

/* local headers */
#include "SL.h"
#include "SL_openGL.h"
#include "utility.h"
  
/* local functions */

  
/*!*****************************************************************************
 *******************************************************************************
 \note  main
 \date  July 1998
 
 \remarks 
 
 entry program: the structure of the entire program is such that we assume 
 some initialization functions, and then a final function that continuously
 runs the simulation in a "clocked" fashion.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     argc : number of elements in argv
 \param[in]     argv : array of argc character strings
 
 ******************************************************************************/
int 
main(int argc, char**argv)

{
  int i,j,n;
  int rc;
  int ans; 

  /* what is the process ID of this program */
  parent_process_id = getppid();
  
  /* install signal handler for shared memory objects */
  signal(SIGABRT,removeSharedMemory);
  signal(SIGFPE,removeSharedMemory);
  signal(SIGILL,removeSharedMemory);
  signal(SIGSEGV,removeSharedMemory);
  signal(SIGINT,removeSharedMemory);
  signal(SIGTERM,removeSharedMemory);
  atexit(removeSharedMemoryAtExit);
  
  /* initialize graphics */
  if (!initGraphics(&argc, &argv))
    return FALSE;
  
  /* generic initialization program */
  if (!initSimulation(argc, argv)) 
    return FALSE;
  
  /* start the main loop */   
  glutMainLoop();
  
  
  return TRUE;
}


