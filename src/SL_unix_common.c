/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_unix_common.c

  \author 
  \date   

  ==============================================================================
  \remarks

  File includes functions that are shared by many processors under
  unix (and not vxWorks)

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// system includes
#include "sys/ioctl.h"
#include "editline/readline.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"

//! defines
#define   MAX_ITEMS             100
#define   MAX_CHARS_COMMAND     20

// global variables
#ifdef __XENO__
sl_rt_mutex mutex1;
#else
sl_rt_mutex mutex1 = PTHREAD_MUTEX_INITIALIZER; //for a safe thread
#endif
int (*window_check_function)(char *) = NULL;
int  run_command_line_thread_flag = FALSE;

// export of main() arguments
int    global_argc;
char **global_argv;

// external variables

// local variabes
static pthread_t       cthread;   // thread for the command interface
static pthread_t       ucthread;  // thread for the user command interface

static char       command[MAX_ITEMS+1][MAX_CHARS_COMMAND];
static int        n_command=0;
static void       (*command_ptr[MAX_ITEMS+1])(void);

static int    time_reset_detected = TRUE;

static char       user_command[MAX_CHARS_COMMAND+1] = "";  // this command can be set by user

// amarcovalle:
static pthread_t        uthread;  // thread for commands sent by the user from a task:
static int              first_time = TRUE;
int                     run_command_from_user_task_thread_flag = FALSE;
// amarcovalle: end

// global functions

// local functions
static void initializeReadLine();
static char **sl_completion(const char *text, int start, int end);
static char *command_generator(const char *text, int state);
static void  checkUserCommand(char *name);
static void *checkKeyboard(void *initial_command);
static void *checkUserCommandThread(void *);

// added: amarcovalle
static void *getCommandFromUserTask(void *v);



/*!*****************************************************************************
*******************************************************************************
\note  checkKeyboard
\date  July 7, 1992
 
\remarks 
 
checks for keyboard interaction an does appropriate functions
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]     inital_command : initial command to be executed as it were typed
                                in the terminal -- pass NULL for nothing
 
******************************************************************************/
static void *
checkKeyboard(void *initial_command)
{
  int             result;
  long            nchars=0;
  int             i=0;
  char            prompt[1000];
  char	         *string;
  char           *ptr, *fptr;
  extern double   servo_time;
  int             rc;


#ifdef __XENO__

  //become a real-time process
  char name[100];
  sprintf(name, "%s_terminal_%d", servo_name, parent_process_id);
  rt_task_shadow(NULL, name, 0, 0);

  // we want this task in non real-time mode
  //if ((rc=rt_task_set_mode(T_PRIMARY,0,NULL)))
  //  printf("SL_unix_common.c : rt_task_set_mode returned %d\n",rc);

#endif


  while (run_command_line_thread_flag) {

    // run initial command as soon as the task servo has started -- there is 
    // a little trick that allows resetting the servo clock and re-run the
    // initial command, which is useful for simulations.

    // the "time_reset_detected" flag is set by the sl_readline_callback
    // function which detects a reset of the servo clock

    if (time_reset_detected && strcmp(servo_name,"task")==0 && initial_command != NULL ) {
      // the clock has been reset

      // Special LittleDog Hack -- to be removed?
      // if the environment variable "SL_TASK_SERVO_STANDALONE" is set,
      // don't wait for the servo time to start ticking:
      if (getenv("SL_TASK_SERVO_STANDALONE"))
        usleep(100000);
      // else wait until the servo_time goes beyond 100ms:
      else {
        while (servo_time < 0.1)
	        usleep(10000);
      }

      if (initial_command != NULL) {
        checkUserCommand((char *)initial_command);
      }
      time_reset_detected = 0;

      /*  
    // run initial command as soon as the task servo has started 

    if (strcmp(servo_name,"task")==0 && initial_command != NULL && servo_time < 0.1) {

      // wait until the servo_time goes beyond 100ms:
      while (servo_time < 0.1)
	usleep(10000);

      checkUserCommand((char *)initial_command);
      */    


    }

    snprintf(prompt, 1000, "%s.%s> ",robot_name,servo_name);
    string = readline(prompt);
    if (string && *string) {
      add_history(string);
      checkUserCommand(string);
    }
    free(string);

    // commented: amarcovalle
    // // this allows the user to run a command line command from a program, 
    // // and in partciular a real-time program
    // if (strlen(user_command) > 0) {
    //   checkUserCommand(user_command);
    //   strcpy(user_command,"\0");
    // }

  } 

  printf("Command line thread terminated\n");

  return NULL;

}

/*!*****************************************************************************
*******************************************************************************
\note  getCommandFromUserTask
\date  April 5, 2016

\author Alonso Marco
 
\remarks 
 
checks whether a new command has been passed from the SL task
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
******************************************************************************/
static void *
getCommandFromUserTask(void *v)
{
  extern double   servo_time;
  int             rc;

// #ifdef __XENO__

//   //become a real-time process
//   char name[100];
//   sprintf(name, "%s_getCommandFromUserTask_%d", servo_name, parent_process_id);
//   int thread_priority = 1; // Lower than the command interface one
//   rt_printf("[DBG]: @check_sent_command(), (1)\n");
//   rt_task_shadow(NULL, name, thread_priority, 0);
//   rt_printf("[DBG]: @check_sent_command(), (2)\n");

//   // we want this task in non real-time mode
//   if ( rc=rt_task_set_mode(T_PRIMARY,0,NULL) )
//     rt_printf("rt_task_set_mode returned %d\n",rc);

//   rt_printf("[DBG]: @check_sent_command(), (3)\n");

// #endif

  if (first_time && strcmp(servo_name,"task")==0) {
    // the clock has been reset

    // Special LittleDog Hack -- to be removed?
    // if the environment variable "SL_TASK_SERVO_STANDALONE" is set,
    // don't wait for the servo time to start ticking:
    if (getenv("SL_TASK_SERVO_STANDALONE"))
      usleep(100000);
    // else wait until the servo_time goes beyond 100ms:
    else {
      while (servo_time < 0.1)
        usleep(10000);
    }
    first_time = 0;
  }

  while (run_command_from_user_task_thread_flag)
  {
    // this allows the user to run a command line command from a program, 
    // and in partciular a real-time program
    if (strlen(user_command) > 0)
    {
      // #ifdef __XENO__
      //   rt_printf("[DBG]: @check_sent_command, new command detected!!, %s\n",user_command);
      // #else
      //   printf("[DBG]: @check_sent_command, new command detected!!, %s\n",user_command);
      // #endif
      checkUserCommand(user_command);
      strcpy(user_command,"\0");
      printPrompt();
    }

    // We check for new commands every 100 ms:
    usleep(500000);

  }

  return NULL;

}

/*!*****************************************************************************
*******************************************************************************
\note  checkUserCommand
\date  June 1999
   
\remarks 

check whether a string matches a user command an executes it

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name of the command


******************************************************************************/
static void
checkUserCommand(char *name)
{

  int i,rc,len;
  char c[100];

  // strip trailing white-space:
  len = strlen(name);
  for (i=len-1; i>=0; i--) {
    if (name[i]==' ')
      name[i]=0;
    else
      break;
  }

  for (i=1; i<=n_command; ++i) {
    if (strcmp(name,command[i])==0) {
      (*command_ptr[i])();
      return;
    }
  }

  if (window_check_function != NULL)
    if ((*window_check_function)(name))
      return;

  if (strcmp(name,"")!=0)
    printf(" ???\n");
  return;

}
/*!*****************************************************************************
*******************************************************************************
\note  printPrompt
\date  Feb.99
   
\remarks 

prints the prompt for input

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
void 
printPrompt(void)

{
  #ifdef __XENO__
    rt_printf("%s.%s> ",robot_name,servo_name);
  #else
    printf("%s.%s> ",robot_name,servo_name);
  #endif
  fflush(stdout);
  fflush(stdin);
}

/*!*****************************************************************************
*******************************************************************************
\note  spawnCommandLineThread
\date  July 7, 1992
 
\remarks 
 
spawns off the command line interface as a separate thread
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in] initial_command : the name of a command line task to be executed
                             at startup
 
******************************************************************************/
void
spawnCommandLineThread(char *initial_command) 
{
  int err = 0;
  int rc;
  pthread_attr_t pth_attr;
  size_t stack_size = 0;

  err = pthread_attr_init(&pth_attr);
  pthread_attr_getstacksize(&pth_attr, &stack_size);
  double reqd = 1024*1024*8;
  if (stack_size < reqd)
    pthread_attr_setstacksize(&pth_attr, reqd);

  /* initialize a thread for the user command interface */
  run_command_line_thread_flag = TRUE;
  initializeReadLine();
  if ((rc=pthread_create( &cthread, &pth_attr, checkKeyboard, initial_command)))
      printf("pthread_create returned with %d\n",rc);

  printSLBanner();

}

/*!*****************************************************************************
*******************************************************************************
\note  spawnCommandFromUserTaskThread
\date  April 5, 2016

\author Alonso Marco
 
\remarks 

spawns off a separate thread for reading commands sent by the user from an SL user task
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
******************************************************************************/
void
spawnCommandFromUserTaskThread(void)
{
  int err = 0;
  int rc;
  pthread_attr_t pth_attr;
  size_t stack_size = 0;

  err = pthread_attr_init(&pth_attr);
  pthread_attr_getstacksize(&pth_attr, &stack_size);
  double reqd = 1024*1024*8;
  if (stack_size < reqd)
    pthread_attr_setstacksize(&pth_attr, reqd);

  /* initialize a thread for the user command interface */
  run_command_from_user_task_thread_flag = TRUE;
  if ((rc=pthread_create( &uthread, &pth_attr, getCommandFromUserTask, NULL)))
      printf("pthread_create returned with %d\n",rc);

}

/*!*****************************************************************************
*******************************************************************************
\note  initializeReadLine
\date  Oct 22, 2008
 
\remarks 
 
Initializes command line completion
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
none
 
******************************************************************************/
static void
initializeReadLine()
{
  // commented: amarcovalle
  extern int rl_catch_signals; // for some reason this isn't in editline/readline.h

  rl_attempted_completion_function = sl_completion;
  rl_catch_signals = 0;
  rl_initialize();

}

/*!*****************************************************************************
*******************************************************************************
\note  sl_completion
\date  Oct 22, 2008
 
\remarks 
 
A custom command-line completion interface for readline. It uses the function
command_generator to generate a list of matches for commands starting with
the given text. Refer readline documentation for details:
http://tiswww.case.edu/php/chet/readline/readline.html#SEC44

 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   text  :  text to be matched
\param[in]   start :  not used
\param[in]     end :  not used
 
******************************************************************************/
static char **
sl_completion(const char *text, int start, int end)
{
  char **matches = NULL;

  if (start == 0)
    matches = rl_completion_matches(text, command_generator);
  else
    rl_attempted_completion_over = TRUE;

  return matches;
}

/*!*****************************************************************************
*******************************************************************************
\note  command_generator
\date  Oct 22, 2008
 
\remarks 
 
Returns the next matching command for the given text.
Refer readline documentation for details:
http://tiswww.case.edu/php/chet/readline/readline.html#SEC44

 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   text : text to be matched
\param[in]  state : ??
 
******************************************************************************/
static char *
command_generator(const char *text, int state)
{
  static int i, len;
  char *name;

  if (!state)
    {
      i = 1;
      len = strlen(text);
    }

  while (i<=n_command)
    {
      name = command[i];
      i++;
      if (strncmp(name, text, len) == 0)
	return (strdup(name));
    }

  return ((char*)NULL);
}

/*!*****************************************************************************
*******************************************************************************
\note  addCommand
\date  June 1999
   
\remarks 

add a command to the executable commands

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name of the command
\param[in]     fptr : function pointer to be called with this command


******************************************************************************/
void
addCommand(char *name, void (*fptr)(void)) 

{

  int i;

  /* does the command already exist ? */
  for (i=1; i<=n_command; ++i) {
    if (strcmp(command[i],name)==0) {
      // overwrite the function pointer
      command_ptr[i] = fptr;
      return;
    }
  }

  if (n_command < MAX_ITEMS) {
    ++n_command;
    strcpy(command[n_command],name);
    command_ptr[n_command] = fptr;
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  installSignalHandlers
\date  Nov. 2007
   
\remarks 

install signal handlers and records process ID of parent process

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
installSignalHandlers(void)

{

  // install signal handler for shared memory objects 
  signal(SIGABRT,removeSharedMemory);
  signal(SIGFPE,removeSharedMemory);
  signal(SIGILL,removeSharedMemory);
  //signal(SIGSEGV,removeSharedMemory);
  signal(SIGINT,removeSharedMemory);
  signal(SIGTERM,removeSharedMemory);
  atexit(removeSharedMemoryAtExit);

}

/*!*****************************************************************************
*******************************************************************************
\note  printSLBanner
\date  Nov. 2007
   
\remarks 

just some decoration

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
printSLBanner(void) 
{

  printf("\n             ******************************************\n");
  printf("             **             Welcome to SL            **\n");    
  printf("             ******************************************\n\n");

}

/*!*****************************************************************************
*******************************************************************************
\note  parseOptions
\date  Nov. 2007
 
\remarks 
 
parses options to the main program
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 
******************************************************************************/
void
parseOptions(int argc, char**argv)

{
  int i,j,n;
  int rc;
  int ans; 

  // assign to global variables
  global_argc = argc;
  global_argv = argv;

  // the parent process ID
  parent_process_id = 0;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-pid")==0 && i < argc-1) {
      sscanf(argv[i+1],"%d",&parent_process_id);
      break;
    }
  }

  // get the realtiime flag
  real_robot_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-rt")==0) {
      real_robot_flag = TRUE;
      break;
    }
  }

  // check for no-graphics flag
  no_graphics_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-ng")==0) {
      no_graphics_flag = TRUE;
      break;
    }
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  sendCommandLineCmd
\date  Jan 2012
   
\remarks 

sends a command to the executed in the command-line thread. Note that this 
should not be called at high frequency as this won't work.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name of the command


******************************************************************************/
void
sendCommandLineCmd(char *name)
{
  int err = 0;
  int rc;
  pthread_attr_t pth_attr;
  pthread_t      lcthread;  
  size_t stack_size = 0;
  int i;

  // the user command
  strncpy(user_command, name, MAX_CHARS_COMMAND);
  rl_stuff_char('\n');


  /* run the command in separate thread */
  /*
  err = pthread_attr_init(&pth_attr);
  pthread_attr_getstacksize(&pth_attr, &stack_size);
  double reqd = 1024*1024*8;
  if (stack_size < reqd)
    pthread_attr_setstacksize(&pth_attr, reqd);

  if ((rc=pthread_create( &ucthread, &pth_attr, checkUserCommandThread, NULL)))
      printf("pthread_create returned with %d\n",rc);

  */
}

/*!*****************************************************************************
*******************************************************************************
\note  checkUserCommandThread
\date  Nov. 2014
 
\remarks 
 
thread to run a specific user command without real-time interference
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in] dummy : dummy variable
 
******************************************************************************/
static void *
checkUserCommandThread(void *dummy) 
{

  checkUserCommand(user_command);
  strcpy(user_command,"\0");


  return NULL;

}
