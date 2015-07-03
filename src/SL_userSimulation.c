/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_userSimulation.c

  \author  Stefan Schaal
  \date    Aug. 2010

  ==============================================================================
  \remarks

  this program manages special simulation routines that a user defines
  in his/her own homedirectory. These simulation functions can be activated
  through a shared memory message such that are included int he numerical
  integration

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_man.h"
#include "string.h"
#include "SL_userSimulation.h"

typedef struct userGraphicsEntry {
  char abr[20];                               //!< keyword for this simulation program
  char exp[1000];                             //!< explanation
  void (*func)(void);                         //!< function pointer
  int  active;                                //!< TRUE/FALSE for displaying this graphics
  char *nptr;                                 //!< next pointer

  void (*functionWithBufferArgument)(void *buf);
  int  numberOfValidBytesInBuffer;
  unsigned char *memoryForDataBuffer;

} UserSimulationEntry;

// global variables

// local variables
static UserSimulationEntry *usims = NULL;

// local functions
static void listUserSimulation(void);

/*!*****************************************************************************
 *******************************************************************************
\note  addToUserSimulation
\date  Nov. 2007
   
\remarks 

      allows to add a function to the user simulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     abr     :   functional name as typed on the keyboard
 \param[in]     string  :   explanation
 \param[in]     fptr    :   pointer to the function

 ******************************************************************************/
void
addToUserSimulation(char *abr, char *string, void (*fptr)(void))
{
  int i;
  UserSimulationEntry *ptr;

  // check for duplicate entries
  ptr = usims;
  
  while ( ptr != NULL ) {

    if (strcmp(ptr->abr,abr)==0) {
      
      // update the function pointer and explantory string */
      ptr->func = fptr;
      strcpy(ptr->exp,string);
      
      // the user simulation is inactive until it is activated
      ptr->active = FALSE;

      ptr->numberOfValidBytesInBuffer = -1;
      
      return;
    }
    if (ptr->nptr == NULL)
      break;
    else
      ptr = (UserSimulationEntry *)ptr->nptr;
  }

  // a new entry is created
  if (usims == NULL) {
    usims = (UserSimulationEntry *) my_calloc(1,sizeof(UserSimulationEntry),MY_STOP);
    ptr = usims;    
  } else {
    ptr->nptr = my_calloc(1,sizeof(UserSimulationEntry),MY_STOP);
    ptr = (UserSimulationEntry *)ptr->nptr;
  }
  strcpy(ptr->abr,abr);
  strcpy(ptr->exp,string);
  ptr->func = fptr;
  ptr->numberOfValidBytesInBuffer = -1;
  ptr->active = FALSE;
  ptr->nptr = NULL;

}


addToUserSimulationWithBuffer(char *functionNameToCallOnCommandLine,
                              char *functionDescription,
                              void (*functionPointerToCall)(void *),
                              int numberOfValidBytesInBuffer) {
  UserSimulationEntry *ptr;

  // use the same limits for graphics messages as simulation for now
  if (numberOfValidBytesInBuffer > MAX_BYTES_USER_GRAPHICS) {
    printf("Error: Buffer Size Exceeded: Max. User Graphics Buffer = %d bytes\n", MAX_BYTES_USER_GRAPHICS);
    return;
  }

  ptr = usims;
  while ( ptr != NULL ) {
    if (strcmp(ptr->abr, functionNameToCallOnCommandLine) == 0) {
      // if we already have an entry with this function name, copy over the old data
      ptr->functionWithBufferArgument = functionPointerToCall;
      strcpy(ptr->exp, functionDescription);
      ptr->active = FALSE;
      free(ptr->memoryForDataBuffer);
      ptr->memoryForDataBuffer = my_calloc(ptr->numberOfValidBytesInBuffer, sizeof(unsigned char), MY_STOP);
      return;
    }
    if (ptr->nptr == NULL)
      break;
    else
      ptr = (UserSimulationEntry *)ptr->nptr;
  }
  // ptr now points at the last entry in the list (if any)

  if (usims == NULL) {
    // then create this as the first entry
    usims = (UserSimulationEntry *) my_calloc(1,sizeof(UserSimulationEntry), MY_STOP);
    ptr = usims;
  } else {
    // otherwise allocate a new entry, point the last entry in the list to it
    ptr->nptr = my_calloc(1,sizeof(UserSimulationEntry), MY_STOP);
    ptr = (UserSimulationEntry *)ptr->nptr;
  }

  // ptr points at the new entry, fill in given data:
  strcpy(ptr->abr, functionNameToCallOnCommandLine);
  strcpy(ptr->exp, functionDescription);
  ptr->functionWithBufferArgument = functionPointerToCall;
  ptr->memoryForDataBuffer = my_calloc(ptr->numberOfValidBytesInBuffer, sizeof(unsigned char), MY_STOP);
  ptr->numberOfValidBytesInBuffer = numberOfValidBytesInBuffer;
  ptr->active = FALSE;
  ptr->nptr = NULL;
}


/*!*****************************************************************************
 *******************************************************************************
\note  initUserSim
\date  Nov. 2007
   
\remarks 

initializes the user simulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
initUserSim(void)
{

  // a simple tool to display existing user simulation functions
  addToMan("listUserSimulation","list all user simulation entries",listUserSimulation);
  addToMan("clearUserSimulation","clears all active user simulation entries",clearUserSimulation);


}

/*!*****************************************************************************
 *******************************************************************************
\note  listUserSimulation
\date  Nov. 2007
   
\remarks 

prints out all user simulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
listUserSimulation(void)

{
  int i;
  UserSimulationEntry *ptr;

  ptr = usims;

  while (ptr != NULL) {
    if (ptr->active)
      printf("%20s -- %s (TRUE)\n",ptr->abr,ptr->exp);
    else 
      printf("%20s -- %s (FALSE)\n",ptr->abr,ptr->exp);
    ptr = (UserSimulationEntry *)ptr->nptr;
  }
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  activateUserSimulation
\date  Sept. 2010
   
\remarks 

 sets the status of a user simulation to active

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]  name: name of simulation to be activated

 ******************************************************************************/
int
activateUserSimulation(char *name)
{
  int i,j;
  UserSimulationEntry *ptr;

  // check whether we have an approriate function with this name
  ptr = usims;
  while (ptr != NULL)  {
    
    if (strcmp(ptr->abr,name)==0) {
      // user simulation was successfully identified
      ptr->active = TRUE;
      break;
    }

    ptr = (UserSimulationEntry *)ptr->nptr;
  }
    
  if (ptr == NULL)
    printf("Couldn't find user simulation >%s<\n",name);

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  runUserSimulation
\date  Nov. 2007
   
\remarks 

runs the current user simulation function

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void 
runUserSimulation(void)
{

  UserSimulationEntry *ptr;
  
  ptr = usims;
  while (ptr != NULL) {
    if (ptr->active) {
      if (ptr->numberOfValidBytesInBuffer == -1) {
        (*ptr->func)();
      } else {
        (*ptr->functionWithBufferArgument)(ptr->memoryForDataBuffer);
      }
    }
    ptr = (UserSimulationEntry *)ptr->nptr;
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  clearUserSimulation
\date  Nov. 2007
   
\remarks 

sets all user simulation's status to FALSE

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
void 
clearUserSimulation(void)
{

  UserSimulationEntry *ptr;

  ptr = usims;
  while (ptr != NULL) {
    ptr->active = FALSE;
    ptr = (UserSimulationEntry *)ptr->nptr;
  }

}



