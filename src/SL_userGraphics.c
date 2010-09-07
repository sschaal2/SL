/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_userGraphics.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  this program manages special graphics routines that a user defines
  in his/her own homedirectory. These graphics functions can be called
  through a shared memory segment by name, which will add them to the
  graphics updating.

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "string.h"
#include "SL_openGL_servo.h"

// openGL includes
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif


#define TIME_OUT_NS  1000000000

typedef struct userGraphicsEntry {
  char abr[20];                               //!< keyword for this graphics program
  char exp[1000];                             //!< explanation
  void (*func)(void *buf);                    //!< function pointer
  int  n_bytes;                               //!< number of valid bytes in buf
  unsigned char *buf;                         //!< memory for data buffer
  int  active;                                //!< TRUE/FALSE for displaying this graphics
  int  user_graphics_update;                  //!< TRUE if buf was updated
  char *nptr;                                 //!< next pointer
} UserGraphicsEntry;

// global variables
int  user_graphics_update = FALSE;

// local variables
static UserGraphicsEntry *ugraphs = NULL;
static int user_graphics_initialized = FALSE;

// local functions
static void initializeUserGraphics(void);
static void  (*user_display_function) (void *) = NULL;
static void displayBall(void *b);
static void listUserGraphics(void);
static void clearUserGraphics(void);


/*!*****************************************************************************
 *******************************************************************************
\note  addToUserGraphics
\date  Nov. 2007
   
\remarks 

      allows to add a function to the user graphics

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     abr     :   functional name as typed on the keyboard
 \param[in]     string  :   explanation
 \param[in]     fptr    :   pointer to the function
 \param[in]     n_bytes :   number of bytes in buffer

 ******************************************************************************/
void
addToUserGraphics(char *abr, char *string, void (*fptr)(void *), int n_bytes)
{
  int i;
  UserGraphicsEntry *ptr;


  if (!user_graphics_initialized) 
    initializeUserGraphics();

  if (n_bytes > MAX_BYTES_USER_GRAPHICS) {
    printf("Error: Buffer Size Exceeded: Max. User Graphics Buffer = %d bytes\n",
	   MAX_BYTES_USER_GRAPHICS);
    return;
  }

  // check for duplicate entries
  ptr = ugraphs;
  
  while ( ptr != NULL ) {

    if (strcmp(ptr->abr,abr)==0) {
      
      // update the function pointer and explantory string */
      ptr->func = fptr;
      strcpy(ptr->exp,string);
      
      // the user graphics is inactive until it receives data
      ptr->active = FALSE;
      ptr->user_graphics_update = FALSE;
      
      // update the memory buffer
      free(ptr->buf);
      ptr->buf = my_calloc(ptr->n_bytes,sizeof(unsigned char),MY_STOP);
      
      return;
    }
    if (ptr->nptr == NULL)
      break;
    else
      ptr = (UserGraphicsEntry *)ptr->nptr;
  }

  // a new entry is created
  if (ugraphs == NULL) {
    ugraphs = (UserGraphicsEntry *) my_calloc(1,sizeof(UserGraphicsEntry),MY_STOP);
    ptr = ugraphs;    
  } else {
    ptr->nptr = my_calloc(1,sizeof(UserGraphicsEntry),MY_STOP);
    ptr = (UserGraphicsEntry *)ptr->nptr;
  }
  strcpy(ptr->abr,abr);
  strcpy(ptr->exp,string);
  ptr->func = fptr;
  ptr->n_bytes = n_bytes;
  ptr->active = FALSE;
  ptr->user_graphics_update = FALSE;
  ptr->buf = my_calloc(ptr->n_bytes,sizeof(unsigned char),MY_STOP);
  ptr->nptr = NULL;

}
/*!*****************************************************************************
 *******************************************************************************
\note  initializeUserGraphics
\date  Nov. 2007
   
\remarks 

initializes the user graphics with a default entry

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
initializeUserGraphics(void)
{

  if (user_graphics_initialized)
    return;

  user_graphics_initialized=TRUE;

  // a simple tool to display existing user graphics functions
  addToMan("listUserGraphics","list all user graphics entries",listUserGraphics);
  addToMan("clearUserGraphics","clears all active user graphics entries",clearUserGraphics);

  // add some generally hand display function
  addToUserGraphics("ball","Display a 6cm diameter ball",displayBall,N_CART*sizeof(float));


}

/*!*****************************************************************************
 *******************************************************************************
\note  listUserGraphics
\date  Nov. 2007
   
\remarks 

      prints out all user graphics

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
static void
listUserGraphics(void)

{
  int i;
  UserGraphicsEntry *ptr;

  if (!user_graphics_initialized)
    initializeUserGraphics();

  ptr = ugraphs;

  while (ptr != NULL) {
    if (ptr->active)
      printf("%20s -- %s (TRUE)\n",ptr->abr,ptr->exp);
    else 
      printf("%20s -- %s (FALSE)\n",ptr->abr,ptr->exp);
    ptr = (UserGraphicsEntry *)ptr->nptr;
  }
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  checkForUserGraphics
\date  Nov. 2007
   
\remarks 

      checks whether user graphics is available and, if yes, assigns the
      appropriate function to the graphics execution

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
int
checkForUserGraphics(void)
{
  int i,j;
  char name[20];
  UserGraphicsEntry *ptr;

  if (!user_graphics_initialized)
    initializeUserGraphics();
  
  // check whether user graphics is ready
  if (semTake(sm_user_graphics_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the user graphics data
  if (semTake(sm_user_graphics_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take user_graphics semaphore\n");
    return FALSE;
  }

  // get the name of user graphics
  strcpy(name,sm_user_graphics->name);

  // check whether we have an approriate function with this name
  ptr = ugraphs;
  while (ptr != NULL)  {

    if (strcmp(ptr->abr,name)==0) {
      // user graphics was successfully identified -- copy remaining shared
      // memory
      memcpy(ptr->buf,sm_user_graphics->buf,ptr->n_bytes);
      
      semGive(sm_user_graphics_sem);
      ptr->user_graphics_update = TRUE;
      ptr->active = TRUE;
      return TRUE;
    }
    ptr = (UserGraphicsEntry *)ptr->nptr;
  } while (TRUE);

  // only happens if entry not found
  printf("Couldn't find user graphics >%s<\n",name);
  semGive(sm_user_graphics_sem);

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  runUserGraphics
\date  Nov. 2007
   
\remarks 

     runs the current user graphics function

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
void 
runUserGraphics(void)
{

  UserGraphicsEntry *ptr;

  if (!user_graphics_initialized)
    initializeUserGraphics();

  ptr = ugraphs;
  while (ptr != NULL) {
    if (ptr->active) {
      user_graphics_update = ptr->user_graphics_update;
      (*ptr->func)(ptr->buf);
      ptr->user_graphics_update = FALSE;
    }
    ptr = (UserGraphicsEntry *)ptr->nptr;
  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  displayBall
 \date  Nov. 2007 
 
 \remarks 

        a simple ball in 3D

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]      b      : the general array of bytes

 ******************************************************************************/
static void
displayBall(void *b)
{
  GLfloat  col[4]={(float)1.0,(float)1.0,(float)0.0,(float)1.0};
  float    ball[N_CART+1];

  // assign the ball position from b array
  memcpy(&(ball[_X_]),b,N_CART*sizeof(float));

  /* here is the drawing rountines */
  glPushMatrix();
  glTranslated((GLdouble)ball[_X_],
	       (GLdouble)ball[_Y_],
	       (GLdouble)ball[_Z_]);
  
  glColor4fv(col);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
  glutSolidSphere(0.03,8,8);
  glPopMatrix();

}

/*!*****************************************************************************
 *******************************************************************************
\note  clearUserGraphics
\date  Nov. 2007
   
\remarks 

sets all user graphics's status to FALSE

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
static void 
clearUserGraphics(void)
{

  UserGraphicsEntry *ptr;

  ptr = ugraphs;
  while (ptr != NULL) {
    ptr->active = FALSE;
    ptr = (UserGraphicsEntry *)ptr->nptr;
  }

}



