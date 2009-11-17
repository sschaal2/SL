/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  This file contains all function to handle the openGL graphics output of the
  simulation.
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// UNIX specific headers
#ifdef UNIX
#include "sys/ioctl.h"
#ifdef sparc
#include "sys/filio.h"
#include "unistd.h"
#endif
#endif

// openGL headers
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

// mathematica headers
#include "mdefs.h"

// user specific headers
#include "SL.h"
#include "SL_openGL.h"
#include "SL_terrains.h"
#include "SL_common.h"
#include "SL_objects.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_kinematics.h"
#include "SL_vx_wrappers.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"


//! local defines
#define   RAD2DEG (180./3.1416)
#define   LIGHT_TURN_RATE	10
#define   VIEW_TURN_RATE	30
#define   MAX_ITEMS             100

// global functions 
void  SLGenericDisplay(void);
int   initializeLighting(OpenGLWPtr wptr);

// local functions
static void  glutMenu(OpenGLWPtr wptr);
static void  reshape(int w, int h);
static void  scene_display(void);
static void  special(int key, int x, int y);
static void  idle(void);
static void  keyboard(unsigned char key, int x, int y);
static void  mouse(int button, int state, int x, int y);
static void  motion(int x, int y);
static void  my_exit(void);
static void  toggleClmcplotMode(void);
static void  togglePlaybackMode(void);
static void  toggleUserGraphicsMode(void);
static void  extractState(char **vnames, Vector D, int n_cols);
static void  reset_playback(void);
static void  go_playback(void);
static void  step_playback(void);
static void  stepsize_playback(void);
static void  new_playback(void);
static void  createTerrainDisplayList(ObjectPtr ptr,double *rgb);
static int   checkWindowHide(char *);
static void  togglePause(void);


// global variables 
int        solid = TRUE;
double     window_update_rate = 30.;
int        clmcplot_mode  = FALSE;
int        playback_mode = FALSE;
int        userGraphics_mode = FALSE;
int        pause_flag    = FALSE;
int        modifiers     = 0;
int        mouseX	 = 0;
int        mouseY        = 0;
SL_Jstate* userGraphics_joint_state;
SL_Cstate  userGraphics_base_state;
SL_quat    userGraphics_base_orient;

/* local variables */
static OpenGLWPtr first_window_ptr = NULL;
static char       menu[3+1][MAX_ITEMS+1][20];
static int        n_menu[3+1]={0,0,0,0};
static void       (*menu_ptr[MAX_ITEMS+1])(void);
static char       command[MAX_ITEMS+1][20];
static int        n_command=0;
static void       (*command_ptr[MAX_ITEMS+1])(void);

static double VIEW_TRANSX0   =   0.0;
static double VIEW_TRANSY0   =   0.0;
static double VIEW_TRANSZ0   =   0.0;
static double VIEW_DISTANCE0 =  -2.0;
static double VIEW_ROTX0     = -80.0;
static double VIEW_ROTY0     =   0.0;
static double VIEW_ROTZ0     =-165.0;

// variables for clmcplot_mode and playback_mode
Matrix playback_D = NULL;
int    playback_n_cols;
int    playback_current_row;
char   **playback_vnames = NULL;

static char   **playback_units  = NULL;
static double playback_freq;
static int    playback_n_rows;
static int    playback_step_mode = FALSE;
static int    playback_step_size = 1;
static char   playback_file_name[100];

/*!*****************************************************************************
*******************************************************************************
\note  initGraphics
\date  August 7, 1992
   
\remarks 

initializes openGl routines

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 

******************************************************************************/
int
initGraphics(int *argc, char*** argv)

{

  int i;
  int rc;
  unsigned char string[255];

  // initialize OpenGL
  glutInit(argc, *argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
  
  // get shared memory
  if (!init_shared_memory())
    return FALSE;

  // object handling
  if (!initObjects())
    return FALSE;

  // add the quit command
  addCommand("quit",my_exit);

  // clmcplot mode for debugging data files
  addToMan("clmcplotMode","toggle clmcplot visualization",toggleClmcplotMode);
  
  // userGraphics mode for controlling the robot state from user graphics
  addToMan("userGraphicsMode","toggle userGraphics robot state visualization",
	   toggleUserGraphicsMode);

  // playback mode for debugging data files
  addToMan("playbackMode","toggle playback visualization",togglePlaybackMode);
  addToMan("r","reset playback file to first frame",reset_playback);
  addToMan("s","step playback file to next frame",step_playback);
  addToMan("g","run playback file",go_playback);
  addToMan("n","new playback file",new_playback);
  addToMan("ss","playback step size",stepsize_playback);
  addToMan("p","toggle pause",togglePause);

  // needed to toggle hide status of windows
  window_check_function = checkWindowHide;

  // memory allocation for userGraphics_joint_state:
  userGraphics_joint_state = (SL_Jstate*)malloc((n_dofs+1)*sizeof(SL_Jstate));

  return TRUE;

}

static void
my_exit(void) {
  exit(-1);
}

/*!*****************************************************************************
*******************************************************************************
\note  reshape
\date  August 7, 1992
   
\remarks 

window moving function

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void 
reshape(int w, int h)

{

  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  glViewport(0,0,(GLsizei) w, (GLsizei) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(ptr->fovea, (GLfloat) w/ (GLfloat) h, 0.1, 20.0);
  glMatrixMode(GL_MODELVIEW);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  mouse
\date  Dec 11, 2007
   
\remarks 

Callback for mouse button up/down events
Used for rotating, panning and zooming the view

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void  mouse(int button, int state, int x, int y)
{
  if (button==GLUT_RIGHT_BUTTON && state==GLUT_DOWN)
    {
      modifiers=glutGetModifiers();
      mouseX=x;
      mouseY=y;
    }
  else
    modifiers=0;
}

/*!*****************************************************************************
*******************************************************************************
\note  mouse
\date  Dec 11, 2007
   
\remarks 

Callback for mouse motion events
Used for rotating, panning and zooming the view

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void  motion(int x, int y)
{
  int flag=0, i;
  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;
  if (modifiers)
    {
      int dx=x-mouseX;
      int dy=y-mouseY;
      if ((modifiers & GLUT_ACTIVE_SHIFT)
	  &&
	  (modifiers & GLUT_ACTIVE_CTRL))
	{
	  // rotation
	  double distance=0.0;
	  double phi;
	  double theta;
	  double x=ptr->eye[_X_]-ptr->center[_X_];
	  double y=ptr->eye[_Y_]-ptr->center[_Y_];
	  double z=ptr->eye[_Z_]-ptr->center[_Z_];

	  distance=sqrt(x*x+y*y+z*z);
	  phi = atan2(sqrt(x*x+y*y),z);
	  theta = atan2(y,x);
	  theta-=(double)dx*.01;
	  phi-=(double)dy*.01;
			
	  if (phi >= PI)
	    phi = PI - .001;
	  if (phi <= 0)
	    phi = .001;

	  ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
	  ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
	  ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
	  flag++;
	}
      else if (modifiers & GLUT_ACTIVE_SHIFT)
	{
	  // panning
	  double x=ptr->eye[_X_]-ptr->center[_X_];
	  double y=ptr->eye[_Y_]-ptr->center[_Y_];
	  double theta = atan2(y,x);
	  double ddx = .003*(-dy*cos(theta) + dx*sin(theta));
	  double ddy = .003*(dx*cos(theta) + dy*sin(theta));
	  ptr->center[_X_]+=ddx;
	  ptr->center[_Y_]-=ddy;
	  ptr->eye[_X_]+=ddx;
	  ptr->eye[_Y_]-=ddy;
	  flag++;
	}
      else if (modifiers & GLUT_ACTIVE_CTRL)
	{
	  // zooming
	  double distance=0.0;
	  double newDist=0.0;
	  double ddist=0.003*(double)dy;
	  for (i=_X_; i<=_Z_; i++)
	    {
	      distance+=sqr(ptr->center[i]-ptr->eye[i]);
	    }
	  distance=sqrt(distance);
	  newDist=distance+ddist;
	  for (i=_X_; i<=_Z_; i++)
	    {
	      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])*(newDist/distance);
	    }
	  flag++;
	}
    }
  mouseX=x;
  mouseY=y;
  
  if (flag)
    glutPostRedisplay();
}

/*!*****************************************************************************
*******************************************************************************
\note  keyboard
\date  August 7, 1992
   
\remarks 

function for keyboard interaction

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     key : which key was pressed
\param[in]     x   : x location where the key was pressed
\param[in]     y   : y location where the key was pressed

******************************************************************************/
static void 
keyboard(unsigned char key, int x, int y)

{
  
  int i,j,n,flag=FALSE;
  OpenGLWPtr ptr;
  double phi;
  double theta;
  double distance;
  char c[100];

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  switch (key) {
  case 't':
    ++flag;
    printf("%c\n",key);
    if (solid == TRUE)
      solid = FALSE;
    else
      solid = TRUE;
    break;
    
  case 'q':
  case 27: /* ESC */
    ++flag;
    printf("%c\n",key);
    exit(-1);
    break;
    
    /* start of view position functions */
  case GLUT_KEY_RIGHT:  /* right turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    theta -= 0.05;
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Right\n");
    break;

  case GLUT_KEY_LEFT:  /* left turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    theta += 0.05;
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Left\n");
    break;

  case GLUT_KEY_UP:  /* upward turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    phi -= 0.05;
    if (phi <= 0) {
      phi = 0.001;
      printf("Camera cannot turn lower\n");
    }
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Downward\n");
    break;
    
  case GLUT_KEY_DOWN:  /* downward turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    phi += 0.05;
    if (phi >= PI) {
      phi = PI - 0.001;
      printf("Camera cannot turn higer\n");
    }
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Upward\n");
    break;

  case 'i':
    ++flag;
    ptr->center[_Y_] -= 0.1;  
    ptr->eye[_Y_]    -= 0.1;  
    printf("Camera +y\n");
    break;

  case 'k':
    ++flag;
    ptr->center[_Y_] += 0.1;  
    ptr->eye[_Y_]    += 0.1;  
    printf("Camera -y\n");
    break;

  case 'l':
    ++flag;
    ptr->center[_X_] += 0.1;  
    ptr->eye[_X_]    += 0.1;  
    printf("Camera +x\n");
    break;

  case 'j':
    ++flag;
    ptr->center[_X_] -= 0.1;  
    ptr->eye[_X_]    -= 0.1;  
    printf("Camera -x\n");
    break;

  case 'u':
    ++flag;
    ptr->center[_Z_] += 0.1;  
    ptr->eye[_Z_]    += 0.1;  
    printf("Camera +z\n");
    break;

  case 'h':
    ++flag;
    ptr->center[_Z_] -= 0.1;  
    ptr->eye[_Z_]    -= 0.1;  
    printf("Camera -z\n");
    break;

  case 'a':
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    distance -= 0.1;
    if (distance <= 0.1) {
      distance = 0.1;
      printf("Camera cannot come closer\n");
    }
    for (i=_X_; i<=_Z_; i++) {
      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])
	*(distance/sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
			sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_])));
    }
    printf("Camera closer\n");
    break;

  case 'z':
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    distance += 0.1;
    for (i=_X_; i<=_Z_; i++) {
      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])
	*(distance/sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
			sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_])));
    }
    printf("Camera further away\n");
    break;

  case GLUT_KEY_F1:  /* default posture */
  case 'r':  /* default posture */
    ++flag;
    ptr->eye[_X_]    = ptr->eye0[_X_];
    ptr->eye[_Y_]    = ptr->eye0[_Y_];
    ptr->eye[_Z_]    = ptr->eye0[_Z_];
    ptr->center[_X_] = ptr->center0[_X_];
    ptr->center[_Y_] = ptr->center0[_Y_];
    ptr->center[_Z_] = ptr->center0[_Z_];
    ptr->up[_X_]     = ptr->up0[_X_];
    ptr->up[_Y_]     = ptr->up0[_Y_];
    ptr->up[_Z_]     = ptr->up0[_Z_];
    printf("Default View\n");

    break;
 
  case 'b':
    ++flag;
    if (ptr->follow_basis==0)
      followBaseByPtr(ptr,TRUE);
    else
      followBaseByPtr(ptr,FALSE);
    break;
 
  case 'x':
    ++flag;
    if (ptr->draw_axis==0){
      ptr->draw_axis=1;
      printf("Show axes ON\n");
    }
    else  {
      ptr->draw_axis=0;
      printf("Show axes OFF\n");
    }
    break;
  
  case ' ':
    ++flag;
    togglePause();
    break;

    /* end of view postions functions */
  default:
    ptr = first_window_ptr;
    while (ptr != NULL) {
      sprintf(c,"%d",ptr->openGLId);
      if (c[0] == key)
	toggleHideWindow(ptr);
      ptr = (OpenGLWPtr) ptr->next_wptr;
    }
    break;
    
  }

  glutPostRedisplay();

  if (flag)
    printPrompt();

}

/*!*****************************************************************************
*******************************************************************************
\note  initialize_lighting
\date  August 7, 1992
   
\remarks 

initializes the light and coordinate settings for OpenGL

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     ptr : pointer to window where lighting is to be applied

******************************************************************************/
int
initializeLighting(OpenGLWPtr ptr)

{

  int i;
  GLfloat mat_specular[]   ={ (float)0.5, (float)0.5, (float)0.5, (float)1.0 };
  GLfloat light_position[] ={ (float)1.0, (float)1.0, (float)1.0, (float)0.0 };

  /* a nice position for lighting */
  glLoadIdentity();
  gluLookAt(ptr->eye[_X_],ptr->eye[_Y_],ptr->eye[_Z_],ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_],ptr->up[_X_],ptr->up[_Y_],ptr->up[_Z_]);
 	
  /* lighting settings */
  glClearColor ((float)0.9, (float)0.9, (float)1.0, (float)1.0);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, (float)50.0);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  createWindow
\date  August 7, 1992
   
\remarks 

creates an openGl window

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     wptr : pointer to window structure 

******************************************************************************/
int
createWindow(OpenGLWPtr wptr)

{

  int i;
  char *ptr;

  /* create the window */
  glutInitWindowPosition(wptr->x,wptr->y);
  glutInitWindowSize(wptr->width, wptr->height);
  wptr->openGLId = glutCreateWindow(wptr->name);  /* note that this 
						     operation make
						     the window current */
  
  /* attach appropriate OpenGL functions to the current window */
  if (wptr->idle != NULL)
    glutIdleFunc(wptr->idle);
  else
    glutIdleFunc(idle);
  	

  /* The same display function is now used for all simulations, just that
     it calls a window specific display function inside */
  glutDisplayFunc(SLGenericDisplay);
  if (wptr->display == NULL)
    wptr->display = scene_display;
  
  if (wptr->keyboard != NULL)
    glutKeyboardFunc(wptr->keyboard);
  else
    glutKeyboardFunc(keyboard);

  if (wptr->mouse != NULL)
    glutMouseFunc(wptr->mouse);
  else
    glutMouseFunc(mouse);

  if (wptr->motion != NULL)
    glutMotionFunc(wptr->motion);
  else
    glutMotionFunc(motion);
  
  if (wptr->special != NULL)
    glutSpecialFunc(wptr->special);
  else
    glutSpecialFunc(special);
  
  if (wptr->menu != NULL)
    (*wptr->menu)();
  else
    glutMenu(wptr);
 		
  /* add window to internal administration structure */
  if (first_window_ptr == NULL) {
    first_window_ptr = wptr;
  } else {
    ptr = (char *)first_window_ptr;
    while (((OpenGLWPtr)ptr)->next_wptr != NULL)
      ptr = ((OpenGLWPtr)ptr)->next_wptr;
    ((OpenGLWPtr)ptr)->next_wptr = (char *) wptr;
  }

  glutReshapeFunc(reshape);
  initializeLighting(wptr);	
 	

  return TRUE;

}


/*!*****************************************************************************
*******************************************************************************
\note  getOpenGLWindow
\date  Feb. 99
   
\remarks 

allocates memory structure for openGL window and fills in reasonable
defaults

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none 

******************************************************************************/
OpenGLWPtr
getOpenGLWindow(void)

{
  OpenGLWPtr wptr;
  static int n_windows = 0;
	
  wptr = (OpenGLWPtr) my_calloc(1, sizeof(OpenGLWindow), MY_STOP);

  wptr->x = 10;
  wptr->y = 40;
  wptr->width = 200;
  wptr->height = 200;
  wptr->fovea = 65;

  strcpy(wptr->name,robot_name);

  wptr->idle     = NULL;
  wptr->display  = scene_display;
  wptr->keyboard = keyboard;
  wptr->reshape  = NULL;
  wptr->special  = special;
  wptr->menu     = NULL;

  wptr->openGLId = -1;
  wptr->ID       = ++n_windows;
  wptr->draw_axis=  TRUE;
  wptr->follow_basis = FALSE;

  /* default view variables for the window needed to reset the view */
  wptr->eye[_X_]    = wptr->eye0[_X_]      = EYEX0;
  wptr->eye[_Y_]    = wptr->eye0[_Y_]      = EYEY0;
  wptr->eye[_Z_]    = wptr->eye0[_Z_]      = EYEZ0;
  wptr->center[_X_] = wptr->center0[_X_]   = CENTERX0;
  wptr->center[_Y_] = wptr->center0[_Y_]   = CENTERY0;
  wptr->center[_Z_] = wptr->center0[_Z_]   = CENTERZ0;
  wptr->up[_X_]     = wptr->up0[_X_]       = UPX0;
  wptr->up[_Y_]     = wptr->up0[_Y_]       = UPY0;
  wptr->up[_Z_]     = wptr->up0[_Z_]       = UPZ0;

  wptr->hide          = FALSE;

  return wptr;

}

/*!*****************************************************************************
*******************************************************************************
\note  special
\date  August 7, 1992
   
\remarks 

a special MacOS add-on to handle other than ASCII keys

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
special(int key, int x, int y)

{
  int rc,dum=0;
  	
  keyboard((unsigned char) key,dum,dum);
	
}

/*!*****************************************************************************
*******************************************************************************
\note  SLGenericDisplay
\date  Feb. 2001
   
Generic display function used for every window


*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
void
SLGenericDisplay(void)

{
  int i;
  static SL_Cstate  *basec = &base_state;
  GLfloat  objscolor[4]={(float)0.2,(float)0.2,(float)0.2,(float)1.0};
  OpenGLWPtr ptr = first_window_ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glPushMatrix();
  gluLookAt(ptr->eye[_X_],ptr->eye[_Y_],ptr->eye[_Z_],
	    ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_],
	    ptr->up[_X_],ptr->up[_Y_],ptr->up[_Z_]);
  
  /* Follow the basis, if necessary (see keys) */
  if (ptr->follow_basis) {
    glTranslatef(-basec[0].x[1],-basec[0].x[2],-basec[0].x[3]);
  }

  /* Draw the scene */
  (*ptr->display)();

  /* Draw the global axes (see keys) */
  if (ptr->draw_axis) {
    glDisable(GL_LIGHTING); /*to have constant colors */
    glColor4f (0.0,1.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(-100,0,0);       
    glVertex3f(100,0,0);  
    glEnd();
    glColor4f (1.0,0.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,-100,0);       
    glVertex3f(0,100,0);  
    glEnd();
    glColor4f (0.0,0.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,0,-100);       
    glVertex3f(0,0,100);  
    glEnd();
    glEnable(GL_LIGHTING);   
  }
                                      
  /* Allow camera movements (see keys) */ 
  glTranslatef(ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_]); 

  /* Draw the rotation point of the camera */
  if (ptr->draw_axis) {
    glDisable(GL_LIGHTING); /*to have constant colors */                
    glColor4f (0.0,1.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(-0.25,0,0);       
    glVertex3f(0.25,0,0);  
    glEnd();
    glColor4f (1.0,0.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,-0.25,0);       
    glVertex3f(0,0.25,0);  
    glEnd();
    glColor4f (0.0,0.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,0,-0.25);       
    glVertex3f(0,0,0.25);  
    glEnd();
    glEnable(GL_LIGHTING);   
  }

  glPopMatrix();

#ifdef MAKEGIF
  make_gif();
#endif

  glutSwapBuffers();

}


 
/*!*****************************************************************************
*******************************************************************************
\note  scene_display
\date  June 1999
 
\remarks 
 
a test function to display something in the case of a NULL
scene_display function
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
 
******************************************************************************/
static void
scene_display(void)
 
{
  /* clear the window */
 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutSolidTeapot((GLdouble) 1.);
  glutSwapBuffers();
 
}                                                                               


	
/*!*****************************************************************************
*******************************************************************************
\note  idle
\date  June 1999
   
\remarks 

the local idle function

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
idle(void)

{

  ;
 
}

	
/*!*****************************************************************************
*******************************************************************************
\note  glutMenu & menu_select
\date  June 1999
   
\remarks 

a simple example how to use menus and pass on the results
to the keyboard

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
menu_select(int mode)
{
  int dum=0;
	
  keyboard((unsigned char) mode, dum,dum);

}

static void
glutMenu(OpenGLWPtr ptr)
{
  char string[100];
  char c[100];

  glutCreateMenu(menu_select);
  glutAddMenuEntry("Rotation   [SHIFT+CTRL+RMB]", 'n');
  glutAddMenuEntry("Panning   [SHIFT+RMB]", 'n');
  glutAddMenuEntry("Zooming   [CTRL+RMB]", 'n');
  glutAddMenuEntry("-----------------------------------", 'n');
  glutAddMenuEntry("Toggle Pause   [space bar]", ' ');
  glutAddMenuEntry("Toggle Show Axes   [x]", 'x');
  glutAddMenuEntry("Toggle follow base coordinates   [b]", 'b');
  glutAddMenuEntry("Turn Camera Left   [left arrow]", GLUT_KEY_LEFT);
  glutAddMenuEntry("Turn Camera Right   [right arrow]", GLUT_KEY_RIGHT);
  glutAddMenuEntry("Turn Camera Upward   [up arrow]", GLUT_KEY_UP);
  glutAddMenuEntry("Turn Camera Downward   [down arrow]", GLUT_KEY_DOWN);
  glutAddMenuEntry("Zoom in   [a]", 'a');
  glutAddMenuEntry("Zoom out   [z]", 'z');
  glutAddMenuEntry("Move Focus Camera -x   [j]", 'j');
  glutAddMenuEntry("Move Focus Camera +x   [l]", 'l');
  glutAddMenuEntry("Move Focus Camera -y   [k]", 'k');
  glutAddMenuEntry("Move Focus Camera +y   [i]", 'i');
  glutAddMenuEntry("Move Focus Camera -z   [h]", 'h');
  glutAddMenuEntry("Move Focus Camera +z   [u]", 'u');
  glutAddMenuEntry("Default View   [F1,r]", GLUT_KEY_F1);
  glutAddMenuEntry("Toggle Wireframe   [t]", 't');
  glutAddMenuEntry("Quit   [q,ESC]", 'q');
  glutAddMenuEntry("-----------------------------------", 'n');
  sprintf(string,"Toggle Hide %s   [%d]",ptr->name,ptr->openGLId);
  sprintf(c,"%d",ptr->openGLId);
  glutAddMenuEntry(string,c[0]);

  glutAttachMenu(GLUT_LEFT_BUTTON);
  
}


/*!*****************************************************************************
*******************************************************************************
\note  addMenuItem
\date  June 1999
   
\remarks 

add a menu item to a particular menu

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name of the item
\param[in]     fptr : function pointer to be called with this item
\param[in]     mID  : which menu to choose


******************************************************************************/
void
addMenuItem(char *name, void (*fptr)(void), int mID) 

{

  if (mID <1 || mID>3)
    return;

  if (n_menu[mID] < MAX_ITEMS) {
    ++n_menu[mID];
    strcpy(menu[mID][n_menu[mID]],name);
    menu_ptr[n_menu[mID]] = fptr;
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  drawObjects
\date  Nov 2000
   
\remarks 

draws all objects in the environment

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none 


******************************************************************************/
void
drawObjects(void)
{

  ObjectPtr ptr;
  GLfloat  objscolor[4]={(float)0.2,(float)0.2,(float)0.2,(float)1.0};

  if (objs == NULL)
    return;

  ptr = objs;

  do {

    if (!ptr->hide) {

      objscolor[0] = ptr->rgb[1];
      objscolor[1] = ptr->rgb[2];
      objscolor[2] = ptr->rgb[3];
      glColor4fv(objscolor); 
      
      switch (ptr->type) {
	
      case CUBE:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);      
	glScaled(ptr->scale[1],ptr->scale[2],ptr->scale[3]);
	if (solid)
	  glutSolidCube(1.0);
	else
	  glutWireCube(1.0);
	glPopMatrix();
	
	break;
	
      case SPHERE:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);      
	glScaled(ptr->scale[1]/2.,ptr->scale[2]/2.,ptr->scale[3]/2.);
	if (solid)
	  glutSolidSphere(1.0,ptr->object_parms[1],ptr->object_parms[1]);
	else
	  glutWireSphere(1.0,ptr->object_parms[1],ptr->object_parms[1]);
	glPopMatrix();
	
	break;
	
      case TERRAIN:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);       
	
	if (!ptr->display_list_active)
	  createTerrainDisplayList(ptr,ptr->rgb);
	glCallList((GLuint)((unsigned long)ptr));
	glPopMatrix();
	
	break;
	
      }
      
    }

    ptr = (ObjectPtr) ptr->nptr;


  } while (ptr != NULL);


}

/*!*****************************************************************************
*******************************************************************************
\note  glutPostRedisplayAll
\date  Dec.2000
   
\remarks 

call glutPostRedisplay for all window in the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
glutPostRedisplayAll(void)
{
  OpenGLWPtr ptr = first_window_ptr;

  while (ptr != NULL) {
    if (!ptr->hide)
      glutPostWindowRedisplay(ptr->openGLId);
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 
}

/*!*****************************************************************************
*******************************************************************************
\note  whichGLWindow
\date  Dec.2000
   
\remarks 

returns the pointer to the structure of the current window

*******************************************************************************
Function Parameters: [in]=input,[out]=output

returns the current window pointer

******************************************************************************/
OpenGLWPtr
whichGLWindow(void) 
{
  int ID;
  OpenGLWPtr ptr = first_window_ptr;

  ID = glutGetWindow();

  while (ptr != NULL) {
    if (ptr->openGLId == ID)
      return ptr;
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 

  return NULL;
}


/*!*****************************************************************************
*******************************************************************************
\note  hide and show of eye windows
\date  March 2003
   
\remarks 

toggle hide status of windows

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
toggleHideWindow(OpenGLWPtr ptr)
{

  glutSetWindow(ptr->openGLId);

  if (ptr->hide) {
    glutShowWindow();
    ptr->hide = FALSE;
    printf("Show Window %s [%d]\n",ptr->name,ptr->ID);
  } else {
    ptr->hide = TRUE;
    glutHideWindow();
    printf("Hide Window %s [%d]\n",ptr->name,ptr->ID);
  }

}


/*!*****************************************************************************
*******************************************************************************
\note  hideWindowByName
\date  March 2003
   
\remarks 

toggle hide status of windows by window name

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name  : string containing the name of the window
\param[in]     hide  : TRUE/FALSE = hide or not

******************************************************************************/
void
hideWindowByName(char *name, int hide)
{
  OpenGLWPtr ptr = first_window_ptr;

  while (ptr != NULL) {
    if (strcmp(name,ptr->name)==0) {
      if (ptr->hide != hide)
	toggleHideWindow(ptr);
      return;
    }
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 
}

/*!*****************************************************************************
*******************************************************************************
\note  followBaseByName
\date  March 2003
   
\remarks 

toggle follow-base status of windows by window name

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name    : string containing the name of the window
\param[in]     follow  : TRUE/FALSE = follow or not

******************************************************************************/
void
followBaseByName(char *name, int follow)
{
  OpenGLWPtr ptr = first_window_ptr;
  int i;
  char c[100];

  while (ptr != NULL) {
    sprintf(c,"%d",ptr->openGLId);
    if (strcmp(name,c)==0) {
      followBaseByPtr(ptr,follow);
      return;
    }
    ptr = (OpenGLWPtr) ptr->next_wptr;
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  followBaseByPtr
\date  March 2003
   
\remarks 

toggle follow-base status of windows by window ptr

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     ptr     : string containing the name of the window
\param[in]     follow  : TRUE/FALSE = follow or not

******************************************************************************/
void
followBaseByPtr(OpenGLWPtr ptr, int follow)
{
  int i;

  if (follow){
    ptr->follow_basis=1;
    for (i=1; i<=N_CART; ++i)
      ptr->center[i] = 0.0;
    printf("Following the base coordinates\n");
  }
  else  {
    ptr->follow_basis=0;
    for (i=1; i<=N_CART; ++i)
      ptr->center[i] = base_state.x[i];
    printf("Return to global view\n");
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  updateAllWindows
\date  March 2003
   
\remarks 

force an update of the diplay of all windows manually

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
updateAllWindows(void)
{
  int ID;
  OpenGLWPtr ptr = first_window_ptr;

  while (ptr != NULL) {
    glutSetWindow(ptr->openGLId);
    SLGenericDisplay();
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 

}


/*!*****************************************************************************
*******************************************************************************
\note  changeWindowUpdateRate
\date  March 2003
   
\remarks 

changes the rate at which openGL windows are updated. This is
a speed-smoothness tradeoff.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
changeWindowUpdateRate(double rate)
{

  if (rate > 0)
    window_update_rate = rate;
  else
    window_update_rate = 0;

  printf("Current OpenGL window update rate is %f [hz]\n",window_update_rate);

}


/*!*****************************************************************************
*******************************************************************************
\note  drawContacts
\date  June 1999
   
\remarks 

draws all contact points and force vectors acting on the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

******************************************************************************/
void
drawContacts(double fscale)

{
  int i,j;
  GLfloat   color_point[4]={(float)1.0,(float)0.35,(float)0.35,(float)1.0};
  double    radius = 0.01;

  /* if there are no objects, exit */
  if (objs==NULL)
    return;

  for (i=0; i<=n_links; ++i) { /* loop over all contact points */

    // check whether there is an active contact
    if (!contacts[i].active || !contacts[i].status)
      continue;

    // draw a blob at the point of contact
    glPushMatrix();
    glTranslated((GLdouble)link_pos_sim[i][_X_],(GLdouble)link_pos_sim[i][_Y_],
		 (GLdouble)link_pos_sim[i][_Z_]);
    glColor4fv(color_point);
    if (solid)
      glutSolidSphere(radius,10,10);
    else
      glutWireSphere(radius,10,10);

    // draw the force and torque vector
    glDisable(GL_LIGHTING); //to have constant colors 
    glColor4f (0.0,1.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3d(0.0,0.0,0.0);       
    //printf("%f %f %f\n",contacts[i].f[_X_],contacts[i].f[_Y_],contacts[i].f[_Z_]);
    glVertex3d(contacts[i].f[_X_]*fscale,
	       contacts[i].f[_Y_]*fscale,
	       contacts[i].f[_Z_]*fscale);  
    glEnd();
    glEnable(GL_LIGHTING);   

    glPopMatrix();

  }

}

/*!*****************************************************************************
*******************************************************************************
\note  toggleClmcplotMode
\date  Nov. 2005
   
\remarks 

puts the simulation into CLMCPLOT mode, i.e., it wait for data
files .clmcplot_current_point witten by clmcplot, and just visualizes
the position state given in these files.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleClmcplotMode(void) 
{
  int i;

  if (clmcplot_mode == 0) {

    clmcplot_mode = TRUE;
    playback_mode = FALSE;
    userGraphics_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    printf("CLMCPLOT mode switched on\n");

  } else {

    clmcplot_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("CLMCPLOT mode switched off\n");

  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  toggleUserGraphicsMode
\date  Oct. 2008
   
\remarks 

puts the opengl display into "userGraphics" mode, which means that
the state of the robot can be set by user graphics functions instead
of the simulation. The relevant variables to set are base_user_state,
base_user_orient, and joint_user_state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleUserGraphicsMode(void) 
{
  int i;

  if (userGraphics_mode == 0) {

    userGraphics_mode = TRUE;
    clmcplot_mode = FALSE;
    playback_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    printf("User graphics mode switched on\n");

  } else {

    userGraphics_mode = FALSE;
    clmcplot_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("User graphics mode switched off\n");

  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  setUserGraphicsUpdateMode
\date  Oct. 2008
   
\remarks 

puts the opengl display into "userGraphics" mode, which means that
the state of the robot can be set by user graphics functions instead
of the simulation. The relevant variables to set are base_user_state,
base_user_orient, and joint_user_state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
setUserGraphicsUpdateMode(int mode) 
{
  int i;
  if (mode)
    mode = TRUE;
  else
    mode = FALSE;
  if (mode!=userGraphics_mode)
    toggleUserGraphicsMode();
}

/*!*****************************************************************************
*******************************************************************************
\note  togglePlaybackMode
\date  Nov. 2005
   
\remarks 

puts the simulation into PLAYBACK mode, i.e., opens a data file
and visualizes the state that should be store in this file frame
by frame.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
togglePlaybackMode(void) 
{
  int i,rc;
  char fname[100];
  FILE *in;
  int   file_number = 0;

  if (playback_mode == 0) {

    if ( ( in = fopen( ".last_data", "r" ) ) != NULL )   {
      rc=fscanf( in, "%d\n", &file_number );
      fclose( in );
    }
    sprintf( fname, "d%05d", file_number-1 );

    while (TRUE) {

      if (!get_string("Filename for playback",fname,fname))
	return;

      // try to read the file
      if (playback_D != NULL) {
	my_free_matrix(playback_D,1,playback_n_rows,1,playback_n_cols);
	playback_D = NULL;
      }
      if (clmcplot_convert(fname,&playback_D,&playback_vnames,&playback_units,
			   &playback_freq,&playback_n_cols,&playback_n_rows))
	break;

    }

    playback_mode = TRUE;
    clmcplot_mode = FALSE;
    userGraphics_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    playback_current_row   = 1;
    strcpy(playback_file_name,fname);

    printf("PLAYBACK mode switched on\n");

  } else {

    playback_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("PLAYBACK mode switched off\n");

  }
  
  // reset simulation 
  playback_step_mode = FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  reset_playback
\date  Sept. 2006
   
\remarks 

resets the framecount of play file to 1

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
reset_playback(void) 
{
  if (!playback_mode)
    return;

  playback_current_row = 1;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  go_playback
\date  Sept. 2006
   
\remarks 

runs playback mode without stepping

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
go_playback(void) 
{
  if (!playback_mode)
    return;

  playback_step_mode = FALSE;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  step_playback
\date  Sept. 2006
   
\remarks 

advances one step in playback frame

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
step_playback(void) 
{
  if (!playback_mode)
    return;

  playback_step_mode = TRUE;

  if (playback_current_row <= playback_n_rows-playback_step_size)
    playback_current_row += playback_step_size;
  else 
    playback_current_row = 1;

  if (playback_current_row < 1)
    playback_current_row = playback_n_rows;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  stepsize_playback
\date  Sept. 2006
   
\remarks 

sets playback stepsize

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
stepsize_playback(void) 
{
  if (!playback_mode)
    return;

  get_int("Playback step size",playback_step_size,&playback_step_size);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  new_playback
\date  Sept. 2006
   
\remarks 

reads a new playback file

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
new_playback(void) 
{
  char fname[100];
  FILE *in;
  int   file_number = 0;
  Matrix Dold;
  int rc;

  if (!playback_mode)
    return;

  if ( ( in = fopen( ".last_data", "r" ) ) != NULL )   {
    rc=fscanf( in, "%d\n", &file_number );
    fclose( in );
  }
  sprintf( fname, "d%05d", file_number-1 );
  
  while (TRUE) {
    
    if (!get_string("Filename for playback",fname,fname))
      return;
    
    // try to read the file
    Dold = playback_D;
    playback_D = NULL;
    if (clmcplot_convert(fname,&playback_D,&playback_vnames,&playback_units,&playback_freq,
			 &playback_n_cols,&playback_n_rows)) {
      if (Dold != NULL) 
	my_free_matrix(Dold,1,playback_n_rows,1,playback_n_cols);
      break;
    }
    playback_D = Dold;
    
  }
  
  playback_current_row   = 1;
  strcpy(playback_file_name,fname);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  userGraphicsUpdateState
\date  Oct 2008
   
\remarks 

Copies the user graphics updated robot states into the real ones

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
userGraphicsUpdateState(void) 
{
  int i;
  for (i=1; i<=n_dofs; i++)
    joint_sim_state[i] = userGraphics_joint_state[i];
  base_state = userGraphics_base_state;
  base_orient = userGraphics_base_orient;
}

/*!*****************************************************************************
*******************************************************************************
\note  playbackUpdateState
\date  March 2006
   
\remarks 

visualizes the next data record

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
playbackUpdateState(void) 
{

  // update the state from the current data record
  extractState(playback_vnames,playback_D[playback_current_row],playback_n_cols);

  if (!playback_step_mode) {
    playback_current_row += playback_step_size;
    if (playback_current_row > playback_n_rows)
      playback_current_row = 1;
    if (playback_current_row < 1)
      playback_current_row = playback_n_rows;
  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  clmcplotUpdateState
\date  March 2006

\remarks 

tries to read the .clmcplot_current_point, and if successful, 
passes the data on to a parser that extracts the current robot
state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
clmcplotUpdateState(void) 
{
  int i,j,rc;
  char fname[]=".clmcplot_current_point";
  char string[100];
  FILE *fp;
  Matrix Dold;

  if ((fp=fopen(fname,"r")) == NULL)
    return;

  rc=fscanf(fp,"%s %d",string,&playback_current_row);

  fclose(fp);

  // erase the file if successful, such that we notice an update
  remove(fname);

  // try to read the file
  if (strcmp(string,playback_file_name) != 0) {
    Dold = playback_D;
    playback_D = NULL;
    if (!clmcplot_convert(string,&playback_D,&playback_vnames,&playback_units,
			  &playback_freq,&playback_n_cols,&playback_n_rows)) {
      playback_D = Dold;
      return;
    }
    if (Dold != NULL)
      my_free_matrix(Dold,1,playback_n_rows,1,playback_n_cols);
    strcpy(playback_file_name,string);
    printf("clmcplot uses file %s\n",playback_file_name);
  }

  // update the state from the current data record
  extractState(playback_vnames,playback_D[playback_current_row],playback_n_cols);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  extractState
\date  March 2006
   
\remarks 

This function searches for position state variables of the
robot. Each found position (joint_state, base_state,
base_orient) is used to overwrite the simulation state such
that the current point of clmcplot is visualized.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     vnames : the variable names in this data set
\param[in]     D      : one data vector
\param[in]     n_cols : number of elements in data vector

******************************************************************************/
static void 
extractState(char **vnames, Vector D, int n_cols) 
{
  int i,j;
  char   string[100];

  // search for state variables
  for (i=1; i<=n_dofs; ++i) {
    sprintf(string,"%s_th",joint_names[i]);
    joint_sim_state[i].th = joint_default_state[i].th;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	joint_sim_state[i].th = D[j];
	break;
      }
    }
  }

  for (i=1; i<=n_links; ++i) {
    sprintf(string,"%s_cstat",link_names[i]);
    contacts[i].status = FALSE;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].status = D[j];
	break;
      }
    }
  }

  for (i=1; i<=n_links; ++i) {
    sprintf(string,"%s_cfx",link_names[i]);
    contacts[i].f[_X_] = 0.0;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_X_] = D[j];
	break;
      }
    }
  }

  for (i=1; i<=n_links; ++i) {
    sprintf(string,"%s_cfy",link_names[i]);
    contacts[i].f[_Y_] = 0.0;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_Y_] = D[j];
	break;
      }
    }
  }

  for (i=1; i<=n_links; ++i) {
    sprintf(string,"%s_cfz",link_names[i]);
    contacts[i].f[_Z_] = 0.0;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_Z_] = D[j];
	break;
      }
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_x",vnames[j])==0) {
      base_state.x[_X_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_y",vnames[j])==0) {
      base_state.x[_Y_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_z",vnames[j])==0) {
      base_state.x[_Z_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q0",vnames[j])==0) {
      base_orient.q[_Q0_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q1",vnames[j])==0) {
      base_orient.q[_Q1_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q2",vnames[j])==0) {
      base_orient.q[_Q2_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q3",vnames[j])==0) {
      base_orient.q[_Q3_] = D[j];
      break;
    }
  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  createTerrainDisplayList
\date  March 2006
   
\remarks 

For terrain objects, an openGL display list is generated

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     vnames : the variable names in this data set
\param[in]     D      : one data vector
\param[in]     n_cols : number of elements in data vector
\param[in]     rgb    : terrain color

******************************************************************************/
static void
createTerrainDisplayList(ObjectPtr ptr, double *rgb) 
{
  int     i,j,k;
  SL_quat q;
  double  x_min,x_max,y_min,y_max;
  int     n_steps_x;
  int     n_steps_y;
  double  display_grid_delta_x; 
  double  display_grid_delta_y; 
  double  display_grid_delta; 
  char    fname[100];
  double  x,y,z,no_go;
  double  n[N_CART+1];
  GLfloat objscolor[4];

  // for terrains we need to create a display list for this terrain
  if (ptr->type != TERRAIN)
    return;

  if (ptr->display_list_active)
    return;

  // get min and max of the terrain board
  sprintf(fname,"%s.asc",ptr->name); 
  if (!getContactTerrainMinMax(fname,&x_min,&x_max,&y_min,&y_max))
    return;

  // create a display list for this terrain
  display_grid_delta = ptr->object_parms[4];
  n_steps_x = ceil((x_max-x_min)/display_grid_delta);
  n_steps_y = ceil((y_max-y_min)/display_grid_delta);
  display_grid_delta_x = (x_max-x_min)/(double)n_steps_x;
  display_grid_delta_y = (y_max-y_min)/(double)n_steps_y;
  
  glNewList((GLuint)((unsigned long) ptr), GL_COMPILE);
  
  for (i=1; i<=n_steps_x; ++i) {
    x = i*display_grid_delta_x+x_min;
    
    for (j=1; j<=n_steps_y; ++j) {
      y = j*display_grid_delta_y + y_min;
      
      if (!getContactTerrainInfo(x, y, fname, &z, n, &no_go))
	continue;
      
      for (k=0; k<3; ++k)
	objscolor[k]=rgb[k+1];
      objscolor[3] = 1.0;

      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      
      glBegin(GL_POLYGON);	    
      
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      x -= display_grid_delta_x;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      y -= display_grid_delta_y;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      x += display_grid_delta_x;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      glEnd();
    }
    
  }
  
  glEndList();

  ptr->display_list_active = TRUE;
  
}


/*!*****************************************************************************
*******************************************************************************
\note  checkWindowHide
\date  June 1999
   
\remarks 

a function that allows hide/display of windows from the command line
interface

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name  : name of the window

******************************************************************************/
static int
checkWindowHide(char *name)

{
  int i,rc;
  OpenGLWPtr ptr = first_window_ptr;
  char c[100];

  /* check whether command is for window hide status */
  while (ptr != NULL) {
    sprintf(c,"%d",ptr->ID);
    if (c[0]==name[0]) {
      toggleHideWindow(ptr);
      return TRUE;
    }
    ptr = (OpenGLWPtr)ptr->next_wptr;
  }

  return FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  togglePause
\date  Nov. 2007
   
\remarks 

toggles pause flag

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
togglePause(void)
{
  if (pause_flag==FALSE){
    pause_flag=TRUE;
    semGive(sm_pause_sem);
    printf("Pausing the simulation\n");
    semFlush(sm_openGL_servo_sem);
  }
  else  {
    pause_flag=FALSE;
    clmcplot_mode = FALSE;
    playback_mode = FALSE;
    semGive(sm_pause_sem);
    printf("Resuming the simulation\n");
    semFlush(sm_openGL_servo_sem);
  }
}
