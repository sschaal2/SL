/*!=============================================================================
  ==============================================================================

  \file    SL_kinect_vision.c

  \author  Stefan Schaal & Jeannette Bohg
  \date    Oct. 2014

  ==============================================================================
  \remarks

  handles the I/O with a generic Kinect Vision System providing blob positions
  through socket communication

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_vision_servo.h"
#include "SL_socket_unix.h"
#include "fcntl.h"

#include "poledata.h"

#define ANGLE       1
#define BLOB_UPPER  2
#define BLOB_LOWER  3

#define N_BLOBS  3

#define CAMERA_1  1
#define N_CAMERAS  1

#define WAIT_IN_NS 100000

// global variables
typedef struct Frame {
  int counter;  /*!< the frame counter */
  PoleData *pole;
} Frame;

static Frame           the_frame;
static int             last_counter;
static int             frame_counter;
static int             socket_fd;
int                    numbytes;


// local functions
static int init_kinect_vision_interface(void);
//static int  read_frame(char *,int );

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_hardware
\date  June 1999
   
\remarks 

        initializes communication with the DBVision system

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_vision_hardware(void)
{
  int i;

  printf("Initializing vision hardware ...");

  // allocate memory for the blobs
  the_frame.pole = my_calloc(1,sizeof(PoleData),MY_STOP);
  
  frame_counter = 0;
  the_frame.counter = 0;

  // initialize the interface
  if (!init_kinect_vision_interface())
    return FALSE;

  printf("done\n");  

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_dbvision_interface
\date  Oct 2000
   
\remarks 

        initializes the interface, i.e., just a serial connection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int
init_kinect_vision_interface(void)
{

  socket_fd = open_socket();
  if (socket_fd < 0) {
    printf("Error when opening Socket\n");
    return FALSE;
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  acquire_blobs
\date  Oct 2000

\remarks 

read the current information about the blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     blobs  : array of blob structures

 ******************************************************************************/
#define MAX_CHARS 10000
int
acquire_blobs(Blob3D blobs[])
{
  int    i,j,r,m;
  int    rc;
  static char buffer[MAX_CHARS];
  static int  n_buffer = 0; 
  char   buffer2[MAX_CHARS];
  int    n_buffer2 = 0;
  int    count = 0;
  int    run = TRUE;
  int numbytes;

  /* get the last frame id */
  last_counter = the_frame.counter;

  // TODO: Discuss this quesiton with Stefan
  // should this be a blocking call to make use of the kinect irregular clock?
  // or should it be a non-blocking call and just continue polling for information?
  if((numbytes = read_socket(socket_fd, sizeof(PoleData), (char *)the_frame.pole)) == -1 )
    {
      // no new data available
      // reusing the 3D blob structure 
      // TODO: replace this with an appropriate structure
      for (i=0; i<N_BLOBS; ++i)
	blobs[i].status = FALSE;
      
      ++frame_counter;
    } 
    else
    {
      // reusing the 3D blob structure 
      // TODO: replace this with an appropriate structure
      blobs[ANGLE].status = TRUE;
      blobs[ANGLE].x[_X_] = the_frame.pole->pole_angle;
      blobs[ANGLE].x[_Y_] = the_frame.pole->pole_angular_velocity;
      blobs[ANGLE].x[_Z_] = -1.0;

      blobs[BLOB_UPPER].status = TRUE;
      blobs[BLOB_UPPER].x[_X_] = the_frame.pole->pos_upper[0];
      blobs[BLOB_UPPER].x[_Y_] = the_frame.pole->pos_upper[1];
      blobs[BLOB_UPPER].x[_Z_] = the_frame.pole->pos_upper[2];

      blobs[BLOB_LOWER].status = TRUE;
      blobs[BLOB_LOWER].x[_X_] = the_frame.pole->pos_lower[0];
      blobs[BLOB_LOWER].x[_Y_] = the_frame.pole->pos_lower[1];
      blobs[BLOB_LOWER].x[_Z_] = the_frame.pole->pos_lower[2];
      
      /* DEBUG 
      printf("listener: pole_angle %.2f \t position upper: %.2f %.2f %.2f\t position lower: %.2f %.2f %.2f\n", 
	     the_frame.pole->pole_angle,
	     the_frame.pole->pos_upper[0], the_frame.pole->pos_upper[1], the_frame.pole->pos_upper[2],
	     the_frame.pole->pos_lower[0], the_frame.pole->pos_lower[1], the_frame.pole->pos_lower[2]);
      */
  
      count_lost_frames += abs(frame_counter - the_frame.counter);
    
      the_frame.counter == last_counter + 1;
      frame_counter = the_frame.counter;
    }

  ++count_all_frames;

  taskDelay(ns2ticks(WAIT_IN_NS));
  return TRUE;
}

