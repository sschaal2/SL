/*!=============================================================================
  ==============================================================================

  \file    SL_newtonlabs.c

  \author  Stefan Schaal
  \date    Oct. 2000

  ==============================================================================
  \remarks

  handles the I/O with the NewtonLab vision system

  ============================================================================*/

#include "vxWorks.h"
#include "stdio.h"
#include "string.h"
#include "intLib.h"
#include "timers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "utility.h"
#include "SL_vision_servo.h"
#include "SL_vx_serial.h"

#define N_TIME_OUT 5000    /*!< a counter to detect time out */
                           /*! 200 seems to be minimum to not forget a frame */
#define CAMERA_1  1
#define CAMERA_2  2
#define N_CAMERAS 2        /*!< binocular */

#define nanosleep(x,y) {int ii; for (ii=1; ii<=10; ++ii);};

typedef struct Blob2DInfo {
  int status;
  int x;
  int y;
  int area;
  int left;
  int top;
  int right;
  int bottom;
  int aspect_ratio;
  int orient;
} Blob2DInfo;

typedef struct Frame {
  int counter;  /*!< the frame counter */
  Blob2DInfo blobinfo[MAX_BLOBS+1][N_CAMERAS+1];
} Frame;

static Frame           the_frame;
static int             last_counter;
static int             frame_counter;
static struct timespec nsleep;
static int             serial_fd;

/* global variables */

/* local functions */
static int  init_newtonlabs_interface(void);
static int  read_frame(void);

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_hardware
\date  June 1999
   
\remarks 

        initializes communication with the QuickMag system

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_vision_hardware(void)
{
  
  /* initialize the interface */
  if (!init_newtonlabs_interface())
    return FALSE;

  /* the sleep structure */
  nsleep.tv_sec  = 0;
  nsleep.tv_nsec = 100;
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_newtonlabs_interface
\date  Oct 2000
   
\remarks 

        initializes the interface, i.e., just a serial connection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int
init_newtonlabs_interface(void)
{

  serial_fd = open_serial(SERIALPORT2,BAUD115K,"ro",TRUE);
  if (serial_fd == FALSE) {
    printf("Error when opening Serial Port\n");
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
int
acquire_blobs(Blob2D blobs[][2+1])

{
  int   i,j;
  int   rc;
  char  buffer[2];
  int   start_time = 0;
  
  /* reset all the blobs to be non existent */

  for (i = 1; i<=MAX_BLOBS; ++i) {
    the_frame.blobinfo[i][CAMERA_1].status = FALSE;
    the_frame.blobinfo[i][CAMERA_2].status = FALSE;
  }

  start_time = (int) time(NULL); 
  
  while( TRUE ) {		/* Wait for data in the serial buffer */
    if (check_serial(serial_fd) > 0) {
      if (read_serial(serial_fd,1,buffer)==1)
	if (buffer[0] == '^')  /* this is the start of frame character */
	  break;
    }
    nanosleep(&nsleep,NULL);
    if (abs(start_time - (int) time(NULL)) > 10) {
      printf("Error: Timeout when reading from vision hardware\n");
      printf("Switch to no-hardware mode\n");
      no_hardware_flag = TRUE;
      return TRUE;
    }
  }

  /* now we are at the beginning of a frame */
  last_counter = the_frame.counter;
  
   /* read the entire frame */
  rc = read_frame();

  ++frame_counter;

  if (vision_servo_calls == 0) {
    /* synchronize the v->frame_counter and the_frame.counter */
    frame_counter = the_frame.counter;
    last_counter = frame_counter - 1;
  }

  ++count_all_frames;
  
  /* frame_counter and the_frame.counter should always coincide,
     anything else counts as a lost frame */
  
  count_lost_frames += abs(frame_counter - the_frame.counter);
  frame_counter = the_frame.counter;

  /* was the frame counter advanced and was read_frame successful ?*/

  if ( (the_frame.counter == last_counter + 1) && rc ) {
    ;
  } else {
    
    /* reset all the blobs to be non existent */
    for (i = 1; i<=MAX_BLOBS; ++i) {
      the_frame.blobinfo[i][CAMERA_1].status = FALSE;
      the_frame.blobinfo[i][CAMERA_2].status = FALSE;
    }
    ++count_lost_frames;
  }
  
  /* copy the data into the global structures */

  for (i=1; i<=MAX_BLOBS; ++i) {
    /*
    if (the_frame.blobinfo[i][CAMERA_1].status &&
	the_frame.blobinfo[i][CAMERA_2].status) {

      blobs[i][CAMERA_1].status = blobs[i][CAMERA_2].status = TRUE;
      blobs[i][CAMERA_1].x[_X_] = the_frame.blobinfo[i][CAMERA_1].x;
      blobs[i][CAMERA_1].x[_Y_] = the_frame.blobinfo[i][CAMERA_1].y;
      blobs[i][CAMERA_2].x[_X_] = the_frame.blobinfo[i][CAMERA_2].x;
      blobs[i][CAMERA_2].x[_Y_] = the_frame.blobinfo[i][CAMERA_2].y;

    } else {

      blobs[i][CAMERA_1].status = blobs[i][CAMERA_2].status = FALSE;

      } */

    if (the_frame.blobinfo[i][CAMERA_1].status) {
      blobs[i][CAMERA_1].status = TRUE;
      blobs[i][CAMERA_1].x[_X_] = the_frame.blobinfo[i][CAMERA_1].x;
      blobs[i][CAMERA_1].x[_Y_] = the_frame.blobinfo[i][CAMERA_1].y;
    } else {
      blobs[i][CAMERA_1].status = FALSE;
    }
    if (the_frame.blobinfo[i][CAMERA_2].status) {
      blobs[i][CAMERA_2].status = TRUE;
      blobs[i][CAMERA_2].x[_X_] = the_frame.blobinfo[i][CAMERA_2].x;
      blobs[i][CAMERA_2].x[_Y_] = the_frame.blobinfo[i][CAMERA_2].y;
    } else {
      blobs[i][CAMERA_2].status = FALSE;
    }
  }

  return TRUE;

}  

/*!*****************************************************************************
 *******************************************************************************
\note  read_frame
\date  Oct 2000
   
\remarks 

        Reads a frame from the serial port. 
	Note: all date comes as ASCII characters!!! Weird .....

	The following characters demark the frame:
	start of frame   : ^
	end of frame     : \n
	start of camera 2: /
	end of channel   : ;
	end of number    : space
	
	For the following readings, the NewtonLabs hardware needs to
	be set to: 
	- report frame counter (otherwise this function reports errors)
	- only report x and y of the blobs, no other data
          (this could be changed if needed)
        - only allow one object per channel 
          (can be improved in the future, too, but requires to use
           temporal reasoning to avoid confusing two objects with
           the same color, and also occlusions)

	Currently, blob numbering coincides with the the color channel
        number.

        Thus, an empty frame would look like:
        1213;;;;/;;;;/

        A valid frame with one blob would looke like:
        4475460 356 269;;;;/238 174;;;;/

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none
     returns the number of bytes read

 ******************************************************************************/
static int
read_frame(void)

{
  int   i,j;
  char *start_camera[N_CAMERAS+1];
  char *startbuf;
  int   rc;
  char  buffer[1000];
  char  store_char;
  char *cptr;

  while (TRUE) {
    if ((rc=check_serial(serial_fd)) > 0) {
      rc=read_serial(serial_fd,rc,buffer);
      if (buffer[rc-1]=='\n')
	break;
      else
	printf("Last character was not CR in LINE mode serial connection\n");
    }
    nanosleep(&nsleep,NULL);
  }

  /* where is the start of camera 2 data: at the first "/"  */
  start_camera[CAMERA_2] = strchr(buffer,'/');
  if (start_camera[CAMERA_2]==NULL) {
    printf("Invalid Frame: / character not found\n");
    printf("Frame = >%s<\n",buffer);
    return FALSE;
  }
  *(start_camera[CAMERA_2]) = '\0';
  ++start_camera[CAMERA_2];

  /* where is the start of camera 1 data: more tricky: it is 
     before the first ";", and after the first space, if blob1
     exists */
  cptr = strchr(buffer,';');
  start_camera[CAMERA_1] = strchr(buffer,' ');
  if (start_camera[CAMERA_1]==NULL) 
    start_camera[CAMERA_1]=cptr;
  else
    if (cptr<start_camera[CAMERA_1])
      start_camera[CAMERA_1]=cptr;
  store_char = *(start_camera[CAMERA_1]);
  *(start_camera[CAMERA_1])='\0';
  
  /* read the frame counter and the first blob of camera 1 */
  if (sscanf(buffer,"%d",&the_frame.counter) == 0) {
    printf("Invalid Frame: frame counter not found\n");
    return FALSE;
  }
  *(start_camera[CAMERA_1])=store_char;

  /* loop through both cameras and parse the data */

  for (i=1; i<=N_CAMERAS; ++i) {
    startbuf=start_camera[i];
    for (j=1; j<=MAX_BLOBS; ++j) {

      /* find the next semicolon */
      cptr = strchr(startbuf,';');
      if (cptr==NULL)
	break;
      *cptr = '\0';

      /* read the data */
      if (sscanf(startbuf,"%d %d",&the_frame.blobinfo[j][i].x,
		 &the_frame.blobinfo[j][i].y) != 2)
	the_frame.blobinfo[j][i].status = FALSE;
      else
	the_frame.blobinfo[j][i].status = TRUE;

      startbuf = cptr+1;
	
    }
  }

  return TRUE;
  
}




