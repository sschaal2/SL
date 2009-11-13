/*!=============================================================================
  ==============================================================================

  \file    SL_quickmag.c

  \author  Tomohiro Shibata
  \date    June 2000 Stefan June 1999

  ==============================================================================
  \remarks

      handles the I/O with the QuickMag system

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
#include "dioDrv.h"
#include "PCIlib.h"


/*! QUICK MAG decoding */
#define VD  (0x2)
#define STB (0x1)

#define nanosleep(x,y) {int ii; for (ii=1; ii<=10; ++ii);};

/*! these are rough offset to make quick mag coordinates coindide
   with robot coordinates to make debugging simpler.
   Read these offsets from QuickMag recorded data from the 
   PC interface */
#define QUICK_MAG_X_OFFSET  ( 0.150/2.)
#define QUICK_MAG_Y_OFFSET  ( 0.398/2.)
#define QUICK_MAG_Z_OFFSET  (-0.280/2.)


#define N_BYTES_PER_FRAME 2+MAX_BLOBS*14
#define N_TIME_OUT 5000    /*!< a counter to detect time out */
                           /* 200 seems to be minimum to not forget a frame */

#define DLEN 5             /*!< data length for one blob */
#define HLEN 2             /*!< header length */
#define N_CAMERAS 2        /*!< binocular */
#define N_BYTES_PER_FRAME_2D  (HLEN+MAX_BLOBS*N_CAMERAS*DLEN)


/* variables to read from parallel port */
static unsigned char csr, data;

/* note: the status codes are:
   window     object
   0x00  none       none
   0x04  exist      none
   0x05  exist      exist
   0x06  tracking   none
   0x07  tracking   exist
   
   thus, the only positive status for us is 0x07
*/

typedef struct Channel {
  unsigned char status_1;  /*!< status camera 1 */
  unsigned char status_2;  /*!< status camera 2 */
  float x;                 /*!< x position blob */
  float y;                 /*!< y position blob */
  float z;                 /*!< z position blob */
} Channel;

typedef struct Frame {
  unsigned short counter;  /*!< the frame counter */
  Channel channels[MAX_BLOBS*N_CAMERAS];
} Frame;

static Frame       the_frame;
static int         last_counter;
static int         frame_counter;
static long       *fpdp_read_reg;
static struct timespec nsleep;

/* global variables */

/* local functions */
static int  init_quickmag_interface(void);
static int  read_frame(void);
static int  read_frame2D(void);
static int  acquire_blobs3D(Blob2D blobs[][2+1]);
static int  acquire_blobs2D(Blob2D blobs[][2+1]);

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
  if (!init_quickmag_interface())
    return FALSE;

  /* the sleep structure */
  nsleep.tv_sec  = 0;
  nsleep.tv_nsec = 100;
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_quickmag_interface
\date  June 1999
   
\remarks 

        initializes the interface

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int
init_quickmag_interface(void)
{
  char  *DIO_Board_Name = {"DIO_PMC"};
  long  DIO_Fd = 0; 
  extern DIO_DEV *dioDev;
  
  /* initialize the board */
  if (InstallPMC(DIO_Board_Name) != 0)
    return FALSE;
  
  /* open the board for read/write */
  DIO_Fd = open(DIO_Board_Name, O_RDWR, 0);              
  
  /* set both 16bit words to write */
  ioctl(DIO_Fd, SET_DIO_IO_MODE, WRITE_WRITE);  
  fpdp_read_reg = 
    (long *)((dioDev->DIO_Base + OUTPUT_REG) | _60x_TO_PCI_MAP);
  *fpdp_read_reg = 0x0;
    
  /* set both 16bit words to read */
  ioctl(DIO_Fd, SET_DIO_IO_MODE, READ_READ);  

  fpdp_read_reg = 
    (long *)((dioDev->DIO_Base + INPUT_REG) | _60x_TO_PCI_MAP);

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_csr
\date  July 1994

\remarks 

reads the status register from the printer port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static int
read_csr(void )
{
  
  return (int) (((LONGSWAP(*fpdp_read_reg))>>8) & 0x3);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_data
\date  July 1994

\remarks 

reads the data from the printer port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static int
read_data(void)
{
  
  return (int) ((LONGSWAP(*fpdp_read_reg)) & 0xff);
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  acquire_blobs
\date  July 1994

\remarks 

read the current information about the blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     blobs  : array of blob structures

 ******************************************************************************/
int
acquire_blobs(Blob2D blobs[][2+1])

{
  int i,j,q;
  extern int stereo_mode;

  /* check the stereo mode an branch accordingly */
  switch (stereo_mode) {
  case VISION_3D_MODE:
    return acquire_blobs3D(raw_blobs2D);
    
  case VISION_2D_MODE:
    return acquire_blobs2D(raw_blobs2D);

  }

  return TRUE;

}  

/*!*****************************************************************************
 *******************************************************************************
\note  acquire_blobs3D
\date  July 1994

\remarks 

read the current information about the blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     blobs  : array of blob structures

 ******************************************************************************/
static int
acquire_blobs3D(Blob2D blobs[][2+1])

{
  int i,j,q;
  int k;
  int b;
  int rc;


 /* reset all the blobs to be non existent, such that
     when the program hangs in the while loops below because
     of no VD signal comes, the blobs are properly invalid  */
  
  for (i = 1; i<=MAX_BLOBS; ++i) {
    the_frame.channels[i-1].status_1 = 0x00;
    the_frame.channels[i-1].status_2 = 0x00;
  }
  
  
  while( TRUE ) {		/* Wait for VD to go low */
    csr = read_csr();
    if ( !( csr & VD ) ) break;
    nanosleep(&nsleep,NULL);
  }

  while( TRUE ) {		/* Wait for VD to go high */
    csr = read_csr();
    if ( csr & VD ) break;
    nanosleep(&nsleep,NULL);
  }
  
  /* immediately after the VD went low, read the frame */
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

  /* was the frame counter advanced and was the number of bytes
     correct ? */

  if ( (the_frame.counter == last_counter + 1) && (rc-2)%14 == 0) {
    ;
  } else {
    
    /* reset all the blobs to be non existent */
    for (i = 1; i<=MAX_BLOBS; ++i) {
      the_frame.channels[i-1].status_1 = 0x00;
      the_frame.channels[i-1].status_2 = 0x00;
    }
    ++count_lost_frames;
  }
  
  /* copy the data into the global structures */

  for (i=1; i<=MAX_BLOBS; ++i) {
    
    if (the_frame.channels[i-1].status_1 == 0x07 &&
	the_frame.channels[i-1].status_2 == 0x07) {

      blobs[i][1].status = blobs[i][2].status = TRUE;
      blobs[i][1].x[_X_] = -(the_frame.channels[i-1].y-QUICK_MAG_Y_OFFSET);
      blobs[i][1].x[_Y_] =   the_frame.channels[i-1].z-QUICK_MAG_Z_OFFSET;
      blobs[i][2].x[_X_] =   the_frame.channels[i-1].x-QUICK_MAG_X_OFFSET;
      blobs[i][2].x[_Y_] =   the_frame.channels[i-1].z-QUICK_MAG_Z_OFFSET;

    } else {

      blobs[i][1].status = blobs[i][2].status = FALSE;

    }
  }

  return TRUE;

}  

/*!*****************************************************************************
 *******************************************************************************
\note  read_frame
\date  July 1994
   
\remarks 

        Reads a frame from the parallel board. Every 20 micro seconds
	a byte is read. One channel needs 14 bytes to be read. The
	program tries to read MAX_BLOBS channels, but uses a TIME_OUT
	to find out that no more data is sent. Adjust N_TIME_OUT for
	other CPUs!!!!!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none
     returns the number of bytes read

 ******************************************************************************/
static int
read_frame(void)

{
  int byte_counter;
  int iaux;
  int i,q;
  int time_out_counter;
  int lock;
  int channel;
  unsigned int int_to_float, *int_to_float_pointer;
  unsigned int temp;
  
  int_to_float_pointer = &int_to_float;

  /* lock out any other interupt on this processor to avoid losing bytes */
  lock = intLock();
  
  for( byte_counter = 1; byte_counter <= N_BYTES_PER_FRAME; byte_counter++ )   {
    
    time_out_counter = 0;
    while( TRUE ) {             /* Wait for STB to go high */
      csr = read_csr();
      if ( csr & STB ) break;
      if ( ++time_out_counter > N_TIME_OUT && byte_counter > 2) {
	intUnlock(lock);
	return byte_counter-1;
      }
     }
      
    time_out_counter = 0;
    while( TRUE ) {            /* Wait for STB to go low */
      csr = read_csr();
      if ( !( csr & STB ) )  break;
      if ( ++time_out_counter > N_TIME_OUT && byte_counter > 2) {
	intUnlock(lock);
	return byte_counter-1;
      }	
    }

    /* the following decoding is used:
       1.Byte: low byte of frame counter  
       2.Byte: high byte of frame counter
       note: order of low and high byte must be reversed
       
       in blocks of 14 bytes, each blob is decoded:
       
       1.Byte: status 2
       2.Byte: status 1
       3.-6  : x-position, must reverse order for proper decoding on 68040
       7.-10 : y-position, must reverse order for proper decoding on 68040
       11.-14: z-position, must reverse order for proper decoding on 68040
       
       */
    
    /* iaux reflects the current number in the byte count for blob */
    
    iaux = (byte_counter-3) % 14;
    channel = (byte_counter-3)/14; 

    if ( byte_counter <= 2 )
      {
	switch( byte_counter )
	  {
	  case 1:
	    the_frame.counter = read_data();
	    break;
	  case 2:
	    the_frame.counter += (read_data() << 8);
	    break;
	  default:
	    fprintf( stderr, "Whoops, byte_counter: %d\n", byte_counter );
	    break;
	  }
	continue;
      }
    switch( iaux )
      {
      case 0:
	the_frame.channels[channel].status_2 = read_data();
	break;
      case 1:
	the_frame.channels[channel].status_1 = read_data();
	break;
      case 2:
	int_to_float = read_data();
	break;
      case 3:
	int_to_float += read_data() << 8;
	break;
      case 4:
	int_to_float += read_data() << 16;
	break;
      case 5:
	int_to_float += read_data() << 24;
	the_frame.channels[channel].x = *((float *) int_to_float_pointer);
	break;
      case 6:
	int_to_float = read_data();
	break;
      case 7:
	int_to_float += read_data() << 8;
	break;
      case 8:
	int_to_float += read_data() << 16;
	break;
      case 9:
	int_to_float += read_data() << 24;
	the_frame.channels[channel].y = *((float *) int_to_float_pointer);
	break;
      case 10:
	int_to_float = read_data();
	break;
      case 11:
	int_to_float += read_data() << 8;
	break;
      case 12:
	int_to_float += read_data() << 16;
	break;
      case 13:
	int_to_float += read_data() << 24;
	the_frame.channels[channel].z = *((float *) int_to_float_pointer);
	break;
      default:
	fprintf( stderr, "Whoops, iaux: %d\n", iaux );
	break;
      }
  }
  
  intUnlock(lock);
  return byte_counter-1;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_uchar
\date  July 1994

\remarks 

reads the data from the printer port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static int
read_uchar (void)
{
  
  return (unsigned char) ((LONGSWAP(*fpdp_read_reg)) & 0xff);
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  acquire_blobs2D
\date  Jun 2000

\remarks 

read the current 2D information about the blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     blobs  : array of blob structures

 ******************************************************************************/
static int
acquire_blobs2D(Blob2D blobs[][2+1])

{
  int i,j,q;
  int k;
  int b;
  int rc;

  
  /* reset all the blobs to be non existent, such that
     when the program hangs in the while loops below because
     of no VD signal comes, the blobs are properly invalid  */
  
  for (i = 1; i<=MAX_BLOBS*N_CAMERAS; ++i) {
    the_frame.channels[i-1].status_1 &= 0xF0;
  }
  
  
  while( TRUE ) {		/* Wait for VD to go low */
    csr = read_csr();
    if ( !( csr & VD ) ) break;
    nanosleep(&nsleep,NULL);
  }

  while( TRUE ) {		/* Wait for VD to go high */
    csr = read_csr();
    if ( csr & VD ) break;
    nanosleep(&nsleep,NULL);
  }
  
  /* immediately after the VD went low, read the frame */
  last_counter = the_frame.counter;
  
   /* read the entire frame */
  rc = read_frame2D();

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

  /* was the frame counter advanced and was the number of bytes
     correct ? */

  if ( (the_frame.counter == last_counter + 1) && (rc-HLEN)%DLEN == 0) {
    ;
  } else {
    
    /* reset all the blobs to be non existent */
    for (i = 1; i<=MAX_BLOBS*N_CAMERAS; ++i) {
      the_frame.channels[i-1].status_1 &= 0xF0;
    }
    ++count_lost_frames;
  }
  
  /* copy the data into the global structures */

/*
      blobs[i][1].status = blobs[i][2].status = TRUE;
      blobs[i][1].x[_X_] =  the_frame.channels[i-1].x;
      blobs[i][1].x[_Y_] =  the_frame.channels[i-1].y;
      blobs[i][2].x[_X_] =  (the_frame.channels[i-1].status_1 & 0xf0) >> 4;
*/

  for (i=1, j=0; i<=MAX_BLOBS; ++i)
    {
      if (the_frame.channels[j].status_1 & 0x0F)
	{
	  blobs[i][1].status = TRUE;
	  blobs[i][1].x[_X_] =  the_frame.channels[j].x;
	  blobs[i][1].x[_Y_] =  the_frame.channels[j].y;
	}
      else
	{
	  blobs[i][1].status = FALSE;
	}
      j++;
      if (the_frame.channels[j].status_1 & 0x0F)
	{
	  blobs[i][2].status = TRUE;
	  blobs[i][2].x[_X_] =  the_frame.channels[j].x;
	  blobs[i][2].x[_Y_] =  the_frame.channels[j].y;
	}
      else
	{
	  blobs[i][2].status = FALSE;
	}
      j++;
    }

  return TRUE;

}  

/*!*****************************************************************************
 *******************************************************************************
\note  read_frame2D
\date  May 2000 by Tom  
\remarks 

        Reads a frame from the parallel board. Every 20 micro seconds
	a byte is read. One channel needs 5 bytes to be read. The
	program tries to read MAX_BLOBS channels, but uses a TIME_OUT
	to find out that no more data is sent. Adjust N_TIME_OUT for
	other CPUs!!!!!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none
     returns the number of bytes read

 ******************************************************************************/
static int
read_frame2D(void)

{
  int byte_counter;
  int iaux;
  int i,q;
  int time_out_counter;
  int lock;
  int channel;
/*  unsigned int int_to_float, *int_to_float_pointer;*/
  unsigned int temp=0;
  unsigned char ch=0;
  int s;
  short value;  /* 2 bytes */
  unsigned char *p;

/*  int_to_float_pointer = &int_to_float;*/
  
  /* lock out any other interupt on this processor to avoid losing bytes */
  lock = intLock();
  
  for( byte_counter = 1; byte_counter <= N_BYTES_PER_FRAME; byte_counter++ )   {
    
    time_out_counter = 0;
    while( TRUE ) {             /* Wait for STB to go high */
      csr = read_csr();
      if ( csr & STB ) break;
      if ( ++time_out_counter > N_TIME_OUT && byte_counter > 2) {
	intUnlock(lock);
	return byte_counter-1;
      }
     }
      
    time_out_counter = 0;
    while( TRUE ) {            /* Wait for STB to go low */
      csr = read_csr();
      if ( !( csr & STB ) )  break;
      if ( ++time_out_counter > N_TIME_OUT && byte_counter > 2) {
	intUnlock(lock);
	return byte_counter-1;
      }	
    }

    /* the following decoding is used:
       1.Byte: low byte of frame counter  
       2.Byte: high byte of frame counter
       note: order of low and high byte must be reversed
       
       in blocks of 5 bytes, each blob is decoded:
       
       1.Byte: status 
       2.-3  : x-position
       4.-5  : y-position
       */
    
    /* iaux reflects the current number in the byte count for blob */
    
    iaux = (byte_counter-HLEN-1) % DLEN;
    channel = (byte_counter-HLEN-1)/DLEN;

    if ( byte_counter <= HLEN )
      {
	switch( byte_counter )
	  {
	  case 1:
	    the_frame.counter = read_data();
	    break;
	  case 2:
	    the_frame.counter += (read_data() << 8);
	    break;
	  default:
	    fprintf( stderr, "Whoops, byte_counter: %d\n", byte_counter );
	    break;
	  }

	continue;
      }
    switch( iaux )
      {
      case 0:
	the_frame.channels[channel].status_1 = read_data();
	break;
/* process X */
      case 1:
	  p = (unsigned char *)& value;
	  p++; 
	  *p = read_uchar ();
	break;
      case 2:
	  p = (unsigned char *)& value;
	  *p = read_uchar ();
	  the_frame.channels[channel].x = value/10.0;
	break;
/* process Y */
      case 3:
	  p = (unsigned char *)& value;
	  p++; 
	  *p = read_uchar ();
	break;
      case 4:
	  p = (unsigned char *)& value;
	  *p = read_uchar ();
	  the_frame.channels[channel].y = value/10.0;
	break;

      default:
	fprintf( stderr, "Whoops, iaux: %d\n", iaux );
	break;
      }
  }
  
  intUnlock(lock);
  return byte_counter-1;
  
}

/*****************************************************************************/
