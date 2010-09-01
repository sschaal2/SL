/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_oscilloscope.c

  \author  Stefan Schaal
  \date    Nov 2009

  ==============================================================================
  \remarks

  Manages visualization of data traces on an oscilloscope, which can be a
  real physical oscilloscope or a graphics simulation of an oscilloscope.
  Additionally, for graphics oscilloscope, other data traces can be visualized.
  
  ============================================================================*/

// system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_oscilloscope.h"
#include "SL_shared_memory.h"
#include "SL_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"

// defines
#define  MAX_CHARS 20

/*! a structure to store information about variables */
typedef struct Cinfo {
  char *ptr;
  char  name[MAX_CHARS];
  int   type;
  int   pID;
  char *nptr;
} OscInfo;

static OscInfo *osc_info_ptr = NULL;

// global variabes

// local variabes
static int   (*d2a_function)(int,double) = NULL;

// data collection for oscilloscope
static SL_oscEntry   oscbuf[OSC_BUFFER_SIZE+1];        //<! the ring buffer of all entries
static int           current_osc_index = 1;            //<! index of latest entry
static int           n_osc_entries = 0;                //<! number of entries in buffer
static int           osc_enabled = FALSE;              //<! is the oscilloscope active?
static char          osc_vars_name[100];               //<! file name for osc variables

// global functions

// local functions
static void oscMenu(void);
static int  readOscVarsScript( char *fn, int flag );
static void updateOscTimeWindow(double w);


/*!*****************************************************************************
*******************************************************************************
\note  setOsc
\date  Nov. 2009
 
\remarks 
 
set the oscilloscope on a particular channel to a particiular value,
expressed as a percentage value of the max range of the D/A converter.
Using percentage values frees the user from knowing the resolution of the
D/A converter.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   channel : which channel to use
\param[in]   pval    : [0 to 100] percentage value of max range to be displayed.
 
******************************************************************************/
#define TIME_OUT_NS 100000
int
setOsc(int channel, double pval)
{
  int    rc;
  double ts;
  char   string[40];
  char   fname[100];
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    if (read_parameter_pool_int(config_files[PARAMETERPOOL],"osc_enabled", &rc))
      osc_enabled = macro_sign(abs(rc));

    if (osc_enabled) {
      addToMan("oscMenu","interactively change oscilloscope settings",oscMenu);
      sprintf(osc_vars_name,"%s_default.osc",servo_name);
      sprintf(string,"osc_vars_%s_file",servo_name);
      if (read_parameter_pool_string(config_files[PARAMETERPOOL],string,fname))
	strcpy(osc_vars_name,fname);
      readOscVarsScript(osc_vars_name,FALSE);
    }

  }

  // if the user provide a special oscilloscope function
  if (d2a_function != NULL) {
    if (semTake(sm_oscilloscope_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
      return FALSE;
    } else {
      rc = (*d2a_function)(channel,pval);
      semGive(sm_oscilloscope_sem);
      return rc;
    }
  }

  if (!osc_enabled)
    return TRUE;

  // the graphics oscilloscope is the default -- just add the data
  // in the appropriate structure in memory

#ifdef __XENO__
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC,&t);
  ts = (double) t.tv_sec + ((double)t.tv_nsec)/1.e9;
#else
  struct timeval t;
  gettimeofday(&t,NULL);
  ts = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
#endif

  sprintf(string,"D2A_%s",servo_name);
  addEntryOscBuffer(string, pval, ts, 0);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  setD2AFunction
\date  Nov. 2009
 
\remarks 
 
Provides a function pointer which is used to display D2A debugging signals.
Usually, this allows re-routing the D2A debugging signals to a D/A signal
converter, which is part of some DAQ board.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   fptr : function pointer
 
******************************************************************************/
void
setD2AFunction(int (*fptr)(int,double))
{
  int rc;

  d2a_function = fptr;

}

/*!*****************************************************************************
*******************************************************************************
\note  addEntryOscBuffer
\date  Aug 2010
 
\remarks 
 
Adds an entry to the oscilloscope ring buffer. As this is a ringer buffer,
the oldest value is overwritten if the buffer size is reached
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]   name : name of entry
\param[in]      v : value of entry
\param[in]     ts : time stamp of entry (in seconds)
\param[in]    pID : plot ID, i.e., in which plot his is visualized
 
******************************************************************************/
void
addEntryOscBuffer(char *name, double v, double ts, int pID)
{

  if (!osc_enabled) 
    return;

  if (++current_osc_index > OSC_BUFFER_SIZE)
    current_osc_index = 1;

  strncpy(oscbuf[current_osc_index].name,name,sizeof(oscbuf[current_osc_index].name)-1);
  oscbuf[current_osc_index].v       = (float) v;
  oscbuf[current_osc_index].ts      = ts;
  oscbuf[current_osc_index].plotID  = pID;

  if (++n_osc_entries > OSC_BUFFER_SIZE) {
    n_osc_entries = OSC_BUFFER_SIZE;
  }

}

/*!****************************************************************************
******************************************************************************
\note  sendOscilloscopeData
\date  Aug. 2010

\remarks

copies all local oscilloscope data to shared memory structure

*****************************************************************************
Function Parameters: [in]=input,[out]=output

none

*****************************************************************************/
#define TIME_OUT_SHORT_NS   10000  //!< time out in nano seconds
void 
sendOscilloscopeData(void)
{
  
  int      i,j,r;
  int      count;
  double   temp = 0;
  OscInfo *optr;
  extern double servo_time;
  
  if (!osc_enabled) 
    return;

  // add data to the buffer
  optr = osc_info_ptr;
  while (optr != NULL) {

    switch (optr->type) {

      case DOUBLE:
	temp = (float) *((double *) optr->ptr);
	break;

      case FLOAT:
	temp = (float) *((float *) optr->ptr);
	break;

      case INT:
	temp = (float) *((int *) optr->ptr);
	break;

      case SHORT:
	temp = (float) *((short *) optr->ptr);
	break;

      case LONG:
	temp = (float) *((long *) optr->ptr);
	break;

      case CHAR:
	temp = (float) *((char *) optr->ptr);
	break;

    }

    addEntryOscBuffer(optr->name, temp, servo_time, optr->pID);

    optr = (OscInfo *) optr->nptr;

  }
  

  // try to take semaphore with very short time-out -- we don't care if we 
  // cannot copy the data to shared memory every time, as this is not a time 
  // critical operation
  if (semTake(sm_oscilloscope_sem,ns2ticks(TIME_OUT_SHORT_NS)) == ERROR)
    return;

  // copy first-in-first-out data into shared memory as long as there is enough
  // space
  if (sm_oscilloscope->n_entries+n_osc_entries > OSC_SM_BUFFER_SIZE) { // cannot copy all
    count = OSC_SM_BUFFER_SIZE - sm_oscilloscope->n_entries;
  } else 
    count = n_osc_entries;

  j = sm_oscilloscope->n_entries;
  for (i=1; i<=count; ++i) {
    r = current_osc_index - n_osc_entries + i;
    if ( r > OSC_BUFFER_SIZE)
      r -= OSC_BUFFER_SIZE;
    if ( r < 1)
      r += OSC_BUFFER_SIZE;
    sm_oscilloscope->entries[j+i] = oscbuf[r];
  }

  n_osc_entries -= count;
  sm_oscilloscope->n_entries += count;
  
  // give back semaphore
  semGive(sm_oscilloscope_sem);
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  readOscVarsScript
\date  Aug. 2010
   
\remarks 

reads an oscilloscope data visulization script

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fn : the name of the file to be opened
 \param[in]   flag : verbose TRUE/FALSE

 ******************************************************************************/
static int
readOscVarsScript( char *fn, int flag )
     
{

  FILE    *infile;
  int      i,rc;
  char     string[100];
  char     fname[100];
  int      pID;
  void    *vptr;
  int      type;
  int      count=0;
  OscInfo *optr;

  if (!osc_enabled)
    return TRUE;

  sprintf(fname,"%s%s",PREFS,fn);
  infile = fopen(fname,"r");

  if (infile == NULL) {
    printf("Error when trying to open file >%s< !\n",fname);
    return FALSE;
  }

  // erase old variables
  optr = osc_info_ptr;
  while (optr != NULL) {
    osc_info_ptr = (OscInfo *)optr;
    optr         = (OscInfo *)optr->nptr;
    free(osc_info_ptr);
  }
  osc_info_ptr = NULL;
    
  
  // now read the file
  while ((rc=fscanf(infile,"%s %d",string,&pID)) != EOF) {

    // figure out whether the string exists
    if (getDataCollectPtr(string, &vptr, &type )) {
      ++count;
      if (count == 1) {
	osc_info_ptr = (OscInfo *) my_calloc(1,sizeof(OscInfo),MY_STOP);
	optr = (OscInfo *) osc_info_ptr;
      } else {
	optr->nptr = (char *) my_calloc(1,sizeof(OscInfo),MY_STOP);
	optr = (OscInfo *) optr->nptr;
      }

      optr->ptr  = (char *)vptr;
      optr->type = type;
      optr->nptr = NULL;
      optr->pID  = pID;
      sprintf(optr->name,"%c.%s",servo_name[0],string);

    }
  }

  fclose(infile);

  if (flag)
    printf("\nRead %d valid names from file >%s< .\n\n",count,fname);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  oscMenu
\date  Aug. 2010
   
\remarks 

a menu to configure the variables for the oscilloscope

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
oscMenu(void)

{
  int aux = 2;
  static double time_window=5.0;
  double temp;
  static char  fname[100] = "default.script";

  if (!osc_enabled)
    return;

  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"osc_time_window_vars", &temp))
    time_window = temp;

  AGAIN:
  
  printf("\n\n\nCHANGE OSCILLOSCOPE SETTINGS:\n\n");
  printf("        Time Window of Plots            ---> %d\n",1);
  printf("        Read Oscilloscope Variables     ---> %d\n",2);
  printf("        Quit                            ---> q\n");
  printf("\n");
  if (!get_int("        ----> Input",aux,&aux)) return;
  
  if (aux > 2 || aux < 1) {
    
    goto AGAIN;
    
  } 
  
  
  if (aux == 1) {
    
    if (get_double("Time Window of Plots [s]?\0",time_window,&temp))
      if (temp > 0) {
	updateOscTimeWindow(temp);
	time_window = temp;
      }
    
  } else if (aux == 2) {
    
  ONCE_MORE:

    // get the file name

    if (!get_string("Name of the Oscilloscope Variables File\0",osc_vars_name,fname)) 
      goto AGAIN;

    // try to read this file

    if (!readOscVarsScript(fname,TRUE)) 
      goto ONCE_MORE;

    strcpy(osc_vars_name,fname);
    
  }

  goto AGAIN;

}

/*!*****************************************************************************
 *******************************************************************************
\note  updateOscTimeWindow
\date  Aug 2010
   
\remarks 

sends messages to openGL servo to update the time window of oscilloscope


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     w : time window value

 ******************************************************************************/
static void 
updateOscTimeWindow(double w) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = w;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));
    
  sendMessageOpenGLServo("updateOscTimeWindow",(void *)cbuf,sizeof(float));

}




