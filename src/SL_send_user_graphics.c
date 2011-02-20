/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo.c

  \author  Stefan Schaal
  \date    1997

  ==============================================================================
  \remarks

  manages the control loop to run different tasks 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_send_user_graphics.h"
#include "SL_tasks.h"
#include "SL_shared_memory.h"
#include "SL_common.h"

#define TIME_OUT_NS  1000000000

/*!*****************************************************************************
 *******************************************************************************
\note  sendUserGraphics
\date  Nov. 2007
   
\remarks 

      sends out information for user graphics to shared memory

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name   : name of graphics
 \param[in]     buf    : byte buffer with information
 \param[in]     n_bytes: number of bytest in buffer

 ******************************************************************************/
int
sendUserGraphics(char *name, void *buf, int n_bytes)
{
  int i,j;
  
  // send the user graphics data
  if (semTake(sm_user_graphics_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take user_graphics semaphore\n");
    return FALSE;
  }

  // first, check whether there is an older graphics object with the same
  // name -- it will be overwritten, as only the latest graphics objects
  // matters for visualization
  for (i=1; i<=sm_user_graphics->n_entries; ++i) {
    if (strcmp(name,sm_user_graphics->name[i]) == 0) {

      // just overwrite the data
      memcpy(sm_user_graphics->buf+sm_user_graphics->moff[i],buf,n_bytes);

      // give semaphores
      semGive(sm_user_graphics_sem);
      semGive(sm_user_graphics_ready_sem);

      return TRUE;

    }
  }

  // make sure the pointer offset buffer is correct
  if (sm_user_graphics->n_entries == 0) {
    sm_user_graphics->moff[1] = 0;
    sm_user_graphics->n_bytes_used = 0;
  }

  // check whether there is space for the graphics object
  if (sm_user_graphics->n_entries >= MAX_N_MESSAGES) {
    printf("User graphics buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  if (MAX_BYTES_USER_GRAPHICS-sm_user_graphics->n_bytes_used < n_bytes) {
    printf("User Graphics memory buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  // update the logistics
  ++sm_user_graphics->n_entries;
  sm_user_graphics->n_bytes_used += n_bytes;
  
  // specify the name and info of this entry
  strcpy(sm_user_graphics->name[sm_user_graphics->n_entries],name);
  memcpy(sm_user_graphics->buf+sm_user_graphics->moff[sm_user_graphics->n_entries],buf,n_bytes);
  
  // prepare pointer buffer for next entry
  if (sm_user_graphics->n_entries < MAX_N_MESSAGES)
    sm_user_graphics->moff[sm_user_graphics->n_entries+1]=
      sm_user_graphics->moff[sm_user_graphics->n_entries]+n_bytes;

  // give semaphores
  semGive(sm_user_graphics_sem);
  semGive(sm_user_graphics_ready_sem);
  
  return TRUE;
}
