/*!=============================================================================
  ==============================================================================

  \file    SL_send_user_graphics.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  the header file for the task_servo.c

  ============================================================================*/


#ifndef _SL_send_user_graphics_
#define _SL_send_user_graphics_

#ifdef __cplusplus
extern "C" {
#endif

  int  sendUserGraphics(char *name, void *buf, int n_bytes);
  
#ifdef __cplusplus
}
#endif

#endif  // _SL_user_graphics_
