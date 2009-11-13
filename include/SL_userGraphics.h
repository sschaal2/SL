/*!=============================================================================
  ==============================================================================

  \file    SL_userGraphics.h

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  SL_userGraphics.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_userGraphics_
#define _SL_userGraphics_

/* external variables */

/* share functions */

#ifdef __cplusplus
extern "C" {
#endif

  void listUserGraphics(void);
  void addToUserGraphics(char *abr, char *string, void (*fptr)(void *),int n_bytes);
  void runUserGraphics(void);
  int  checkForUserGraphics(void);

  extern char user_display_function_name[];
  extern int  user_graphics_update;

#ifdef __cplusplus
}
#endif

#endif  /* _SL_userGraphics_ */
