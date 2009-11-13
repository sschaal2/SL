/*!=============================================================================
  ==============================================================================

  \file    SL_invdyn_servo.h

  \author  Stefan Schaal
  \date    April 2000

  ==============================================================================

      the header file for the SL_invdyn_servo.c

  ============================================================================*/


#ifndef _SL_invdyn_servo_
#define _SL_invdyn_servo_

#ifndef SYS_CLOCK_RATE
#define SYS_CLOCK_RATE  60
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int    invdyn_lookup_data;
extern int    invdyn_learning_data;
extern int    invdyn_servo_errors;
extern int    invdyn_servo_initialized;
extern int    servo_enabled;

void dis(void);
void init_invdyn_servo(void);
int  run_invdyn_servo_learning(void);
int  run_invdyn_servo_lookup(void);
int  init_vxworks( void );
void status(void);

#ifdef __cplusplus
}
#endif

#endif  // _SL_invdyn_servo_
