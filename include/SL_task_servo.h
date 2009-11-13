/*!=============================================================================
  ==============================================================================

  \file    SL_task_servo.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  the header file for the task_servo.c

  ============================================================================*/


#ifndef _SL_task_servo_
#define _SL_task_servo_

#ifndef SYS_CLOCK_RATE
#define SYS_CLOCK_RATE  60
#endif

#define MOTORSERVO  1
#define INVDYNSERVO 2
#define CARTSERVO   3

#ifdef __cplusplus
extern "C" {
#endif

  extern int    task_servo_errors;
  extern long   task_servo_calls;
  extern int    task_servo_initialized;
  extern double task_servo_time;
  extern double servo_time;
  extern int    servo_enabled;
  extern int    task_servo_rate;
  extern int    frame_counter;
  extern int    exit_on_stop;
  extern char   initial_user_command[];
  
  void dts(void);
  int  go(int jID);
  void go0(void);
  int  go0_wait(void);
  int  go_target_wait(SL_DJstate *target);
  int  go_target_wait_ID(SL_DJstate *target);
  void freeze(void);
  void f(void);
  void init_task_servo(void);
  int  run_task_servo(void);
  int  init_vxworks( void );
  int  send_raw_blobs(void);
  int  send_raw_blobs2D(void);
  int  go_cart_target_wait(SL_Cstate *ctar,int *stat, double mt);
  void goVisTarget(void);
  void status(void);
  int  stop(char *);
  int  setServoMode(int type);
  int  init_generic_tasks(void);
  int  run_generic_tasks(void);
  int  sendUserGraphics(char *name, void *buf, int n_bytes);
  void reset(void);
  void followBase(void);
  void changeHideObject(char *name, int hide); 
  void changePDGains(double *pGain, double *dGain);
  void sendUextSim(void);
  void changeRealTime(int flag);
  void hideWindowByName(char *name, int hide);
  void freezeBase(int flag);
  void setGravity(double grav);
  void setG(void);


  
#ifdef __cplusplus
}
#endif

#endif  // _SL_task_servo_
