/*
 * SL_rt_mutex.h
 *
 *  Created on: Nov 10, 2010
 *      Author: kalakris
 */

#ifndef SL_RT_MUTEX_H_
#define SL_RT_MUTEX_H_

/**
 * This file defines a type "sl_rt_mutex", which is a wrapper for a real-time mutex.
 * It uses Xenomai mutexes on the Xenomai RTOS, and pthread mutexes otherwise.
 * We attempt to mimic the pthread_mutex_* api.
 *
 * Pthread condition functions are also wrapped as sl_rt_cond_*.
 *
 * TODOs:
 *  Error codes are not converted yet. 
 *  Calls with timeout values are not implemented yet.
 */


#ifdef __XENO__
#include <native/mutex.h>
#include <native/cond.h>
#else
#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __XENO__
  typedef RT_MUTEX sl_rt_mutex;
  typedef RT_COND sl_rt_cond;
#else
  typedef pthread_mutex_t sl_rt_mutex;
  typedef pthread_cond_t sl_rt_cond;
#endif

int sl_rt_mutex_init(sl_rt_mutex* mutex);
int sl_rt_mutex_destroy(sl_rt_mutex* mutex);
int sl_rt_mutex_lock(sl_rt_mutex* mutex);
int sl_rt_mutex_trylock(sl_rt_mutex* mutex);
int sl_rt_mutex_unlock(sl_rt_mutex* mutex);

int sl_rt_cond_init(sl_rt_cond* cond);
int sl_rt_cond_destroy(sl_rt_cond* cond);
int sl_rt_cond_signal(sl_rt_cond* cond);
int sl_rt_cond_wait(sl_rt_cond* cond, sl_rt_mutex* mutex);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////////
// inline function definitions follow:

inline int sl_rt_mutex_init(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_mutex_create(mutex, NULL);
#else
  return pthread_mutex_init(mutex, NULL);
#endif
}

inline int sl_rt_mutex_destroy(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_mutex_delete(mutex);
#else
  return pthread_mutex_destroy(mutex);
#endif
}

inline int sl_rt_mutex_lock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_mutex_acquire(mutex, TM_INFINITE);
#else
  return pthread_mutex_lock(mutex);
#endif
}

inline int sl_rt_mutex_trylock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_mutex_acquire(mutex, TM_NONBLOCK);
#else
  return pthread_mutex_trylock(mutex);
#endif
}

inline int sl_rt_mutex_unlock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_mutex_release(mutex);
#else
  return pthread_mutex_unlock(mutex);
#endif
}

inline int sl_rt_cond_init(sl_rt_cond* cond)
{
#ifdef __XENO__
  return rt_cond_create(cond, NULL);
#else
  return pthread_cond_init(cond, NULL);
#endif
}

inline int sl_rt_cond_destroy(sl_rt_cond* cond)
{
#ifdef __XENO__
  return rt_cond_delete(cond);
#else
  return pthread_cond_destroy(cond);
#endif
}

inline int sl_rt_cond_signal(sl_rt_cond* cond)
{
#ifdef __XENO__
  return rt_cond_signal(cond);
#else
  return pthread_cond_signal(cond);
#endif
}

inline int sl_rt_cond_wait(sl_rt_cond* cond, sl_rt_mutex* mutex)
{
#ifdef __XENO__
  return rt_cond_wait(cond, mutex, TM_INFINITE);
#else
  return pthread_cond_wait(cond, mutex);
#endif
}

#endif /* SL_RT_MUTEX_H_ */
