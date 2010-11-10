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
 */


#ifdef __XENO__
#include <native/mutex.h>
#else
#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __XENO__
  typedef RT_MUTEX sl_rt_mutex;
#else
  typedef pthread_mutex_t sl_rt_mutex;
#endif

int sl_rt_mutex_init(sl_rt_mutex* mutex);
int sl_rt_mutex_destroy(sl_rt_mutex* mutex);
int sl_rt_mutex_lock(sl_rt_mutex* mutex);
int sl_rt_mutex_trylock(sl_rt_mutex* mutex);
int sl_rt_mutex_unlock(sl_rt_mutex* mutex);

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

#endif /* SL_RT_MUTEX_H_ */
