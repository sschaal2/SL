/*!=============================================================================
  ==============================================================================

  \file    SL_socket_unix.h

  \author  Jeannette Bohg
  \date    Oct. 2014

  ==============================================================================

  supports SL_socket_unix.c

  ============================================================================*/


#ifndef _SL_socket_unix_
#define _SL_socket_unix_

#ifdef __cplusplus
extern "C" {
#endif

int  open_socket();
void close_socket(int fd);
int  read_socket(int fd,int n_bytes, char *buffer);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_socket_unix_ */
