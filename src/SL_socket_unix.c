/*!=============================================================================
  ==============================================================================

  \file    SL_socket.c

  \author  Jeannette Bohg
  \date    Oct 2014

  ==============================================================================
  \remarks

  generic routines for managing socket communiction

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_man.h"
#include "termios.h"
#include "fcntl.h"
#include "sys/ioctl.h"
#include "unistd.h"

// socket includes
#include <sys/socket.h>
#include <netdb.h>

#define MYPORT "4950"	// the port users will be connecting to

/* local variables */

/* global variables */
struct sockaddr_storage their_addr;
socklen_t addr_len;

/* local functions */

/*!*****************************************************************************
 *******************************************************************************
\note  open_socket
\date  Oct 2014
   
\remarks 

        opens a socket to a specific port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 return socket_fd, i.e., the file descriptor, or FALSE

 ******************************************************************************/
int
open_socket()
{
  // socket id
  int sockfd;
  int rv;
  struct addrinfo hints, *servinfo, *p;

  addr_len = sizeof their_addr;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE; // use my IP
  
  if ((rv = getaddrinfo(NULL, MYPORT, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return FALSE;
  }

  // loop through all the results and bind to the first we can
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, 
			 p->ai_socktype,
			 p->ai_protocol)) == -1) {
      perror("listener: socket");
      continue;
    }

    if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("listener: bind");
      continue;
    }

    break;
  }

  if (p == NULL) {
    fprintf(stderr, "listener: failed to bind socket\n");
    return FALSE;
  }

  freeaddrinfo(servinfo);

  // making the socket read a blocking call with a timeout of 40ms
  struct timeval tv;
  tv.tv_sec = 1;  // blocking read for one second
  tv.tv_usec = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));

  return sockfd;
}
/*!*****************************************************************************
 *******************************************************************************
\note  close_socket
\date  Oct 2014
   
\remarks 

        closes the socket connection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor

 ******************************************************************************/
void
close_socket(int fd) 
{
  close(fd);
}


/*!*****************************************************************************
 *******************************************************************************
\note  read_socket
\date  Oct 2014
   
\remarks 

        reads into a buffer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor
 \param[in]     n_bytes: number of bytes to read
 \param[out]    buffer: buffer for read

     returns the number of bytes actually read
     this is a non-blocking call

 ******************************************************************************/
int
read_socket(int fd,int n_bytes, char *buffer) 
{

  rt_task_set_mode(T_PRIMARY,0,NULL);
  return recvfrom(fd, buffer, n_bytes , MSG_WAITALL, //MSG_DONTWAIT,
		  (struct sockaddr *)&their_addr, &addr_len);
}

