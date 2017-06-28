/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <applanix_network.h>

static int applanix_socket_fd = -1;
static struct sockaddr_in applanix_remote_address;

void applanix_shutdown(void)
{
  if(applanix_socket_fd != -1) {
    printf("APPLANIX: Closing connection to Applanix POS LV...\n");
    fflush(stdout);
    shutdown(applanix_socket_fd, SHUT_RDWR);
    close(applanix_socket_fd);
    applanix_socket_fd = -1;
  }
}

void applanix_connect(char *remote_ip, unsigned short remote_port)
{
  const unsigned short arbitrary_local_port = 5000;
  struct sockaddr_in local_address;
  struct hostent *phostent;
  int ret;
  long opt;
        
  applanix_shutdown();
  
  printf("Opening connection to Applanix POS LV [%s:%hu]...", 
         remote_ip, remote_port);
  fflush(stdout);
  
  /* The arbitrary local port is a "hint".  If the port is unavailable, 
     another port will be selected automatically.  Note that the port
     is NOT privileged. */
        
  /* Initialize Local Port for Outbound Communication. */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(arbitrary_local_port);
  
  applanix_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if(applanix_socket_fd == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be created.\n");
    exit(1);
  }
  
  bind(applanix_socket_fd, (struct sockaddr *)&local_address, 
       sizeof(local_address));
  
  /* Socket is blocking. */
  opt = 0;
  ret = ioctl(applanix_socket_fd, FIONBIO, &opt);
  if(ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be set to blocking.\n");
    exit(1);
  }
  
  /* Socket will not receive broadcasts. */
  opt = 0;
  ret = setsockopt(applanix_socket_fd, SOL_SOCKET, SO_BROADCAST, 
                   &opt, sizeof(opt));
  if(ret == -1) {
    fprintf(stderr, 
            "\nAPPLANIX: Socket could not be set to block broadcasts.\n");
    exit(1);
  }
        
  /* Initialize Remote Address for Outbound Communication */
  phostent = gethostbyname(remote_ip);
  if(phostent == NULL) {
    fprintf(stderr, "\nAPPLANIX: Could not resolve hostname/IP address.\n");
    exit(1);
  }
  memset(&applanix_remote_address, 0, sizeof(applanix_remote_address));
  applanix_remote_address.sin_family = AF_INET;
  applanix_remote_address.sin_port = htons(remote_port);
  applanix_remote_address.sin_addr = *((struct in_addr *) phostent->h_addr);
  
  /* Connect to Applanix Hardware */        
  ret = connect(applanix_socket_fd, (struct sockaddr *) &applanix_remote_address, sizeof(applanix_remote_address));
  if(ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Could not connect to Applanix hardware.\n");
    exit(1);
  }
  
  printf("...SUCCESS!\nListening for messages...\n");
  fflush(stdout);
}

int applanix_read(char *buf, int buf_size)
{
  int bytes_received;
  
  if(applanix_socket_fd == -1) {
    fprintf(stderr,
            "\nAPPLANIX: applanix_read() called before applanix_connect().\n");
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
  
  bytes_received = recv(applanix_socket_fd, buf, buf_size, 0);
                        
  if(bytes_received == 0) {
    fprintf(stderr,
            "\nAPPLANIX: Applanix device performed orderly shutdown.\n");
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
        
  if(bytes_received == -1) {
    if(errno == EAGAIN)                /* No data right now.  Try again later. */
      return APPLANIX_READ_OK;
    
    fprintf(stderr, "\nAPPLANIX: Error while reading data.\n");
    fprintf(stderr, "APPLANIX: Error message is: %s\n", strerror(errno));
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
  return bytes_received;
}
