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


#include <roadrunner.h>
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <netdb.h>

#define    SENDTO_ADDRESS    "192.168.1.255"
#define    SENDTO_PORT       4953   

int main(int argc, char **argv)
{
  char msg[100];
  int sockfd;
  struct sockaddr_in their_addr; 
  struct hostent *he;
  int numbytes;
  int broadcast = 1;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s enable run\n", argv[0]);

  if((he = gethostbyname(SENDTO_ADDRESS)) == NULL) { 
    herror("gethostbyname");
    exit(1);
  }
  
  if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }
  
  if(setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
		sizeof(broadcast)) == -1) {
    perror("setsockopt (SO_BROADCAST)");
    exit(1);
  }

  their_addr.sin_family = AF_INET;     
  their_addr.sin_port = htons(SENDTO_PORT);
  their_addr.sin_addr = *((struct in_addr *)he->h_addr);
  memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

  msg[0] = 0x01;
  msg[1] = 0x07;
  msg[2] = atoi(argv[1]);
  msg[3] = atoi(argv[2]);

  if((numbytes = sendto(sockfd, msg, 4, 0,
			(struct sockaddr *)&their_addr, 
			sizeof(struct sockaddr))) == -1) {
    perror("sendto");
    exit(1);
  }
  close(sockfd);
  return 0;
}
