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


#include <sys/wait.h>

char *libpath = NULL;

int spawn_process(char *command, char *env)
{
  char lparam[200];
  char *envlist[3];
  int spawned_pid;
  
  if((spawned_pid = fork()) == 0) {
    envlist[0] = env;
    if(libpath != NULL) {
      sprintf(lparam, "LD_LIBRARY_PATH=%s", libpath);
      envlist[1] = lparam;
      envlist[2] = NULL;
    }
    else
      envlist[1] = NULL;
    execle("/bin/tcsh", "/bin/tcsh", "-c", command, NULL, envlist);
  }
  return spawned_pid;
}

int main(int argc, char **argv)
{
  char command[1000], env[1000];
  int i, count;

  libpath = getenv("LD_LIBRARY_PATH");

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s central-count bin-path process-file\n", argv[0]);

  count = atoi(argv[1]);
  for(i = 0; i < count; i++) {
    sprintf(command, "%s/pidcontrol %s", argv[2], argv[3]);
    sprintf(env, "CENTRALHOST=localhost:%d", 1381 + i);
    spawn_process(command, env);
  }

  while(1)
    sleep(1);
  return 0;
}
