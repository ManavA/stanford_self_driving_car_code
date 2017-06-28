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


#ifndef DGC_SERIAL_H
#define DGC_SERIAL_H

namespace dgc {

/* connect to a serial port at a given speed */

int dgc_serial_connect(int *fd, const char *dev_name, int baudrate);

/* set the buadrate and the parity of the serial port - doesn't currently
   support flow control.  Uses 8 data bits and 1 stop bit */

int dgc_serial_setparams(int fd, int baudrate, char parity);

/* returns the number of bytes available to be read on a serial channel */
        
long int dgc_serial_bytes_available(int fd);

/* clear the input buffer of a serial channel */

int dgc_serial_clear_input_buffer(int fd);

/* writes n characters to a serial channel. Timeout applies to each
   individual write, not the total write time */

int dgc_serial_writen(int fd, unsigned char *buffer, int n, double timeout);

/* reads n characters from a serial channel unless a timeout or 
   error occurs */

int dgc_serial_readn(int fd, unsigned char *buffer, int n, double timeout);

/* closes the serial port */

void dgc_serial_close(int fd);

} //namespace dgc
#endif
