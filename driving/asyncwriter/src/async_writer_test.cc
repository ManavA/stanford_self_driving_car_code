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


#include <async_writer.h>

using dgc::AsyncWriter;

void test_chunk_size(char *filename, int chunk_size, int num_chunks)
{
  unsigned char *buffer;
  AsyncWriter writer;
  int i;

  buffer = new unsigned char[chunk_size];
  
  for (i = 0; i < chunk_size; i++)
    buffer[i] = (i % 100);
  
  writer.Open(filename);
  for (i = 0; i < num_chunks; i++) 
    writer.Write(chunk_size, buffer);
  writer.Close();

  delete [] buffer;
}

const int kLadybugPacketSize = 1000000;

void test_ladybug_rate(void)
{
  unsigned char *buffer;
  AsyncWriter writer;
  int i, count = 0;
  double t1, t2, write_time = 0;

  buffer = new unsigned char[kLadybugPacketSize];
  for (i = 0; i < kLadybugPacketSize; i++)
    buffer[i] = (i % 100);

  writer.Open("ladybug.dat");

  do {
    t1 = dgc_get_time();
    writer.Write(kLadybugPacketSize, buffer);
    t2 = dgc_get_time();
    write_time += (t2 - t1);
    usleep(33333);
    count++;
  } while(count < 10 * 30);

  writer.Close();
  delete [] buffer;

  fprintf(stderr, "Write time = %f\n", write_time);
}

void test_ladybug_sync(void)
{
  unsigned char *buffer;
  int i, count = 0;
  FILE *fp;
  double t1, t2, write_time = 0;
  
  buffer = new unsigned char[kLadybugPacketSize];
  for (i = 0; i < kLadybugPacketSize; i++)
    buffer[i] = (i % 100);

  fp = fopen("ladybug.dat", "w");
  if(fp == NULL)
    dgc_die("Error: could not open file.\n");

  do {
    t1 = dgc_get_time();
    fwrite(buffer, kLadybugPacketSize, 1, fp);
    t2 = dgc_get_time();
    write_time += (t2 - t1);
    usleep(33333);
    count++;
  } while(count < 10 * 30);

  fclose(fp);
  delete [] buffer;

  fprintf(stderr, "Write time = %f\n", write_time);
}

int main(void)
{
  fprintf(stderr, "Async:\n");
  test_ladybug_rate();
  fprintf(stderr, "Sync:\n");
  test_ladybug_sync();
  return 0;

  test_chunk_size("test.dat", 10000, 100000);
  test_chunk_size("test2.dat", 65536, 15258);
  test_chunk_size("test3.dat", 100000, 10000);
  return 0;
}
