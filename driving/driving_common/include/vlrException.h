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


#ifndef DRIVING_COMMON_EXCEPTION_H_
#define DRIVING_COMMON_EXCEPTION_H_

#include <string>

namespace vlr {

struct InvalidArgumentEx {};
struct ZeroPointerArgumentEx {};

struct IOEx {};
struct FileIOEx {};
struct SocketIOEx {};

class BaseException {
 public:
  BaseException(const std::string& error = std::string()) :
                   error_message_(error) {
  }

  virtual ~BaseException() {}

  const std::string& errorMessage() const {return error_message_;}
  const std::string& what() const  {return error_message_;}

protected:
  std::string error_message_;
};

struct EmptyEx {};

template<class t1 = EmptyEx, class t2 = EmptyEx, class t3 = EmptyEx>
struct Ex : public Ex<t2, t3, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<t2, t3>(error) {}
};

template<>
struct Ex<EmptyEx, EmptyEx, EmptyEx> : public BaseException {
  Ex(const std::string& error = std::string()) : BaseException(error) {}
};

template<class t1>
struct Ex<t1, EmptyEx, EmptyEx> : public Ex<EmptyEx, EmptyEx, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<>(error) {}
};

template<class t1, class t2 >
struct Ex<t1, t2, EmptyEx> : public Ex<t2, EmptyEx, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<t2>(error) {}
};


#define VLRException(str) vlr::Ex<>(__PRETTY_FUNCTION__ + std::string(": ") + str)
#define VLRExceptionLevel1(str, t1) vlr::Ex<t1>(__PRETTY_FUNCTION__ + std::string(": ") + str)
#define VLRExceptionLevel2(str, t1, t2) vlr::Ex<t1, t2>(__PRETTY_FUNCTION__ + std::string(": ") + str)
#define VLRExceptionLevel3(str, t1, t2, t3) vlr::Ex<t1, t2, t3>(__PRETTY_FUNCTION__ + std::string(": ") + str)

} // namespace vlr

#endif

