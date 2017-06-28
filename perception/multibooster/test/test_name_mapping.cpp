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


#include <multibooster/name_mapping.h>
#include <gtest/gtest.h>

using namespace std;

TEST(NameMapping, mapping) {
  vector<string> names1;
  names1.push_back("aaa");
  names1.push_back("bbb");
  names1.push_back("ccc");
  vector<string> names2;
  names2.push_back("ccc");
  names2.push_back("bbb");
  names2.push_back("aaa");
  
  NameMapping m1(names1);
  NameMapping m2(names2);

  NameTranslator t12(m1, m2);
  for(size_t i=0; i<names1.size(); ++i) {
    EXPECT_TRUE(m1.toName(i) == m2.toName(t12.toMap2(i)));
  }
}

TEST(NameMapping, loadAndSave) {
  vector<string> names1;
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  NameMapping m1(names1);
  string m1str = m1.serialize();
  //  cout << m1str << endl;

  istringstream iss(m1str);
  NameMapping m2(iss);
  //  cout << m2.serialize() << endl;
  EXPECT_TRUE(m1.compare(m2));
}

TEST(NameMapping, compare) {
  vector<string> names1;
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  vector<string> names2;
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("xxx");

  NameMapping m1(names1);
  NameMapping m2(names2);
  
  EXPECT_TRUE(m1.compare(m1));
  EXPECT_FALSE(m1.compare(m2));
}


TEST(NameMapping, nospaces) {
  vector<string> names1;
  names1.push_back("c c c");
  names1.push_back("bbb");
  names1.push_back("aaa");

  bool failed = false;
  try  {
    NameMapping m1(names1);
  }
  catch (int e) {
    if(e == 5)
      failed = true;
  }
  
  EXPECT_TRUE(failed);
}

TEST(NameMapping, augmentation) {
  vector<string> names1;
  names1.push_back("xxx");
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  vector<string> names2;
  names2.push_back("ccc");


  NameMapping m1(names1);
  NameMapping m2(names2);

  m2.augment(m1);
  //cout << m1.serialize() << endl;
  //cout << m2.serialize() << endl;
  
  NameTranslator t12(m1, m2);
  for(size_t i=0; i<4; ++i) { 
    EXPECT_TRUE(m1.toName(i) == m2.toName(t12.toMap2(i)));
  }
}
  

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

  
