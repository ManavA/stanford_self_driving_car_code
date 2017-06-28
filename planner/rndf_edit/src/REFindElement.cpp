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


#include <REFindElement.h>

FindDialog::FindDialog(QWidget* parent) : QDialog(parent), close_button_pressed_(false) {
  find_button_ = new QPushButton(tr("&Find"));
  close_button_ = new QPushButton(tr("&Close"));

  QLabel* label = new QLabel(tr("Find:"));
  search_box_ = new QLineEdit;
  label->setBuddy(search_box_);

  QHBoxLayout* top_layout = new QHBoxLayout;
  top_layout->addWidget(label);
  top_layout->addWidget(search_box_);

  QHBoxLayout* bottom_layout = new QHBoxLayout;
  bottom_layout->addStretch();
  bottom_layout->addWidget(find_button_);
  bottom_layout->addWidget(close_button_);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout(top_layout);
  main_layout->addLayout(bottom_layout);
  setLayout(main_layout);
  setWindowTitle(tr("Find Element"));

  connect(find_button_, SIGNAL(clicked()), this, SLOT(on_action_findButton_clicked()));
  connect(close_button_, SIGNAL(clicked()), this, SLOT(on_action_closeButton_clicked()));
  connect(search_box_, SIGNAL(editingFinished()), this, SLOT(on_action_searchBox_editingFinished()));
}

FindDialog::~FindDialog() {
}

void FindDialog::on_action_closeButton_clicked() {
  hide();
  close_button_pressed_ = true;
}

void FindDialog::on_action_findButton_clicked() {
  hide();
  close_button_pressed_ = false;
}

void FindDialog::on_action_searchBox_editingFinished() {
  hide();
  close_button_pressed_ = false;
}
