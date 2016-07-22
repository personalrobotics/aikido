/*****************************************************************************
 *
 * Copyright (c) 2013, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/
#include "SignalHandler.h"
#include <assert.h>
#include <map>
#include <list>
#include <signal.h>
#include <stdlib.h>
using namespace std;

typedef void (*SIGNAL_PROC) (int);
typedef list<SignalHandler*> HandlerList;

map<int,SIGNAL_PROC> initialHandlers;
map<int,list<SignalHandler*> > sigHandlers;

void EraseHandler(HandlerList& l,SignalHandler* h)
{
  list<SignalHandler*>::iterator i,p;
  for(i=l.begin();i!=l.end();i++) {
    if(*i==h) {
      p=i; p--;
      l.erase(i);
      i=p;
    }
  }
}

bool HasHandler(int signum)
{
  return sigHandlers.count(signum) != 0 && !sigHandlers[signum].empty();
}



void theSignalHandlerProc(int signum)
{
  assert(sigHandlers.count(signum) != 0);
  assert(!sigHandlers[signum].empty());
  sigHandlers[signum].back()->OnRaise(signum);
}



SignalHandler::~SignalHandler()
{
  for(map<int,list<SignalHandler*> >::iterator i=sigHandlers.begin();i!=sigHandlers.end();i++) {
    EraseHandler(i->second,this);
    if(i->second.empty()) 
      signal(i->first,initialHandlers[i->first]);
  }
}

void SignalHandler::SetCurrent(int signum)
{
  SIGNAL_PROC prevSignalProc = signal(signum,theSignalHandlerProc);
  if(prevSignalProc == SIG_IGN) {
    signal(signum,SIG_IGN);
    return;
  }
  if(!HasHandler(signum)) {
    initialHandlers[signum] = prevSignalProc;
  }
  sigHandlers[signum].push_back(this);
}

bool SignalHandler::IsCurrent(int signum) const
{
  if(!HasHandler(signum)) return false;
  return (this == sigHandlers[signum].back());
}

void SignalHandler::UnsetCurrent(int signum)
{
  assert(HasHandler(signum));
  assert(sigHandlers[signum].back() == this);
  sigHandlers[signum].pop_back();
  if(sigHandlers[signum].empty()) {
    signal(signum,initialHandlers[signum]);
    initialHandlers[signum]=NULL;
  }
}

SignalHandler* SignalHandler::GetCurrent(int signum)
{
  if(!HasHandler(signum)) return NULL;
  return sigHandlers[signum].back();
}
