/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef HAVE_EXECINFO
#include <execinfo.h>
#endif  // HAVE_EXECINFO

#include <iostream>

#include <ros/ros.h>

#include <mcl_3dl/mcl_3dl.h>

void stacktrace(int signum)
{
  signal(signum, SIG_DFL);
#ifdef HAVE_EXECINFO
  void* buffer[100];
  int nptrs = backtrace(buffer, 100);
  fprintf(stderr, "stacktrace (%d):\n", nptrs);
  fflush(stderr);
  backtrace_symbols_fd(buffer, nptrs, STDERR_FILENO);
  fflush(stderr);
#endif  // HAVE_EXECINFO
  raise(signum);
}

int main(int argc, char* argv[])
{
  signal(SIGSEGV, &stacktrace);
  signal(SIGABRT, &stacktrace);

  ros::init(argc, argv, "mcl_3dl");

  mcl_3dl::MCL3dlNode mcl;
  if (!mcl.configure())
  {
    return 1;
  }
  ros::spin();

  return 0;
}
