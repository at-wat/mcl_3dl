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
#include <stdint.h>

#include <dlfcn.h>

#include <ros/ros.h>

#include <mcl_3dl/mcl_3dl.h>

void showStack(int i, void* addr)
{
  Dl_info info;
  dladdr(addr, &info);
  fprintf(stderr, "#%u %p %s %p %s %p\n",
          i, addr, info.dli_fname, info.dli_fbase, info.dli_sname, info.dli_saddr);
}

void stacktrace(int signum)
{
  signal(signum, SIG_DFL);

  fprintf(stderr, "mcl_3dl is exiting by signal %d\n", signum);

  while (true)
  {
    void* addr0 = __builtin_frame_address(0);
    if (addr0 == nullptr)
      break;
    showStack(0, addr0);
    void* addr1 = __builtin_frame_address(1);
    if (addr1 == nullptr)
      break;
    showStack(1, addr1);
    void* addr2 = __builtin_frame_address(2);
    if (addr2 == nullptr)
      break;
    showStack(2, addr2);
    void* addr3 = __builtin_frame_address(3);
    if (addr3 == nullptr)
      break;
    showStack(3, addr3);

    break;
  }
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
