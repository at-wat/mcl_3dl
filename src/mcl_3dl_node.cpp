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
#include <inttypes.h>

#define UNW_LOCAL_ONLY
#include <libunwind.h>
#include <cxxabi.h>

#include <ros/ros.h>

#include <mcl_3dl/mcl_3dl.h>

void trace(unw_context_t* context)
{
  unw_cursor_t cursor;
  unw_init_local2(&cursor, context, 1);

  int n = 0;
  while (unw_step(&cursor))
  {
    unw_word_t ip, sp, off;

    unw_get_reg(&cursor, UNW_REG_IP, &ip);
    unw_get_reg(&cursor, UNW_REG_SP, &sp);

    char symbol[256] = { "<unknown>" };
    char* name = symbol;

    if (!unw_get_proc_name(&cursor, symbol, sizeof(symbol), &off))
    {
      int status;
      if ((name = abi::__cxa_demangle(symbol, NULL, NULL, &status)) == 0)
        name = symbol;
    }

    fprintf(
        stderr, "#%-2d 0x%016" PRIxPTR " sp=0x%016" PRIxPTR " %s + 0x%" PRIxPTR "\n",
        ++n,
        static_cast<uintptr_t>(ip),
        static_cast<uintptr_t>(sp),
        name,
        static_cast<uintptr_t>(off));

    if (name != symbol)
      free(name);
  }
}

void signalHandler(int signum, siginfo_t* siginfo, void* crashContextPtr)
{
  signal(signum, SIG_DFL);

  fprintf(stderr, "mcl_3dl is exiting by signal %d\n", signum);
  unw_context_t context;
  unw_getcontext(&context);
  trace((unw_context_t*)&context.uc_mcontext);

  raise(signum);
}

void setSignalHandler(int signum)
{
  struct sigaction action;
  action.sa_sigaction = signalHandler;
  sigfillset(&action.sa_mask);
  action.sa_flags = SA_SIGINFO;
  sigaction(signum, &action, NULL);
}

int main(int argc, char* argv[])
{
  setSignalHandler(SIGSEGV);
  setSignalHandler(SIGABRT);

  ros::init(argc, argv, "mcl_3dl");

  mcl_3dl::MCL3dlNode mcl;
  if (!mcl.configure())
  {
    return 1;
  }
  ros::spin();

  return 0;
}
