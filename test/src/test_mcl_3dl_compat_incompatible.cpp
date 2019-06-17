/*
 * Copyright (c) 2018, the mcl_3dl authors
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

#include <cstdlib>

#include <ros/ros.h>

#include <gtest/gtest.h>

#define UNDEF_COMPATIBILITY_LEVEL

namespace mcl_3dl_compat
{
int current_level;
int supported_level;
int default_level;
}  // namespace mcl_3dl_compat
#include <mcl_3dl_compat/compatibility.h>

int test_mode = 0;

TEST(Mcl3DlCompat, CompatMode)
{
  mcl_3dl_compat::supported_level = 2;
  mcl_3dl_compat::current_level = 3;
  mcl_3dl_compat::default_level = mcl_3dl_compat::supported_level;

  ros::NodeHandle("~").setParam("compatible", 2);
  ASSERT_NO_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("~").setParam("compatible", 3);
  ASSERT_NO_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("~").setParam("compatible", test_mode ? 4 : 1);
  ASSERT_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      },  // NOLINT(whitespace/braces)
      std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mcl_3dl_compat_incompatible");

  if (argc != 2)
  {
    std::cerr << "test_mcl_3dl_compat_incompatible must have one argument (0 or 1)" << std::endl;
    return 1;
  }
  test_mode = std::atoi(argv[1]);

  return RUN_ALL_TESTS();
}
