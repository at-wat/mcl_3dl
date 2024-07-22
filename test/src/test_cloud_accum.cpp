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

#include <vector>
#include <string>

#include <sensor_msgs/PointCloud2.h>

#include <mcl_3dl/cloud_accum.h>

#include <gtest/gtest.h>

TEST(CloudAccumulationLogic, PassThrough)
{
  mcl_3dl::CloudAccumulationLogicPassThrough accum;
  std::string seq;
  const sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);

  const auto process = [&seq]()
  {
    seq += "p";
  };
  const auto accumulateOK = [&seq, msg](const sensor_msgs::PointCloud2::ConstPtr& msg2) -> bool
  {
    EXPECT_EQ(msg, msg2);
    seq += "a";
    return true;
  };
  const auto accumulateNG = [&seq, msg](const sensor_msgs::PointCloud2::ConstPtr& msg2) -> bool
  {
    EXPECT_EQ(msg, msg2);
    seq += "a'";
    return false;
  };
  const auto clear = [&seq]()
  {
    seq += "c";
  };
  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("cap"), seq);

  accum.push("2", msg, process, accumulateNG, clear);
  ASSERT_EQ(std::string("capca'"), seq);
}

TEST(CloudAccumulationLogic, Accumulate)
{
  mcl_3dl::CloudAccumulationLogic accum(2, 6);
  std::string seq;
  const sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);

  const auto process = [&seq]()
  {
    seq += "p";
  };
  const auto accumulateOK = [&seq, msg](const sensor_msgs::PointCloud2::ConstPtr& msg2) -> bool
  {
    EXPECT_EQ(msg, msg2);
    seq += "a";
    return true;
  };
  const auto accumulateNG = [&seq, msg](const sensor_msgs::PointCloud2::ConstPtr& msg2) -> bool
  {
    EXPECT_EQ(msg, msg2);
    seq += "a'";
    return false;
  };
  const auto clear = [&seq]()
  {
    seq += "c";
  };

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("a"), seq);

  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aa"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaa"), seq);

  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaa"), seq);

  // (accum_cloud_ + 1) of sensor 1 messages received.
  // Process and start next accumulation.
  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapca"), seq);

  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaa"), seq);

  // Sensor 2 publishes extra message.
  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaa"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaa"), seq);

  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaa"), seq);

  // Process and start next accumulation.
  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapca"), seq);

  // Failed to accumulate. Clear and start new accumulation.
  accum.push("2", msg, process, accumulateNG, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'c"), seq);

  accum.push("2", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'ca"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caa"), seq);

  // Sensor 2 stopped publishing message.
  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaa"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaaa"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaaaa"), seq);

  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaaaaa"), seq);

  // Exceeds accum_max_. Force processing and start next accumulation.
  accum.push("1", msg, process, accumulateOK, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaaaaapca"), seq);

  accum.push("1", msg, process, accumulateNG, clear);
  ASSERT_EQ(std::string("aaaapcaaaaapcaa'caaaaaapcaa'c"), seq);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
