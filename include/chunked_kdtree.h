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

#ifndef CHUNKED_KDTREE_H
#define CHUNKED_KDTREE_H

#include <pcl/point_types.h>

#include <unordered_map>
#include <vector>

template <typename POINT_TYPE>
class ChunkedKdtree
{
protected:
  class ChunkId
  {
  public:
    int x_;
    int y_;
    int z_;
    constexpr bool operator==(const ChunkId &a) const
    {
      return x_ == a.x_ && x_ == a.x_ && x_ == a.x_;
    }
    constexpr bool operator!=(const ChunkId &a) const
    {
      return !operator==(a);
    }
    size_t operator()(const ChunkId &id) const
    {
      return (id.x_) ^ (id.y_ << 11) ^ (id.z_ << 22);
    }
  };
  class Chunk
  {
  public:
    pcl::KdTreeFLANN<POINT_TYPE> kdtree_;
  };

  const float pos_to_chunk_;
  float epsilon_;
  boost::shared_ptr<pcl::PointRepresentation<POINT_TYPE>> point_rep_;

  using ChunkMap = std::unordered_map<ChunkId, Chunk, ChunkId>;
  using ChunkCloud = std::unordered_map<ChunkId, typename pcl::PointCloud<POINT_TYPE>::Ptr, ChunkId>;
  ChunkMap chunks_;

public:
  using Ptr = std::shared_ptr<ChunkedKdtree>;
  explicit ChunkedKdtree(const float chunk_length = 5.0)
    : pos_to_chunk_(1.0 / chunk_length)
  {
    chunks_.clear();
  }
  void setEpsilon(const float epsilon)
  {
    epsilon_ = epsilon;
    for (auto &chunk : chunks_)
    {
      chunk.second.kdtree_.setEpsilon(epsilon_);
    }
  }
  void setPointRepresentation(
      boost::shared_ptr<pcl::PointRepresentation<POINT_TYPE>> point_rep)
  {
    point_rep_ = point_rep;
    for (auto &chunk : chunks_)
    {
      chunk.second.kdtree_.setPointRepresentation(point_rep_);
    }
  }
  void setInputCloud(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr cloud)
  {
    if (chunks_.size())
      chunks_.clear();
    ChunkCloud clouds;
    for (auto &p : *cloud)
    {
      const auto chunk_id = getChunkId(p);
      if (!clouds[chunk_id])
      {
        clouds[chunk_id].reset(new pcl::PointCloud<POINT_TYPE>);
      }
      clouds[chunk_id]->push_back(p);
    }
    for (auto &cloud : clouds)
    {
      chunks_[cloud.first].kdtree_.setPointRepresentation(point_rep_);
      chunks_[cloud.first].kdtree_.setEpsilon(epsilon_);
      chunks_[cloud.first].kdtree_.setInputCloud(cloud.second);
    }
  }
  bool radiusSearch(
      const POINT_TYPE &p,
      const float &radius,
      std::vector<int> &id,
      std::vector<float> &dist_sq,
      const size_t &num)
  {
    const auto chunk_id = getChunkId(p);
    if (chunks_.find(chunk_id) == chunks_.end())
      return false;

    return chunks_[chunk_id].kdtree_.radiusSearch(p, radius, id, dist_sq, num);
  }
  void nearestKSearch(
      const POINT_TYPE &p,
      const size_t &num,
      std::vector<int> &id,
      std::vector<float> &dist_sq)
  {
    const auto chunk_id = getChunkId(p);
    if (chunks_.find(chunk_id) == chunks_.end())
      return;

    chunks_[chunk_id].kdtree_.nearestKSearch(p, num, id, dist_sq);
  }

protected:
  ChunkId getChunkId(const POINT_TYPE p)
  {
    ChunkId ret;
    ret.x_ = lroundf(p.x * pos_to_chunk_);
    ret.y_ = lroundf(p.y * pos_to_chunk_);
    ret.z_ = lroundf(p.z * pos_to_chunk_);

    return ret;
  }
};

#endif  // CHUNKED_KDTREE_H
