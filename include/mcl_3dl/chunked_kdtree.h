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

#ifndef MCL_3DL_CHUNKED_KDTREE_H
#define MCL_3DL_CHUNKED_KDTREE_H

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace mcl_3dl
{
template <typename POINT_TYPE>
class ChunkedKdtree
{
protected:
  class ChunkId
  {
  public:
    const int x_;
    const int y_;
    const int z_;
    constexpr bool operator==(const ChunkId &a) const
    {
      return x_ == a.x_ && y_ == a.y_ && z_ == a.z_;
    }
    constexpr bool operator!=(const ChunkId &a) const
    {
      return !operator==(a);
    }
    constexpr ChunkId operator+(const ChunkId &a) const
    {
      return ChunkId(x_ + a.x_, y_ + a.y_, z_ + a.z_);
    }
    size_t operator()(const ChunkId &id) const
    {
      return (id.x_) ^ (id.y_ << 11) ^ (id.z_ << 22);
    }
    ChunkId(const int &x, const int &y, const int &z)
      : x_(x)
      , y_(y)
      , z_(z)
    {
    }
    ChunkId()
      : x_(0)
      , y_(0)
      , z_(0)
    {
    }
  };
  class Chunk
  {
  public:
    pcl::KdTreeFLANN<POINT_TYPE> kdtree_;
    std::vector<size_t> original_ids_;
  };

  const float pos_to_chunk_;
  const float chunk_length_;
  const float max_search_radius_;
  bool set_epsilon_;
  float epsilon_;
  boost::shared_ptr<pcl::PointRepresentation<POINT_TYPE>> point_rep_;

  using ChunkMap = std::unordered_map<ChunkId, Chunk, ChunkId>;
  using ChunkCloud = std::unordered_map<ChunkId, typename pcl::PointCloud<POINT_TYPE>, ChunkId>;
  using ChunkOriginalIds = std::unordered_map<ChunkId, std::vector<size_t>, ChunkId>;
  ChunkMap chunks_;

public:
  using Ptr = std::shared_ptr<ChunkedKdtree>;
  explicit ChunkedKdtree(const float chunk_length = 20.0, const float max_search_radius = 1.0)
    : pos_to_chunk_(1.0 / chunk_length)
    , chunk_length_(chunk_length)
    , max_search_radius_(max_search_radius)
    , set_epsilon_(false)
  {
    chunks_.clear();
  }
  void setEpsilon(const float epsilon)
  {
    epsilon_ = epsilon;
    set_epsilon_ = true;
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
    ChunkOriginalIds ids;
    size_t i = 0;
    for (auto &p : *cloud)
    {
      const auto chunk_id = getChunkId(p);
      clouds[chunk_id].push_back(p);
      ids[chunk_id].push_back(i);

      const float in_chunk_x = p.x - chunk_id.x_ * chunk_length_;
      int x_bound = 0;
      if (in_chunk_x < max_search_radius_)
        x_bound = -1;
      else if (in_chunk_x > chunk_length_ - max_search_radius_)
        x_bound = 1;

      const float in_chunk_y = p.y - chunk_id.y_ * chunk_length_;
      int y_bound = 0;
      if (in_chunk_y < max_search_radius_)
        y_bound = -1;
      else if (in_chunk_y > chunk_length_ - max_search_radius_)
        y_bound = 1;

      const float in_chunk_z = p.z - chunk_id.z_ * chunk_length_;
      int z_bound = 0;
      if (in_chunk_z < max_search_radius_)
        z_bound = -1;
      else if (in_chunk_z > chunk_length_ - max_search_radius_)
        z_bound = 1;

      if (x_bound && y_bound && z_bound)
      {
        const ChunkId id(chunk_id + ChunkId(x_bound, y_bound, z_bound));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (x_bound && y_bound)
      {
        const ChunkId id(chunk_id + ChunkId(x_bound, y_bound, 0));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (y_bound && z_bound)
      {
        const ChunkId id(chunk_id + ChunkId(0, y_bound, z_bound));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (z_bound && x_bound)
      {
        const ChunkId id(chunk_id + ChunkId(x_bound, 0, z_bound));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (x_bound)
      {
        const ChunkId id(chunk_id + ChunkId(x_bound, 0, 0));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (y_bound)
      {
        const ChunkId id(chunk_id + ChunkId(0, y_bound, 0));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      if (z_bound)
      {
        const ChunkId id(chunk_id + ChunkId(0, 0, z_bound));
        clouds[id].push_back(p);
        ids[id].push_back(i);
      }
      ++i;
    }
    for (auto &cloud : clouds)
    {
      if (point_rep_)
        chunks_[cloud.first].kdtree_.setPointRepresentation(point_rep_);
      if (set_epsilon_)
        chunks_[cloud.first].kdtree_.setEpsilon(epsilon_);
      chunks_[cloud.first].kdtree_.setInputCloud(cloud.second.makeShared());
      chunks_[cloud.first].original_ids_ = ids[cloud.first];
    }
  }
  int radiusSearch(
      const POINT_TYPE &p,
      const float &radius,
      std::vector<int> &id,
      std::vector<float> &dist_sq,
      const size_t &num)
  {
    if (radius > chunk_length_)
      throw std::runtime_error("ChunkedKdtree: radius must be <chunk_length");

    const auto chunk_id = getChunkId(p);
    if (chunks_.find(chunk_id) == chunks_.end())
      return false;

    const auto ret = chunks_[chunk_id].kdtree_.radiusSearch(p, radius, id, dist_sq, num);

    for (auto &i : id)
      i = chunks_[chunk_id].original_ids_[i];

    return ret;
  }

protected:
  ChunkId getChunkId(const POINT_TYPE p) const
  {
    return ChunkId(static_cast<int>(floor(p.x * pos_to_chunk_)),
                   static_cast<int>(floor(p.y * pos_to_chunk_)),
                   static_cast<int>(floor(p.z * pos_to_chunk_)));
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_CHUNKED_KDTREE_H
