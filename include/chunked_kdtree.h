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

#include <stdexcept>
#include <unordered_map>
#include <vector>

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
  };

  const float pos_to_chunk_;
  const float chunk_length_;
  float epsilon_;
  boost::shared_ptr<pcl::PointRepresentation<POINT_TYPE>> point_rep_;

  using ChunkMap = std::unordered_map<ChunkId, Chunk, ChunkId>;
  using ChunkCloud = std::unordered_map<ChunkId, typename pcl::PointCloud<POINT_TYPE>::Ptr, ChunkId>;
  ChunkMap chunks_;

public:
  using Ptr = std::shared_ptr<ChunkedKdtree>;
  explicit ChunkedKdtree(const float chunk_length = 20.0)
    : pos_to_chunk_(1.0 / chunk_length)
    , chunk_length_(chunk_length)
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
  static void mergeIdDist(
      std::vector<int> &id, std::vector<float> &dist_sq,
      const std::vector<int> &id2, const std::vector<float> &dist_sq2,
      const size_t &num)
  {
    auto it_id2 = id2.begin();
    auto it_dist2 = dist_sq2.begin();

    auto it_dist = dist_sq.begin();
    auto it_id = id.begin();
    for (; it_id != id.end(); ++it_id, ++it_dist)
    {
      if (*it_dist > *it_dist2)
      {
        it_dist = dist_sq.insert(it_dist, *it_dist2);
        it_id = id.insert(it_id, *it_id2);
        ++it_dist2;
        ++it_id2;
        if (it_id2 == id2.end())
          break;
      }
      if (it_id - id.begin() >= num)
        break;
    }
    if (id.size() >= num)
    {
      id.erase(id.begin() + num, id.end());
      dist_sq.erase(dist_sq.begin() + num, dist_sq.end());
    }
  }
  void mergeRadiusSearch(
      const POINT_TYPE &p,
      const float &radius,
      std::vector<int> &id,
      std::vector<float> &dist_sq,
      const size_t &num,
      const ChunkId &chunk_id,
      const ChunkId &rel_pos,
      const float remain_dist_sq)
  {
    const ChunkId chunk_id2(
        chunk_id.x_ + rel_pos.x_,
        chunk_id.y_ + rel_pos.y_,
        chunk_id.z_ + rel_pos.z_);
    if (chunks_.find(chunk_id2) != chunks_.end())
    {
      std::vector<int> id2;
      std::vector<float> dist_sq2;

      size_t num_remain = num;
      for (auto &d_sq : dist_sq)
      {
        if (d_sq < remain_dist_sq)
          --num_remain;
        else
          break;
      }
      if (num_remain == 0 || num_remain > num)
        return;
      const int result = chunks_[chunk_id2].kdtree_.radiusSearch(p, radius, id2, dist_sq2, num_remain);
      if (result > 0)
        mergeIdDist(id, dist_sq, id2, dist_sq2, num);
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
    {
      throw std::runtime_error("ChunkedKdtree: radius must be <chunk_length");
    }

    const auto chunk_id = getChunkId(p);
    if (chunks_.find(chunk_id) == chunks_.end())
      return false;

    const int result = chunks_[chunk_id].kdtree_.radiusSearch(p, radius, id, dist_sq, num);

    const float in_chunk_x = p.x - chunk_id.x_ * chunk_length_;
    int x_bound = 0;
    float dist_sq_x = 0;
    if (in_chunk_x < radius)
    {
      x_bound = -1;
      dist_sq_x = powf(in_chunk_x, 2.0);
    }
    else if (in_chunk_x > chunk_length_ - radius)
    {
      x_bound = 1;
      dist_sq_x = powf(chunk_length_ - in_chunk_x, 2.0);
    }

    const float in_chunk_y = p.y - chunk_id.y_ * chunk_length_;
    int y_bound = 0;
    float dist_sq_y = 0;
    if (in_chunk_y < radius)
    {
      y_bound = -1;
      dist_sq_y = powf(in_chunk_y, 2.0);
    }
    else if (in_chunk_y > chunk_length_ - radius)
    {
      y_bound = 1;
      dist_sq_y = powf(chunk_length_ - in_chunk_y, 2.0);
    }

    const float in_chunk_z = p.z - chunk_id.z_ * chunk_length_;
    int z_bound = 0;
    float dist_sq_z = 0;
    if (in_chunk_z < radius)
    {
      z_bound = -1;
      dist_sq_z = powf(in_chunk_z, 2.0);
    }
    else if (in_chunk_z > chunk_length_ - radius)
    {
      z_bound = 1;
      dist_sq_z = powf(chunk_length_ - in_chunk_z, 2.0);
    }

    if (x_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(x_bound, 0, 0), dist_sq_x);
    if (y_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(0, y_bound, 0), dist_sq_y);
    if (z_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(0, 0, z_bound), dist_sq_z);
    if (x_bound && y_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(x_bound, y_bound, 0),
                        dist_sq_x + dist_sq_y);
    if (y_bound && z_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(0, y_bound, z_bound),
                        dist_sq_y + dist_sq_z);
    if (z_bound && x_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(x_bound, 0, z_bound),
                        dist_sq_z + dist_sq_x);
    if (x_bound && y_bound && z_bound)
      mergeRadiusSearch(p, radius, id, dist_sq, num, chunk_id, ChunkId(x_bound, y_bound, z_bound),
                        dist_sq_x + dist_sq_y + dist_sq_z);

    return result;
  }

protected:
  ChunkId getChunkId(const POINT_TYPE p) const
  {
    return ChunkId(lroundf(p.x * pos_to_chunk_),
                   lroundf(p.y * pos_to_chunk_),
                   lroundf(p.z * pos_to_chunk_));
  }
};

#endif  // CHUNKED_KDTREE_H
