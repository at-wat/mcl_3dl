/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL18_BACKPORTS_VOXEL_GRID_H
#define PCL18_BACKPORTS_VOXEL_GRID_H

#ifndef BACKPORT_PCL_VOXEL_GRID

#include <pcl/filters/voxel_grid.h>

namespace pcl
{
template <typename PointT>
using VoxelGrid18 = VoxelGrid<PointT>;
}

#else

// System has old PCL; backport VoxelGrid from pcl-1.8

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>

namespace pcl
{
/** \brief VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
   *
   * The VoxelGrid class creates a *3D voxel grid* (think about a voxel
   * grid as a set of tiny 3D boxes in space) over the input point cloud data.
   * Then, in each *voxel* (i.e., 3D box), all the points present will be
   * approximated (i.e., *downsampled*) with their centroid. This approach is
   * a bit slower than approximating them with the center of the voxel, but it
   * represents the underlying surface more accurately.
   *
   * \author Radu B. Rusu, Bastian Steder
   * \ingroup filters
   */
template <typename PointT>
class VoxelGrid18 : public Filter<PointT>
{
protected:
  using Filter<PointT>::filter_name_;
  using Filter<PointT>::getClassName;
  using Filter<PointT>::input_;
  using Filter<PointT>::indices_;

  typedef typename Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
  typedef boost::shared_ptr<VoxelGrid18<PointT>> Ptr;
  typedef boost::shared_ptr<const VoxelGrid18<PointT>> ConstPtr;

  /** \brief Empty constructor. */
  VoxelGrid18()
    : leaf_size_(Eigen::Vector4f::Zero())
    , inverse_leaf_size_(Eigen::Array4f::Zero())
    , leaf_layout_()
    , min_b_(Eigen::Vector4i::Zero())
    , max_b_(Eigen::Vector4i::Zero())
    , div_b_(Eigen::Vector4i::Zero())
    , divb_mul_(Eigen::Vector4i::Zero())
    , filter_limit_min_(-FLT_MAX)
    , filter_limit_max_(FLT_MAX)
    , filter_limit_negative_(false)
    , min_points_per_voxel_(0)
  {
    filter_name_ = "VoxelGrid";
  }

  /** \brief Destructor. */
  virtual ~VoxelGrid18()
  {
  }

  /** \brief Set the voxel grid leaf size.
     * \param[in] leaf_size the voxel grid leaf size
     */
  inline void
  setLeafSize(const Eigen::Vector4f& leaf_size)
  {
    leaf_size_ = leaf_size;
    // Avoid division errors
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    // Use multiplications instead of divisions
    inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
  }

  /** \brief Set the voxel grid leaf size.
     * \param[in] lx the leaf size for X
     * \param[in] ly the leaf size for Y
     * \param[in] lz the leaf size for Z
     */
  inline void
  setLeafSize(float lx, float ly, float lz)
  {
    leaf_size_[0] = lx;
    leaf_size_[1] = ly;
    leaf_size_[2] = lz;
    // Avoid division errors
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    // Use multiplications instead of divisions
    inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
  }

  /** \brief Get the voxel grid leaf size. */
  inline Eigen::Vector3f
  getLeafSize() const
  {
    return (leaf_size_.head<3>());
  }

  /** \brief Set the minimum number of points required for a voxel to be used.
     * \param[in] min_points_per_voxel the minimum number of points for required for a voxel to be used
     */
  inline void
  setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel)
  {
    min_points_per_voxel_ = min_points_per_voxel;
  }

  /** \brief Return the minimum number of points required for a voxel to be used.
    */
  inline unsigned int
  getMinimumPointsNumberPerVoxel() const
  {
    return min_points_per_voxel_;
  }

  /** \brief Get the minimum coordinates of the bounding box (after
     * filtering is performed).
     */
  inline Eigen::Vector3i
  getMinBoxCoordinates() const
  {
    return (min_b_.head<3>());
  }

  /** \brief Get the minimum coordinates of the bounding box (after
     * filtering is performed).
     */
  inline Eigen::Vector3i
  getMaxBoxCoordinates() const
  {
    return (max_b_.head<3>());
  }

  /** \brief Get the number of divisions along all 3 axes (after filtering
     * is performed).
     */
  inline Eigen::Vector3i
  getNrDivisions() const
  {
    return (div_b_.head<3>());
  }

  /** \brief Get the multipliers to be applied to the grid coordinates in
     * order to find the centroid index (after filtering is performed).
     */
  inline Eigen::Vector3i
  getDivisionMultiplier() const
  {
    return (divb_mul_.head<3>());
  }

  /** \brief Returns the index in the resulting downsampled cloud of the specified point.
     *
     * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering
     * performed, and that the point is inside the grid, to avoid invalid access (or use
     * getGridCoordinates+getCentroidIndexAt)
     *
     * \param[in] p the point to get the index at
     */
  inline int
  getCentroidIndex(const PointT& p) const
  {
    return (leaf_layout_.at((Eigen::Vector4i(static_cast<int>(floor(p.x * inverse_leaf_size_[0])),
                                             static_cast<int>(floor(p.y * inverse_leaf_size_[1])),
                                             static_cast<int>(floor(p.z * inverse_leaf_size_[2])), 0) -
                             min_b_)
                                .dot(divb_mul_)));
  }

  /** \brief Returns the indices in the resulting downsampled cloud of the points at the specified grid coordinates,
     * relative to the grid coordinates of the specified point (or -1 if the cell was empty/out of bounds).
     * \param[in] reference_point the coordinates of the reference point (corresponding cell is allowed to be empty/out of bounds)
     * \param[in] relative_coordinates matrix with the columns being the coordinates of the requested cells, relative to the reference point's cell
     * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed
     */
  inline std::vector<int>
  getNeighborCentroidIndices(const PointT& reference_point, const Eigen::MatrixXi& relative_coordinates) const
  {
    Eigen::Vector4i ijk(static_cast<int>(floor(reference_point.x * inverse_leaf_size_[0])),
                        static_cast<int>(floor(reference_point.y * inverse_leaf_size_[1])),
                        static_cast<int>(floor(reference_point.z * inverse_leaf_size_[2])), 0);
    Eigen::Array4i diff2min = min_b_ - ijk;
    Eigen::Array4i diff2max = max_b_ - ijk;
    std::vector<int> neighbors(relative_coordinates.cols());
    for (int ni = 0; ni < relative_coordinates.cols(); ni++)
    {
      Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
      // checking if the specified cell is in the grid
      if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
        neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot(divb_mul_))];  // .at() can be omitted
      else
        neighbors[ni] = -1;  // cell is out of bounds, consider it empty
    }
    return (neighbors);
  }

  /** \brief Returns the layout of the leafs for fast access to cells relative to current position.
     * \note position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the grid (-1 if empty)
     */
  inline std::vector<int>
  getLeafLayout() const
  {
    return (leaf_layout_);
  }

  /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z).
     * \param[in] x the X point coordinate to get the (i, j, k) index at
     * \param[in] y the Y point coordinate to get the (i, j, k) index at
     * \param[in] z the Z point coordinate to get the (i, j, k) index at
     */
  inline Eigen::Vector3i
  getGridCoordinates(float x, float y, float z) const
  {
    return (Eigen::Vector3i(static_cast<int>(floor(x * inverse_leaf_size_[0])),
                            static_cast<int>(floor(y * inverse_leaf_size_[1])),
                            static_cast<int>(floor(z * inverse_leaf_size_[2]))));
  }

  /** \brief Returns the index in the downsampled cloud corresponding to a given set of coordinates.
     * \param[in] ijk the coordinates (i,j,k) in the grid (-1 if empty)
     */
  inline int
  getCentroidIndexAt(const Eigen::Vector3i& ijk) const
  {
    int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot(divb_mul_);
    if (idx < 0 || idx >= static_cast<int>(leaf_layout_.size()))
    // this checks also if leaf_layout_.size () == 0 i.e. everything was computed as needed
    {
      return (-1);
    }
    return (leaf_layout_[idx]);
  }

  /** \brief Set the field filter limits. All points having field values outside this interval will be discarded.
     * \param[in] limit_min the minimum allowed field value
     * \param[in] limit_max the maximum allowed field value
     */
  inline void
  setFilterLimits(const double& limit_min, const double& limit_max)
  {
    filter_limit_min_ = limit_min;
    filter_limit_max_ = limit_max;
  }

  /** \brief Get the field filter limits (min/max) set by the user. The default values are -FLT_MAX, FLT_MAX.
     * \param[out] limit_min the minimum allowed field value
     * \param[out] limit_max the maximum allowed field value
     */
  inline void
  getFilterLimits(double& limit_min, double& limit_max) const
  {
    limit_min = filter_limit_min_;
    limit_max = filter_limit_max_;
  }

  /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
     * Default: false.
     * \param[in] limit_negative return data inside the interval (false) or outside (true)
     */
  inline void
  setFilterLimitsNegative(const bool limit_negative)
  {
    filter_limit_negative_ = limit_negative;
  }

  /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
     * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
     */
  inline void
  getFilterLimitsNegative(bool& limit_negative) const
  {
    limit_negative = filter_limit_negative_;
  }

  /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
     * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
     */
  inline bool
  getFilterLimitsNegative() const
  {
    return (filter_limit_negative_);
  }

protected:
  /** \brief The size of a leaf. */
  Eigen::Vector4f leaf_size_;

  /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
  Eigen::Array4f inverse_leaf_size_;

  /** \brief The leaf layout information for fast access to cells relative to current position **/
  std::vector<int> leaf_layout_;

  /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
  Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside
        (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
  bool filter_limit_negative_;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  unsigned int min_points_per_voxel_;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  /** \brief Downsample a Point Cloud using a voxelized grid approach
     * \param[out] output the resultant point cloud message
     */
  void
  applyFilter(PointCloud& output)
  {
    // Has the input dataset been set already?
    if (!input_)
    {
      PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
      output.width = output.height = 0;
      output.points.clear();
      return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;       // downsampling breaks the organized structure
    output.is_dense = true;  // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
      PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.",
               getClassName().c_str());
      output = *input_;
      return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int>(floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int>(floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_->size());

    // No distance filtering, process all data
    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite(input_->points[*it].x) ||
            !pcl_isfinite(input_->points[*it].y) ||
            !pcl_isfinite(input_->points[*it].z))
          continue;

      int ijk0 = static_cast<int>(
          floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
      int ijk1 = static_cast<int>(
          floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
      int ijk2 = static_cast<int>(
          floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int>(idx), *it));
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

    // Third pass: count output cells
    // we need to skip all the same, adjacent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int>> first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
      unsigned int i = index + 1;
      while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
        ++i;
      if (i - index >= min_points_per_voxel_)
      {
        ++total;
        first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
      }
      index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    output.points.resize(total);

    index = 0;
    for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
    {
      // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
      unsigned int first_index = first_and_last_indices_vector[cp].first;
      unsigned int last_index = first_and_last_indices_vector[cp].second;

      CentroidPoint<PointT> centroid;

      // fill in the accumulator with leaf points
      for (unsigned int li = first_index; li < last_index; ++li)
        centroid.add(input_->points[index_vector[li].cloud_point_index]);

      centroid.get(output.points[index]);

      ++index;
    }
    output.width = static_cast<uint32_t>(output.points.size());
  }
};
}  // namespace pcl

#endif  // BACKPORT_PCL_VOXEL_GRID

#endif  // PCL18_BACKPORTS_VOXEL_GRID_H
