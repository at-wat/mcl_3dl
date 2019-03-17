/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */

#ifndef PCL18_BACKPORTS_CENTROID_H
#define PCL18_BACKPORTS_CENTROID_H

#include <pcl/common/centroid.h>

#ifdef BACKPORT_PCL_CENTROID

// System has old PCL; backport CentroidPoint class from pcl-1.8

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>

#include <pcl18_backports/accumulators.h>

namespace pcl
{
/** A generic class that computes the centroid of points fed to it.
    *
    * Here by "centroid" we denote not just the mean of 3D point coordinates,
    * but also mean of values in the other data fields. The general-purpose
    * \ref computeNDCentroid() function also implements this sort of
    * functionality, however it does it in a "dumb" way, i.e. regardless of the
    * semantics of the data inside a field it simply averages the values. In
    * certain cases (e.g. for \c x, \c y, \c z, \c intensity fields) this
    * behavior is reasonable, however in other cases (e.g. \c rgb, \c rgba,
    * \c label fields) this does not lead to meaningful results.
    *
    * This class is capable of computing the centroid in a "smart" way, i.e.
    * taking into account the meaning of the data inside fields. Currently the
    * following fields are supported:
    *
    *  Data      | Point fields                          | Algorithm
    *  --------- | ------------------------------------- | -------------------------------------------------------------------------------------------
    *  XYZ       | \c x, \c y, \c z                      | Average (separate for each field)
    *  Normal    | \c normal_x, \c normal_y, \c normal_z | Average (separate for each field), resulting vector is normalized
    *  Curvature | \c curvature                          | Average
    *  Color     | \c rgb or \c rgba                     | Average (separate for R, G, B, and alpha channels)
    *  Intensity | \c intensity                          | Average
    *  Label     | \c label                              | Majority vote; if several labels have the same largest support then the  smaller label wins
    *
    * The template parameter defines the type of points that may be accumulated
    * with this class. This may be an arbitrary PCL point type, and centroid
    * computation will happen only for the fields that are present in it and are
    * supported.
    *
    * Current centroid may be retrieved at any time using get(). Note that the
    * function is templated on point type, so it is possible to fetch the
    * centroid into a point type that differs from the type of points that are
    * being accumulated. All the "extra" fields for which the centroid is not
    * being calculated will be left untouched.
    *
    * Example usage:
    *
    * \code
    * // Create and accumulate points
    * CentroidPoint<pcl::PointXYZ> centroid;
    * centroid.add (pcl::PointXYZ (1, 2, 3);
    * centroid.add (pcl::PointXYZ (5, 6, 7);
    * // Fetch centroid using `get()`
    * pcl::PointXYZ c1;
    * centroid.get (c1);
    * // The expected result is: c1.x == 3, c1.y == 4, c1.z == 5
    * // It is also okay to use `get()` with a different point type
    * pcl::PointXYZRGB c2;
    * centroid.get (c2);
    * // The expected result is: c2.x == 3, c2.y == 4, c2.z == 5,
    * // and c2.rgb is left untouched
    * \endcode
    *
    * \note Assumes that the points being inserted are valid.
    *
    * \note This class template can be successfully instantiated for *any*
    * PCL point type. Of course, each of the field averages is computed only if
    * the point type has the corresponding field.
    *
    * \ingroup common
    * \author Sergey Alexandrov */
template <typename PointT>
class CentroidPoint
{
public:
  CentroidPoint() = default;

  /** Add a new point to the centroid computation.
        *
        * In this function only the accumulators and point counter are updated,
        * actual centroid computation does not happen until get() is called. */
  void add(const PointT& point)
  {
    // Invoke add point on each accumulator
    boost::fusion::for_each(accumulators_, detail::AddPoint<PointT>(point));
    ++num_points_;
  }

  /** Retrieve the current centroid.
        *
        * Computation (division of accumulated values by the number of points
        * and normalization where applicable) happens here. The result is not
        * cached, so any subsequent call to this function will trigger
        * re-computation.
        *
        * If the number of accumulated points is zero, then the point will be
        * left untouched. */
  template <typename PointOutT>
  void get(PointOutT& point) const
  {
    if (num_points_ != 0)
    {
      // Filter accumulators so that only those that are compatible with
      // both PointT and requested point type remain
      auto ca = boost::fusion::filter_if<detail::IsAccumulatorCompatible<PointT, PointOutT>>(accumulators_);
      // Invoke get point on each accumulator in filtered list
      boost::fusion::for_each(ca, detail::GetPoint<PointOutT>(point, num_points_));
    }
  }

  /** Get the total number of points that were added. */
  size_t getSize() const
  {
    return (num_points_);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  size_t num_points_ = 0;
  typename pcl::detail::Accumulators<PointT>::type accumulators_;
};

}  // namespace pcl

#endif  // BACKPORT_PCL_CENTROID
#endif  // PCL18_BACKPORTS_CENTROID_H
