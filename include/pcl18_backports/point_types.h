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

#ifndef PCL18_BACKPORTS_POINT_TYPES_H
#define PCL18_BACKPORTS_POINT_TYPES_H

#include <pcl/point_types.h>

#ifdef BACKPORT_PCL_CENTROID

#include <vector>
#include <bitset>

#include <pcl/pcl_macros.h>
#include <pcl/register_point_struct.h>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/vector.hpp>

namespace pcl
{
namespace traits
{
/** \brief Metafunction to check if a given point type has a given field.
     *
     *  Example usage at run-time:
     *
     *  \code
     *  bool curvature_available = pcl::traits::has_field<PointT, pcl::fields::curvature>::value;
     *  \endcode
     *
     *  Example usage at compile-time:
     *
     *  \code
     *  BOOST_MPL_ASSERT_MSG ((pcl::traits::has_field<PointT, pcl::fields::label>::value),
     *                        POINT_TYPE_SHOULD_HAVE_LABEL_FIELD,
     *                        (PointT));
     *  \endcode
     */
template <typename PointT, typename Field>
struct has_field : boost::mpl::contains<typename pcl::traits::fieldList<PointT>::type, Field>::type
{
};

/** Metafunction to check if a given point type has all given fields. */
template <typename PointT, typename Field>
struct has_all_fields : boost::mpl::fold<
                            Field,
                            boost::mpl::bool_<true>,
                            boost::mpl::and_<boost::mpl::_1,
                                             has_field<PointT, boost::mpl::_2>>>::type
{
};

/** Metafunction to check if a given point type has any of the given fields. */
template <typename PointT, typename Field>
struct has_any_field : boost::mpl::fold<
                           Field,
                           boost::mpl::bool_<false>,
                           boost::mpl::or_<boost::mpl::_1,
                                           has_field<PointT, boost::mpl::_2>>>::type
{
};

/** Metafunction to check if a given point type has x, y, and z fields. */
template <typename PointT>
struct has_xyz : has_all_fields<
                     PointT, boost::mpl::vector<pcl::fields::x,
                                                pcl::fields::y,
                                                pcl::fields::z>>
{
};

/** Metafunction to check if a given point type has normal_x, normal_y, and
      * normal_z fields. */
template <typename PointT>
struct has_normal : has_all_fields<
                        PointT, boost::mpl::vector<pcl::fields::normal_x,
                                                   pcl::fields::normal_y,
                                                   pcl::fields::normal_z>>
{
};

/** Metafunction to check if a given point type has curvature field. */
template <typename PointT>
struct has_curvature : has_field<PointT, pcl::fields::curvature>
{
};

/** Metafunction to check if a given point type has intensity field. */
template <typename PointT>
struct has_intensity : has_field<PointT, pcl::fields::intensity>
{
};

/** Metafunction to check if a given point type has either rgb or rgba field. */
template <typename PointT>
struct has_color : has_any_field<
                       PointT, boost::mpl::vector<pcl::fields::rgb,
                                                  pcl::fields::rgba>>
{
};

/** Metafunction to check if a given point type has label field. */
template <typename PointT>
struct has_label : has_field<PointT, pcl::fields::label>
{
};
}  // namespace traits
}  // namespace pcl

#endif  // BACKPORT_PCL_CENTROID
#endif  // PCL18_BACKPORTS_POINT_TYPES_H
