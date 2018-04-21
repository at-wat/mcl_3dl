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

#ifndef COMPATIBILITY_H
#define COMPATIBILITY_H

#include <ros/ros.h>

namespace mcl_3dl_compat
{
bool getCompat()
{
  bool compat(true);
  ros::NodeHandle("~").param("compatible", compat, compat);

  return compat;
}
void checkCompatMode()
{
  if (getCompat())
  {
    ROS_ERROR_ONCE(
        "======= [Deprecated] %s is run in compatible mode =======\n"
        "=========================================================\n"
        "Set _compatible:=false to switch to new topic namespaces.\n"
        "Compatible mode will be obsolated in the future update.\n"
        "=========================================================",
        ros::this_node::getName().c_str());
  }
}
std::string getSimplifiedNamespace(ros::NodeHandle &nh)
{
  if (nh.getUnresolvedNamespace() == ros::this_node::getName())
    return std::string("~/");
  if (nh.getUnresolvedNamespace() == std::string())
    return std::string();
  return nh.getNamespace() + "/";
}
template <class M, class T>
ros::Subscriber subscribe(
    ros::NodeHandle &nh_new,
    const std::string &topic_new,
    ros::NodeHandle &nh_old,
    const std::string &topic_old,
    uint32_t queue_size,
    void (T::*fp)(M) const,
    T *obj,
    const ros::TransportHints &transport_hints = ros::TransportHints())
{
  if (getCompat())
  {
    ROS_ERROR(
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, fp, obj, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, fp, obj, transport_hints);
  }
}
template <class M, class T>
ros::Subscriber subscribe(
    ros::NodeHandle &nh_new,
    const std::string &topic_new,
    ros::NodeHandle &nh_old,
    const std::string &topic_old,
    uint32_t queue_size,
    void (T::*fp)(M),
    T *obj,
    const ros::TransportHints &transport_hints = ros::TransportHints())
{
  if (getCompat())
  {
    ROS_ERROR(
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, fp, obj, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, fp, obj, transport_hints);
  }
}
template <class M>
ros::Publisher advertise(
    ros::NodeHandle &nh_new,
    const std::string &topic_new,
    ros::NodeHandle &nh_old,
    const std::string &topic_old,
    uint32_t queue_size,
    bool latch = false)
{
  if (getCompat())
  {
    ROS_ERROR(
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.advertise<M>(topic_old, queue_size, latch);
  }
  else
  {
    return nh_new.advertise<M>(topic_new, queue_size, latch);
  }
}
template <class T, class MReq, class MRes>
ros::ServiceServer advertiseService(
    ros::NodeHandle &nh_new,
    const std::string &service_new,
    ros::NodeHandle &nh_old,
    const std::string &service_old,
    bool (T::*srv_func)(MReq &, MRes &),
    T *obj)
{
  if (getCompat())
  {
    ROS_ERROR(
        "Use %s (%s%s) service instead of %s (%s%s)",
        nh_new.resolveName(service_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), service_new.c_str(),
        nh_old.resolveName(service_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), service_old.c_str());
    return nh_old.advertiseService(service_old, srv_func, obj);
  }
  else
  {
    return nh_new.advertiseService(service_new, srv_func, obj);
  }
}
}

#endif  // COMPATIBILITY_H
