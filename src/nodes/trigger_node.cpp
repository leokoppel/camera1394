/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/

#include <signal.h>
#include <dc1394/dc1394.h>
#include <ros/ros.h>

/** @file

    @brief test ROS node for software triggering of IIDC-compatible
    IEEE 1394 digital cameras.

*/

/** Segfault signal handler.
 *
 *  Sadly, libdc1394 sometimes crashes.
 */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  ROS_FATAL("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

inline std::string stringFromGuid(uint64_t guid) {
  std::stringstream ss;
  ss << "[" << std::setw(16) << std::setfill('0') << std::hex << guid << "]";
  return ss.str();
}


class Camera {
public:
  Camera(dc1394_t *dc1394, uint64_t guid) :
    ptr_{dc1394_camera_new(dc1394, guid), dc1394_camera_free},
    name_{stringFromGuid(guid)}
  {
    ROS_INFO_STREAM("Connecting to camera, GUID: " << name_);
    if (!ptr_) {
      ROS_FATAL_STREAM(name_ << " failed to initialize camera");
      throw std::runtime_error("\"Failed to initialize camera");
    }
  }

  /** Send software trigger to camera.
   *  @returns true if able to set software trigger.
   */
  bool trigger(void)
  {
    ROS_DEBUG_STREAM(name_ << " triggering camera");
    dc1394error_t err = dc1394_software_trigger_set_power(ptr_.get(), DC1394_ON);
    bool retval = (err == DC1394_SUCCESS);
    if (!retval)
      ROS_FATAL_STREAM(name_ << " camera does not provide software trigger");
    return retval;
  }
  
private:
  std::shared_ptr<dc1394camera_t> ptr_;
  std::string name_;
};

class TriggerNode
{
public:
  /** Set up device connection to each camera on the bus.
   *
   *  @returns true if successful
   *
   *  Failure cleanup is sketchy, but that is OK because the node 
   *  terminates, anyway.
   */
  bool setup(void)
  {
    dc1394_t *dev = dc1394_new();
    if (dev == NULL)
      {
        ROS_FATAL("Failed to initialize dc1394_context.");
        return false;
      }

    // list all 1394 cameras attached to this computer
    dc1394camera_list_t *camera_list;
    int err = dc1394_camera_enumerate(dev, &camera_list);
    if (err != DC1394_SUCCESS)
      {
        ROS_FATAL("Could not get list of cameras");
        return false;
      }
    if (camera_list->num == 0)
      {
        ROS_FATAL("No cameras found");
        return false;
      }

    // attach to each camera found
    for (uint32_t i = 0; i < camera_list->num; ++i) {
      cameras_.emplace_back(dev, camera_list->ids[i].guid);
    }

    dc1394_camera_free_list(camera_list);
    return true;
  }

  /** spin, triggering the device twice a second. */
  bool spin(void)
  {
    bool retval = true;
    while (node_.ok()) {
        ros::spinOnce();
        for (auto& cam : cameras_) {
          retval = cam.trigger();
          if (!retval) {
            break;
          }
        }
        rate_.sleep();
      }
    return retval;
  }

private:
  ros::NodeHandle node_{"~"};
  std::vector<Camera> cameras_;
  ros::Rate rate_ = node_.param("rate", 2.0);
};

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera1394_trigger_node");
  signal(SIGSEGV, &sigsegv_handler);

  TriggerNode trig;

  if (!trig.setup())                    // device connection failed?
    return 1;

  if (!trig.spin())                     // device does not support trigger?
    return 2;

  return 0;
}
