/*
 * Copyright (c) 2023, ARCS Lab
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
 *     * Neither the name of the ARCS Lab nor the names of its
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

#ifndef CORRESPONDENCE_RVIZ_PLUGIN_CORRESPONDENCE_H
#define CORRESPONDENCE_RVIZ_PLUGIN_CORRESPONDENCE_H

#include <rviz/message_filter_display.h>
#include "correspondence_rviz_plugin/PointCloudCorrespondence.h"

namespace rviz {
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
class IntProperty;
class ColorProperty;
class Line2;

/**
 * \class PointCloudCorrespondenceDisplay
 * \brief Displays lines between point correspondences in two point clouds.
 */
class PointCloudCorrespondenceDisplay : public MessageFilterDisplay<correspondence_rviz_plugin::PointCloudCorrespondence> {
  Q_OBJECT
 public:
  PointCloudCorrespondenceDisplay();
  ~PointCloudCorrespondenceDisplay() override;

  void reset() override;

  void update(float wall_dt, float ros_dt) override;
  void allocateLines(size_t size);

  std::vector<Ogre::Vector3> transform_pointcloud(const sensor_msgs::PointCloud2& point_cloud, const Ogre::Matrix4& transform, const std::vector<int>& indices);

private Q_SLOTS:
  void updateStyle();
  void updateLengthLimit();

 protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(const correspondence_rviz_plugin::PointCloudCorrespondenceConstPtr& msg) override;

private:

  ColorProperty* start_color_property_;
  ColorProperty* end_color_property_;
  BoolProperty* gradient_color_enabled_property_;
  FloatProperty* alpha_property_;
  BoolProperty* max_length_enable_property_;
  FloatProperty* max_length_property_;
  BoolProperty* min_length_enable_property_;
  FloatProperty* min_length_property_;

  std::vector<rviz::Line2*> line_buffer_;
};

}  // namespace rviz

#endif  // CORRESPONDENCE_RVIZ_PLUGIN_CORRESPONDENCE_H