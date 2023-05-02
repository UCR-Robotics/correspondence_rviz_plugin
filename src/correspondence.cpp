#include <QColor>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <ros/time.h>

#include <pluginlib/class_loader.hpp>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/validate_floats.h>

#include "line.h"
#include "correspondence.h"

namespace rviz {

PointCloudCorrespondenceDisplay::PointCloudCorrespondenceDisplay(){
    start_color_property_ = new ColorProperty("Line Color", QColor(0x66, 0xcc, 0xff), "Color of the line",
                                            this, SLOT(updateStyle()), this);
    alpha_property_ = new FloatProperty("Alpha", 1.0,
                                      "Amount of transparency to apply to the lines. "
                                      "Note that this is experimental and does not always look correct.",
                                      this, SLOT(updateStyle()), this);
    alpha_property_->setMin(0);
    alpha_property_->setMax(1);
    gradient_color_enabled_property_ = new BoolProperty("Gradient Color", false, "Enable gradient color from start to end", 
                                            this, SLOT(updateStyle()), this);
    end_color_property_ = new ColorProperty("Line End Color", QColor(0x66, 0xcc, 0xff), "Color of the end of the line, if gradient is enabled",
                                            this, SLOT(updateStyle()), this);
    end_color_property_->hide();                                        
}

PointCloudCorrespondenceDisplay::~PointCloudCorrespondenceDisplay() {
  PointCloudCorrespondenceDisplay::unsubscribe();
  allocateLines(0);
}

void PointCloudCorrespondenceDisplay::onInitialize() {
  // Use the threaded queue for processing of incoming messages
  update_nh_.setCallbackQueue(context_->getThreadedQueue());

  MFDClass::onInitialize();
}

void PointCloudCorrespondenceDisplay::update(float wall_dt, float ros_dt) {}

void PointCloudCorrespondenceDisplay::reset() {
  MFDClass::reset();
  allocateLines(0);
}

void PointCloudCorrespondenceDisplay::allocateLines(size_t size){
    if(line_buffer_.size()<size){
        line_buffer_.resize(size);
        for(size_t i=line_buffer_.size();i<size;++i){
            line_buffer_[i] = new Line2(scene_manager_, scene_node_);
        }
    }
    else{
        for(size_t i=size;i<line_buffer_.size();++i){
            delete line_buffer_[i];
        }
        line_buffer_.resize(size);
    }
}

void PointCloudCorrespondenceDisplay::updateStyle(){
    QColor color_s = start_color_property_->getColor();
    QColor color_e = end_color_property_->getColor();
    float alpha = alpha_property_->getFloat();

    Ogre::ColourValue ogre_color_s(color_s.redF(),color_s.greenF(),color_s.blueF(),alpha);
    Ogre::ColourValue ogre_color_e(color_e.redF(),color_e.greenF(),color_e.blueF(),alpha);
    bool gradient = gradient_color_enabled_property_->getBool();

    for(auto&& ptr:line_buffer_){
        ptr->setColor(ogre_color_s);
    }
    end_color_property_->hide();
    if(gradient){
        end_color_property_->show();
        for(auto&& ptr:line_buffer_){
            ptr->setColor2(ogre_color_e);
        }
    }
}

void PointCloudCorrespondenceDisplay::processMessage(const correspondence_rviz_plugin::PointCloudCorrespondenceConstPtr& msg){
    if(msg->index_source.size() != msg->index_target.size()){
        setStatus(StatusProperty::Error, "Unmatched Index!","The size of the correspondence index of two point clouds are not match.");
        return;
    }

    Ogre::Vector3 position_s;
    Ogre::Quaternion orientation_s;
    if (!context_->getFrameManager()->getTransform(msg->cloud_target.header, position_s, orientation_s)) {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->cloud_target.header.frame_id.c_str(),
                qPrintable(context_->getFixedFrame()));
        return;
    }
    Ogre::Matrix4 transform_s(orientation_s);
    transform_s.setTrans(position_s);

    auto points_s = transform_pointcloud(msg->cloud_target, transform_s, msg->index_target);

    Ogre::Vector3 position_e;
    Ogre::Quaternion orientation_e;
    if (!context_->getFrameManager()->getTransform(msg->cloud_source.header, position_e, orientation_e)) {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->cloud_source.header.frame_id.c_str(),
                qPrintable(context_->getFixedFrame()));
        return;
    }
    Ogre::Matrix4 transform_e(orientation_e);
    transform_e.setTrans(position_e);

    auto points_e = transform_pointcloud(msg->cloud_source, transform_e, msg->index_source);

    allocateLines(points_s.size());

    for(size_t i=0;i<points_s.size();++i){
        line_buffer_[i]->setPoint(points_s[i], points_e[i]);
    }
    updateStyle();
}

int32_t findChannelIndex(const sensor_msgs::PointCloud2& cloud, const std::string& channel)
{
  for (size_t i = 0; i < cloud.fields.size(); ++i)
  {
    if (cloud.fields[i].name == channel)
    {
      return i;
    }
  }

  return -1;
}

std::vector<Ogre::Vector3> PointCloudCorrespondenceDisplay::transform_pointcloud(const sensor_msgs::PointCloud2 &cloud, const Ogre::Matrix4 &transform, const std::vector<int> &indices){
    std::vector<Ogre::Vector3> ret(indices.size());

    int32_t xi = findChannelIndex(cloud, "x");
    int32_t yi = findChannelIndex(cloud, "y");
    int32_t zi = findChannelIndex(cloud, "z");
    const uint32_t xoff = cloud.fields[xi].offset;
    const uint32_t yoff = cloud.fields[yi].offset;
    const uint32_t zoff = cloud.fields[zi].offset;
    const uint32_t point_step = cloud.point_step;

    for (uint32_t i = 0; i < indices.size(); ++i) {
        const uint8_t* ptr = cloud.data.data() + indices[i] * point_step;
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);
        Ogre::Vector3 p(x,y,z);
        ret[i] = transform*p;
    }
    return ret;
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::PointCloudCorrespondenceDisplay, rviz::Display)