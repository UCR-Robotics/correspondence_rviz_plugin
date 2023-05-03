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

#include <sstream>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "line.h"

namespace rviz {
Line2::Line2(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node) : Object(manager) {
  if (!parent_node) {
    parent_node = manager->getRootSceneNode();
  }
  manual_object_ = manager->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "LineMaterial" << count++;

  // NOTE: The second parameter to the create method is the resource group the material will be added to.
  // If the group you name does not exist (in your resources.cfg file) the library will assert() and your
  // program will crash
  manual_object_material_ =
      Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  manual_object_material_->setReceiveShadows(false);
  manual_object_material_->getTechnique(0)->setLightingEnabled(true);
  manual_object_material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE | Ogre::TVC_SPECULAR | Ogre::TVC_EMISSIVE);

  scene_node_->attachObject(manual_object_);

  start_ = Ogre::Vector3(0,0,0);
  end_ = Ogre::Vector3(0,0,0);
  start_colour_ = Ogre::ColourValue(1,1,1,1);
  end_colour_ = Ogre::ColourValue(1,1,1,1);
}

Line2::~Line2() {
  if (scene_node_->getParentSceneNode()) {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
  Ogre::MaterialManager::getSingleton().remove(manual_object_material_->getName());
}

void Line2::setPoint(const Ogre::Vector3& start, const Ogre::Vector3& end) {
  start_ = start;
  end_ = end;
  setPointColour();
}

void Line2::setPointColour(){
  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
  manual_object_->position(start_);
  manual_object_->colour(start_colour_);
  manual_object_->position(end_);
  manual_object_->colour(end_colour_);
  manual_object_->end();
  setVisible(true);
}

void Line2::setVisible(bool visible) { scene_node_->setVisible(visible, true); }

void Line2::setPosition(const Ogre::Vector3& position) { scene_node_->setPosition(position); }

void Line2::setDirection(const Ogre::Vector3& direction) {
  if (!direction.isZeroLength()) {
    setOrientation((end_-start_).getRotationTo(direction));
  }
}

void Line2::setOrientation(const Ogre::Quaternion& orientation) { scene_node_->setOrientation(orientation); }

void Line2::setScale(const Ogre::Vector3& scale) { scene_node_->setScale(scale); }

void Line2::setColor(const Ogre::ColourValue& c) {
  start_colour_ = c;
  end_colour_ = c;
  setPointColour();
}

void Line2::setColor2(const Ogre::ColourValue& c) {
  end_colour_ = c;
  setPointColour();
}

void Line2::setColor(float r, float g, float b, float a) { setColor(Ogre::ColourValue(r, g, b, a)); }

void Line2::setColor2(float r, float g, float b, float a) { setColor2(Ogre::ColourValue(r, g, b, a)); }

// where are the void Line::setColour(...) convenience methods??? ;)

const Ogre::Vector3& Line2::getPosition() { return scene_node_->getPosition(); }

const Ogre::Quaternion& Line2::getOrientation() { return scene_node_->getOrientation(); }

void Line2::setUserData(const Ogre::Any& data) { manual_object_->getUserObjectBindings().setUserAny(data); }

float Line2::distance() const { return start_.distance(end_); }

}  // namespace rviz