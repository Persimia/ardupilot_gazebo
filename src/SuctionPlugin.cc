/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Collision.hh"

#include "gz/sim/components/ExternalWorldWrenchCmd.hh"

#include "SuctionPlugin.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/////////////////////////////////////////////////
void SuctionPlugin::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &/*_eventMgr*/)
{
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
        gzerr << "SuctionPlugin should be attached to a model entity. "
                     << "Failed to initialize." << std::endl;
        return;
    }

    if (_sdf->HasElement("parent_link"))
    {
        auto parentLinkName = _sdf->Get<std::string>("parent_link");
        this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
        if (kNullEntity == this->parentLinkEntity)
        {
            gzerr << "Link with name " << parentLinkName
                         << " not found in model " << this->model.Name(_ecm)
                         << ". Make sure the parameter 'parent_link' has the "
                         << "correct value. Failed to initialize.\n";
            return;
        }
    }
    else
    {
        gzerr << "'parent_link' is a required parameter for SuctionPlugin. "
                            "Failed to initialize.\n";
        return;
    }

    if (_sdf->HasElement("suction_force"))
    {
        this->suction_force = _sdf->Get<std::float_t>("suction_force");
    }
    else
    {
        gzerr << "'suction_force' is a required parameter for SuctionPlugin."
                            "Failed to initialize.\n";
        return;
    }

    // Setup detach topic
    std::vector<std::string> detachTopics;
    if (_sdf->HasElement("detach_topic"))
    {
        detachTopics.push_back(_sdf->Get<std::string>("detach_topic"));
    }
    detachTopics.push_back("/model/" + this->model.Name(_ecm) +
            "/detachable_joint/detach");

    if (_sdf->HasElement("topic"))
    {
        if (_sdf->HasElement("detach_topic"))
        {
            if (_sdf->Get<std::string>("topic") !=
                    _sdf->Get<std::string>("detach_topic"))
            {
                gzerr << "<topic> and <detach_topic> tags have different contents. "
                                 "Please verify the correct string and use <detach_topic>."
                            << std::endl;
            }
            else
            {
                gzdbg << "Ignoring <topic> tag and using <detach_topic> tag."
                            << std::endl;
            }
        }
        else
        {
            detachTopics.insert(detachTopics.begin(),
                                                    _sdf->Get<std::string>("topic"));
        }
    }

    this->detachTopic = validTopic(detachTopics);
    if (this->detachTopic.empty())
    {
        gzerr << "No valid detach topics for SuctionPlugin could be found.\n";
        return;
    }
    gzdbg << "Detach topic is: " << this->detachTopic << std::endl;

    // Setup subscriber for detach topic
    this->node.Subscribe(
            this->detachTopic, &SuctionPlugin::OnDetachRequest, this);

    gzdbg << "SuctionPlugin subscribing to messages on "
                 << "[" << this->detachTopic << "]" << std::endl;

    // Setup attach topic
    std::vector<std::string> attachTopics;
    if (_sdf->HasElement("attach_topic"))
    {
        attachTopics.push_back(_sdf->Get<std::string>("attach_topic"));
    }
    attachTopics.push_back("/model/" + this->model.Name(_ecm) +
            "/detachable_joint/attach");
    this->attachTopic = validTopic(attachTopics);
    if (this->attachTopic.empty())
    {
        gzerr << "No valid attach topics for SuctionPlugin could be found.\n";
        return;
    }
    gzdbg << "Attach topic is: " << this->attachTopic << std::endl;

    // Setup subscriber for attach topic
    auto msgCb = std::function<void(const transport::ProtoMsg &)>(
            [this](const auto &)
            {
                if (this->isAttached){
                    gzdbg << "Already attached" << std::endl;
                    return;
                }
                this->attachRequested = true;
                gzerr << "Attach primed!" << std::endl;
            });

    if (!this->node.Subscribe(this->attachTopic, msgCb))
    {
        gzerr << "Subscriber could not be created for [attach] topic.\n";
        return;
    }

    // Setup output topic
    std::vector<std::string> outputTopics;
    if (_sdf->HasElement("output_topic"))
    {
        outputTopics.push_back(_sdf->Get<std::string>("output_topic"));
    } 
    this->outputTopic = validTopic(outputTopics);
    if (this->outputTopic.empty())
    {
        gzerr << "No valid output topics for SuctionPlugin could be found.\n";
        return;
    }
    gzdbg << "Output topic is: " << this->outputTopic << std::endl;

    // Setup publisher for output topic
    this->outputPub = this->node.Advertise<gz::msgs::StringMsg>(
            this->outputTopic);
    if (!this->outputPub)
    {
        gzerr << "Error advertising topic [" << this->outputTopic << "]"
                            << std::endl;
        return;
    }
}

//////////////////////////////////////////////////
void SuctionPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    GZ_PROFILE("SuctionPlugin::PreUpdate");
    // Finish up some loading
    // Setup potential connections
    if (!this->validConfig){
        std::vector<Entity> potentialEntities;
        _ecm.Each<components::Collision>(
                [&](const Entity &_entity, const components::Collision *) -> bool
                {
                    potentialEntities.push_back(_entity);
                    return true;
                });

        // Create a list of collision entities that have been marked as contact
        // sensors in this model. These are collisions that have a ContactSensorData
        // component
        auto allLinks =
                _ecm.ChildrenByComponents(this->model.Entity(), components::Link());

        for (const Entity linkEntity : allLinks)
        {
            auto linkCollisions =
                    _ecm.ChildrenByComponents(linkEntity, components::Collision());
            for (const Entity colEntity : linkCollisions)
            {
                if (_ecm.EntityHasComponentType(colEntity, components::ContactSensorData::typeId))
                {
                    this->collisionEntities.push_back(colEntity);
                }
            }
        }

        // Check if the component already exists, and create or update it
        if (!_ecm.Component<components::ExternalWorldWrenchCmd>(this->parentLinkEntity))
        {
            _ecm.CreateComponent(
                    this->parentLinkEntity,
                    components::ExternalWorldWrenchCmd());
            gzdbg << "Creating force" << std::endl;
        }
        this->validConfig = true;
    }

    // only allow attaching if child entity is detached
    if (!this->isAttached)
    {
        // Only attach when requested
        if (this->attachRequested){
            // Only attach when models are in contact
            if (this->touching){
                this->attachRequested = false;
                this->isAttached = true;
                this->PublishJointState(this->isAttached);
            }
        }
    }

    // Detach when detach is requested
    if (this->isAttached)
    {
        // 
        if (this->detachRequested)
        {
            // Detach the models
            this->detachRequested = false;
            this->isAttached = false;
            this->PublishJointState(this->isAttached);
        }
    }

    // If we are attached, exert a force
    if (this->isAttached)
    {
        gz::msgs::Vector3d force;
        force.set_x(this->touching_direction.x()*-this->suction_force);
        force.set_y(this->touching_direction.y()*-this->suction_force);
        force.set_z(this->touching_direction.z()*-this->suction_force);

        // Update the force and torque if the component already exists
        auto *wrenchCmd = _ecm.Component<components::ExternalWorldWrenchCmd>(this->parentLinkEntity);
        if (wrenchCmd)
        {
            wrenchCmd->Data().mutable_force()->CopyFrom(force);

            if ((std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime) - 
                this->lastMsgTime) > this->msgGap) 
            {
                gzdbg << "Force vector | x: " << force.x() 
                << " y: " << force.y()
                << " z: " << force.z() << std::endl;
                this->lastMsgTime = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime);
            }
        }
    }
}

//////////////////////////////////////////////////
void SuctionPlugin::PublishJointState(bool attached)
{
    msgs::StringMsg detachedStateMsg;
    if (attached)
    {
        detachedStateMsg.set_data("attached");
    }
    else
    {
        detachedStateMsg.set_data("detached");
    }
    this->outputPub.Publish(detachedStateMsg);
}

void SuctionPlugin::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
    this->touching = false;

    GZ_PROFILE("SuctionPlugin::PostUpdate");
    if (_info.paused)
        return;

    for (const Entity colEntity : this->collisionEntities)
    {
        auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);
        if (contacts)
        {
            
            // Check if the contacts include one of the target entities.
            for (const auto &contact : contacts->Data().contact())
            {
                // gzerr << "Contact 1: " << contact.collision1().name() << std::endl;
                // gzerr << "Contact 2: " << contact.collision2().name() << std::endl;
                // bool col1Target = std::binary_search(this->targetEntities.begin(),
                //         this->targetEntities.end(),
                //         contact.collision1().id());
                // bool col2Target = std::binary_search(this->targetEntities.begin(),
                //         this->targetEntities.end(),
                //         contact.collision2().id());
                // if (col1Target || col2Target)
                // {
                this->touching = true;
                this->touching_direction = **contact.normal().data();
                // }
                break;
            }
        }
    }
}

//////////////////////////////////////////////////
void SuctionPlugin::OnDetachRequest(const msgs::Empty &)
{
    gzerr << "Detach primed!" << std::endl;
    if (!this->isAttached){
        gzdbg << "Already detached" << std::endl;
        return;
    }
    this->detachRequested = true;
}

GZ_ADD_PLUGIN(SuctionPlugin,
    System,
    SuctionPlugin::ISystemConfigure,
    SuctionPlugin::ISystemPostUpdate,
    SuctionPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SuctionPlugin, "SuctionPlugin")