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

    if (_sdf->HasElement("child_model"))
    {
        this->childModelName = _sdf->Get<std::string>("child_model");
    }
    else
    {
        gzerr << "'child_model' is a required parameter for SuctionPlugin."
                            "Failed to initialize.\n";
        return;
    }

    if (_sdf->HasElement("child_link"))
    {
        this->childLinkName = _sdf->Get<std::string>("child_link");
    }
    else
    {
        gzerr << "'child_link' is a required parameter for SuctionPlugin."
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

    outputTopics.push_back("/model/" + this->childModelName +
            "/detachable_joint/state");

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

    // Supress Child Warning
    this->suppressChildWarning =
            _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
                    .first;

    this->targetName = this->childModelName;
}

void SuctionPlugin::AddTargetEntities(const EntityComponentManager &_ecm, const std::vector<Entity> &_entities)
{
    if (_entities.empty())
        return;

    for (Entity entity : _entities)
    {
        // The target name can be a substring of the desired collision name so we
        // have to iterate through all collisions and check if their scoped name has
        // this substring
        std::string name = scopedName(entity, _ecm);
        if (name.find(this->targetName) != std::string::npos)
        {
            this->targetEntities.push_back(entity);
        }
    }

    // Sort so that we can do binary search later on.
    std::sort(this->targetEntities.begin(), this->targetEntities.end());
}

//////////////////////////////////////////////////
void SuctionPlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
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

        this->AddTargetEntities(_ecm, potentialEntities);

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

        this->validConfig = true;
    }

    // only allow attaching if child entity is detached
    if (!this->isAttached)
    {
        // return if attach is not requested.
        if (!this->attachRequested){
            return;
        }
        // return if not in contact
        if (!this->touching){
            // gzerr << "Not in contact!" << std::endl;
            return;
        }
        // Look for the child model and link
        Entity modelEntity{kNullEntity};

        if ("__model__" == this->childModelName)
        {
            modelEntity = this->model.Entity();
        }
        else
        {
            modelEntity = _ecm.EntityByComponents(
                    components::Model(), components::Name(this->childModelName));
        }
        if (kNullEntity != modelEntity)
        {
            this->childLinkEntity = _ecm.EntityByComponents(
                    components::Link(), components::ParentEntity(modelEntity),
                    components::Name(this->childLinkName));

            if (kNullEntity != this->childLinkEntity)
            {
                // Attach the models
                // We do this by creating a detachable joint entity.
                this->detachableJointEntity = _ecm.CreateEntity();

                _ecm.CreateComponent(
                        this->detachableJointEntity,
                        components::DetachableJoint({this->parentLinkEntity,
                                                                                 this->childLinkEntity, "fixed"}));
                this->attachRequested = false;
                this->isAttached = true;
                this->PublishJointState(this->isAttached);
                gzdbg << "Attaching entity: " << this->detachableJointEntity
                             << std::endl;
            }
            else
            {
                gzwarn << "Child Link " << this->childLinkName
                                << " could not be found.\n";
            }
        }
        else if (!this->suppressChildWarning)
        {
            gzwarn << "Child Model " << this->childModelName
                            << " could not be found.\n";
        }
    }

 // only allow detaching if child entity is attached
    if (this->isAttached)
    {
        if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
        {
            // Detach the models
            gzdbg << "Removing entity: " << this->detachableJointEntity << std::endl;
            _ecm.RequestRemoveEntity(this->detachableJointEntity);
            this->detachableJointEntity = kNullEntity;
            this->detachRequested = false;
            this->isAttached = false;
            this->PublishJointState(this->isAttached);
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
                bool col1Target = std::binary_search(this->targetEntities.begin(),
                        this->targetEntities.end(),
                        contact.collision1().id());
                bool col2Target = std::binary_search(this->targetEntities.begin(),
                        this->targetEntities.end(),
                        contact.collision2().id());
                if (col1Target || col2Target)
                {
                    this->touching = true;
                }
            }
        }
    }
}

//////////////////////////////////////////////////
void SuctionPlugin::OnDetachRequest(const msgs::Empty &)
{
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