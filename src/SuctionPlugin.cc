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

    int index = 1;  // Start with 1 for parent_link1
    while (true)
    {
        // Construct tag names dynamically
        std::string parentLinkTag = "parent_link" + std::to_string(index);
        std::string suctionForceTag = "suction_force" + std::to_string(index);
        std::string suctionDelayTag = "suction_delay" + std::to_string(index);
        std::string detachDelayTag = "detach_delay" + std::to_string(index);

        // Check if the parent_link element exists
        if (!_sdf->HasElement(parentLinkTag))
        {
            // If the element doesn't exist, break the loop (stop parsing)
            break;
        }

        // Get the parent link name
        std::string parentLinkName = _sdf->Get<std::string>(parentLinkTag);

        // Resolve the parent link entity
        Entity parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
        if (kNullEntity == parentLinkEntity)
        {
            gzerr << "Link with name [" << parentLinkName
                << "] not found in model [" << this->model.Name(_ecm)
                << "]. Ensure 'parent_link' has the correct value.\n";
            return;
        }

        // Check if the suction_force element exists
        if (!_sdf->HasElement(suctionForceTag))
        {
            gzerr << "Missing <" << suctionForceTag << "> for parent link [" << parentLinkName
                << "]. Aborting initialization.\n";
            return;
        }

        // Check if the suction_force element exists
        if (!_sdf->HasElement(suctionDelayTag))
        {
            gzerr << "Missing <" << suctionDelayTag << "> for parent link [" << parentLinkName
                << "]. Aborting initialization.\n";
            return;
        }

        // Check if the suction_force element exists
        if (!_sdf->HasElement(detachDelayTag))
        {
            gzerr << "Missing <" << detachDelayTag << "> for parent link [" << parentLinkName
                << "]. Aborting initialization.\n";
            return;
        }

        // Get the suction force value
        double suctionForce = _sdf->Get<double>(suctionForceTag);
        double suctionDelay = _sdf->Get<double>(suctionDelayTag);
        double detachDelay = _sdf->Get<double>(detachDelayTag);

        // Add this handler to the list
        this->suctionHandlers.emplace_back(parentLinkEntity, suctionForce, suctionDelay, detachDelay);
        gzdbg << "Created SuctionHandler for link [" << parentLinkName
            << "] with force [" << suctionForce << "].\n";

        // Increment index to process the next set of elements
        index++;
    }

    // Setup detach topic
    std::vector<std::string> detachTopics;
    if (_sdf->HasElement("detach_topic"))
    {
        detachTopics.push_back(_sdf->Get<std::string>("detach_topic"));
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
                if (this->attachRequested){
                    gzdbg << "Attach already primed!" << std::endl;
                    return;
                }
                this->attachRequested = true;
                this->PublishLegCommand(this->leg_extended_pos); // publish leg position command
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

    // Setup leg topic
    std::vector<std::string> legTopics;
    if (_sdf->HasElement("leg_topic"))
    {
        legTopics.push_back(_sdf->Get<std::string>("leg_topic"));
        if (_sdf->HasElement("leg_retracted_pos")) {
            this->leg_retracted_pos = _sdf->Get<std::double_t>("leg_retracted_pos");
            this->use_leg = true;
        }
        else {
            gzerr << "Need leg_retracted_pos field when using leg_topic" << std::endl;
            return;
        }
        if (_sdf->HasElement("leg_extended_pos")) {
            this->leg_extended_pos = _sdf->Get<std::double_t>("leg_extended_pos");
            this->use_leg = true;
        }
        else {
            gzerr << "Need leg_extended_pos field when using leg_topic" << std::endl;
            return;
        }
    } 
    this->legTopic = validTopic(legTopics);
    if (this->legTopic.empty())
    {
        gzwarn << "No valid leg topics could be found. Not using leg publisher\n";
    }
    else {
        gzdbg << "Leg topic is: " << this->legTopic << std::endl;

        // Setup publisher for leg topic
        this->legPub = this->node.Advertise<gz::msgs::Double>(
                this->legTopic);
        if (!this->legPub)
        {
            gzerr << "Error advertising topic [" << this->legTopic << "]" << std::endl;
            return;
        }
    }

    // Setup failure topic
    std::vector<std::string> failureTopics;
    if (_sdf->HasElement("failure_topic"))
    {
        failureTopics.push_back(_sdf->Get<std::string>("failure_topic"));
    }

    this->failureTopic = validTopic(failureTopics);
    if (this->failureTopic.empty())
    {
        gzerr << "No valid failure topics for SuctionPlugin could be found.\n";
        return;
    }
    gzdbg << "failure topic is: " << this->failureTopic << std::endl;

    // Setup subscriber for failure topic
    // Setup subscriber for attach topic
    auto failureMsgCb = std::function<void(const transport::ProtoMsg &)>(
        [this](const transport::ProtoMsg &msg)
        {
            // Assuming ProtoMsg has a method `Data()` returning the string payload
            std::string receivedMessage;
            if (!msg.SerializeToString(&receivedMessage)) {
                gzerr << "Failure topic deserialization failed" << std::endl;
            }

            size_t pos;

            // check for disattached or disdetached (TODO)
            _disattached = (findNumNextToKeyword(receivedMessage, this->disattached_keyword).compare("0")==0);
            _disdetached = (findNumNextToKeyword(receivedMessage, this->disdetached_keyword).compare("0")==0);

            if (_disattached) {
                gzdbg << "Attached signal disabled" << std::endl;
            }
            if (_disdetached) {
                gzdbg << "Detached signal disabled" << std::endl;
            }
            

            // Check for "noattach" keyword
            std::string num_noattach = findNumNextToKeyword(receivedMessage, this->no_attach_keyword);
            std::string num_nodetach = findNumNextToKeyword(receivedMessage, this->no_detach_keyword);
            // std::string num_clear = findNumNextToKeyword(receivedMessage, this->clear_faults_keyword);

            uint8_t index = 0;
            for (auto &handler : this->suctionHandlers) {
                uint8_t handler_idx = index+1;
                uint8_t fault_count = 0;
                handler.clearFailures(); // clear all failures when message arrives
                if (num_noattach.find(std::to_string(handler_idx)) != std::string::npos) { 
                    handler.disableAttach(); 
                    gzdbg << "Cup " << std::to_string(handler_idx) << " suction disabled" << std::endl;
                    fault_count++;
                }
                if (num_nodetach.find(std::to_string(handler_idx)) != std::string::npos) { 
                    handler.disableDetach(); 
                    gzdbg << "Cup " << std::to_string(handler_idx) << " suction stuck on" << std::endl;
                    fault_count++;
                }
                if (fault_count == 0) {
                    gzdbg << "Cup " << std::to_string(handler_idx) << " faults cleared" << std::endl;
                }
                index++;
            }
        });

    if (!this->node.Subscribe(this->failureTopic, failureMsgCb))
    {
        gzerr << "Subscriber could not be created for failure topic.\n";
        return;
    }

    gzdbg << "SuctionPlugin subscribing to messages on "
                 << "[" << this->failureTopic << "]" << std::endl;
    
}

std::string SuctionPlugin::findNumNextToKeyword(std::string &receivedMessage, const std::string &keyword) {
    size_t pos = receivedMessage.find(keyword); // Find keyword
    if (pos != std::string::npos)
    {
        size_t numStart = pos + keyword.length(); // Position after the keyword
        std::string number = "0"; // To store the digits

        // Extract digits starting from numStart
        while (numStart < receivedMessage.size() && std::isdigit(receivedMessage[numStart]))
        {
            number += receivedMessage[numStart];
            ++numStart; // Move to the next character
        }

        return number; // Return the extracted digits
    }
    return ""; // Return empty string if keyword is not found
}

//////////////////////////////////////////////////
void SuctionPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    GZ_PROFILE("SuctionPlugin::PreUpdate");

    // Ensure all handlers are initialized
    if (!this->validConfig)
    {
        for (auto &handler : this->suctionHandlers)
        {
            handler.Initialize(_ecm);
        }
        this->validConfig = true;
    }

    // Update attach state for each handler. Need thgis in PreUpdate for access to _info which isnt available in a callback
    if (this->attachRequested){ 
        for (auto &handler : this->suctionHandlers){
            handler.Attach(_info.simTime);
        }
    }

    // Apply force for attached handlers
    for (auto &handler : this->suctionHandlers)
    {
        if (handler.IsAttached())
        {
            handler.ApplyForce(_ecm, _info.simTime);
        }
    }

    // Publish state of handlers
    bool allAttached = true;
    for (auto &handler : this->suctionHandlers)
    {
        if (!handler.HasSuction()) {allAttached = false;}
    }
    if (allAttached != _allAttached) {
        PublishJointState(allAttached);
        _allAttached = allAttached;
    }

}

//////////////////////////////////////////////////
void SuctionPlugin::PublishJointState(bool attached)
{
    msgs::StringMsg detachedStateMsg;
    if (attached && !_disattached)
    {
        detachedStateMsg.set_data("attached");
        gzdbg << "Publishing attached" << std::endl;
    }
    else if (!attached && !_disdetached) 
    {
        detachedStateMsg.set_data("detached");
        gzdbg << "Publishing detached" << std::endl;
    }
    else
    {
        gzdbg << "Publishing disabled for " << (attached ? "attached" : "detached") << std::endl;
        return;
    }
    this->outputPub.Publish(detachedStateMsg);
}

void SuctionPlugin::PublishLegCommand(double cmd)
{
    if (!this->use_leg) {return;}
    gz::msgs::Double legCommandMsg;

    legCommandMsg.set_data(cmd);
    this->legPub.Publish(legCommandMsg);
}

void SuctionPlugin::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
    GZ_PROFILE("SuctionPlugin::PostUpdate");
    if (_info.paused)
        return;

    for (auto &handler : this->suctionHandlers)
    {
        handler.UpdateTouchingState(_ecm);
    }
}

//////////////////////////////////////////////////
void SuctionPlugin::OnDetachRequest(const msgs::Empty &)
{
    this->attachRequested = false;
    for (auto &handler : this->suctionHandlers)
    {
        handler.Detach();
    }
    this->PublishLegCommand(this->leg_retracted_pos); // publish leg position command
    gzdbg << "All suction handlers detached." << std::endl;
}


GZ_ADD_PLUGIN(SuctionPlugin,
    System,
    SuctionPlugin::ISystemConfigure,
    SuctionPlugin::ISystemPostUpdate,
    SuctionPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SuctionPlugin, "SuctionPlugin")