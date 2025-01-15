/*
cd /home/skemp32/ros2_ws/src/ardupilot_gazebo/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
*/
/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef SUCTIONPLUGIN_HH_
#define SUCTIONPLUGIN_HH_

#include <gz/msgs/empty.pb.h>

#include <memory>
#include <string>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system that initially attaches two models via a fixed joint and
  /// allows for the models to get detached during simulation via a topic. A
  /// model can be re-attached during simulation via a topic. The status of the
  /// detached state can be monitored via a topic as well.
  ///
  /// ## System Parameters
  ///
  /// - `<parent_link>`: Name of the link in the parent model to be used in
  /// creating a fixed joint with a link in the child model.
  ///
  /// - `<child_model>`: Name of the model to which this model will be connected
  ///
  /// - `<child_link>`: Name of the link in the child model to be used in
  /// creating a fixed joint with a link in the parent model.
  ///
  /// - `<topic>` (optional): Topic name to be used for detaching connections.
  /// Using <detach_topic> is preferred.
  ///
  /// - `<detach_topic>` (optional): Topic name to be used for detaching
  /// connections. If multiple detachable plugin is used in one model,
  /// `detach_topic` is REQUIRED to detach child models individually.
  ///
  /// - `<attach_topic>` (optional): Topic name to be used for attaching
  /// connections. If multiple detachable plugin is used in one model,
  /// `attach_topic` is REQUIRED to attach child models individually.
  ///
  /// - `<output_topic>` (optional): Topic name to be used for publishing
  /// the state of the detachment. If multiple detachable plugin is used in
  /// one model, `output_topic` is REQUIRED to publish child models state
  /// individually.
  ///
  /// - `<suppress_child_warning>` (optional): If true, the system
  /// will not print a warning message if a child model does not exist yet.
  /// Otherwise, a warning message is printed. Defaults to false.

class SuctionHandler
{
public:
    SuctionHandler(Entity _parentLink, double _suctionForce, double _suctionDelay, double _detachDelay)
        : parentLink(_parentLink), suctionForce(_suctionForce), suctionDelay(_suctionDelay), detachDelay(_detachDelay), hasSuction(false),
          isAttached(false), detachRequested(false), isTouching(false), contactStartTime(0), detachRequestTime(0), msgPeriod_s(INFINITY), lastMsgTime(0),
          noAttach(false), noDetach(false) {}

    void Initialize(EntityComponentManager &_ecm)
    {
        std::vector<Entity> potentialEntities;
        _ecm.Each<components::Collision>(
        [&](const Entity &_entity, const components::Collision *) -> bool
        {
            potentialEntities.push_back(_entity);
            return true;
        });

        auto linkCollisions = _ecm.ChildrenByComponents(this->parentLink, components::Collision());
        for (const Entity colEntity : linkCollisions)
        {
            if (_ecm.EntityHasComponentType(colEntity, components::ContactSensorData::typeId))
            {
                this->collisionEntities.push_back(colEntity);
                gzdbg << "Found collision entity." << std::endl;
            }
        }

        if (!_ecm.Component<components::ExternalWorldWrenchCmd>(this->parentLink))
        {
            _ecm.CreateComponent(this->parentLink, components::ExternalWorldWrenchCmd());
            gzdbg << "Creating force component for parent link entity." << std::endl;
        }
    }

    void UpdateTouchingState(const EntityComponentManager &_ecm)
    {
        this->isTouching = false;
        for (const auto &colEntity : this->collisionEntities)
        {
            auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);
            if (contacts)
            {
                for (const auto &contact : contacts->Data().contact())
                {

                    this->isTouching = true;
                    this->touchingDirection = **contact.normal().data();
                }
            }
        }
        
    }

    void Attach(const std::chrono::duration<double> &simTime)
    {
        if (!this->isAttached && this->isTouching)
        {
            this->isAttached = true;
            this->contactStartTime = simTime;
            gzdbg << "Attachment successful at time: " << simTime.count() << std::endl;
        }
    }

    void Detach()
    {
        if (this->isAttached)
        {
            gzdbg << "Detach started" << std::endl;
            this->detachRequested = true;
            // auto future = std::async(std::launch::async, [this]() {
            //     while _info.simTime
            //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(this->detachDelay * 1000)));
            //     this->isAttached = false;
            //     gzdbg << "Detach successful after delay of " << this->detachDelay << " seconds." << std::endl;
            // });
        }
    }

    void ApplyForce(EntityComponentManager &_ecm, const std::chrono::duration<double> &simTime)
    {
        this->hasSuction = false;
        if (this->detachRequested) {
            if ((simTime - this->detachRequestTime).count() > this->detachDelay) {
                this->isAttached = false;
                this->detachRequested = false;
            }
            else if ((simTime - this->contactStartTime).count() <= this->suctionDelay || !this->isTouching) { //skip delay if already off blade or attach hasn't started yet
                this->isAttached = false;
                this->detachRequested = false;
            }
        } else {
            this->detachRequestTime = simTime;
        }
        if (
            (
                this->isAttached 
                && (simTime - this->contactStartTime).count() > this->suctionDelay
                && !(this->noAttach)
                // && this->isTouching
            )
            ||
            (
                this->isTouching && this->noDetach
            )
        )
        {
            this->hasSuction = true;
            auto *wrenchCmd = _ecm.Component<components::ExternalWorldWrenchCmd>(this->parentLink);
            if (wrenchCmd)
            {
                gz::msgs::Vector3d force;
                force.set_x(this->touchingDirection.x() * -this->suctionForce);
                force.set_y(this->touchingDirection.y() * -this->suctionForce);
                force.set_z(this->touchingDirection.z() * -this->suctionForce);
                wrenchCmd->Data().mutable_force()->CopyFrom(force);

                if ((simTime - this->lastMsgTime).count() > this->msgPeriod_s || (this->lastMsgTime.count() == 0)){
                    gzdbg << "Applying force: ["
                      << force.x() << ", "
                      << force.y() << ", "
                      << force.z() << "]" << std::endl;
                      this->lastMsgTime = simTime;
                } 
            }
        }
    }

    bool IsAttached() const { return this->isAttached; }
    bool HasSuction() const { return this->hasSuction; }
    bool IsTouching() const { return this->isTouching; }
    void disableAttach() { this->noAttach = true; }
    void disableDetach() { this->noDetach = true; }
    void clearFailures() {
        this->noAttach = false;
        this->noDetach = false;
    }

private:
    Entity parentLink;
    double suctionForce;
    double suctionDelay;
    double detachDelay;
    bool isAttached;
    bool hasSuction;
    bool detachRequested;
    bool isTouching;
    bool noAttach;
    bool noDetach;
    gz::msgs::Vector3d  touchingDirection;
    std::chrono::duration<double> contactStartTime;
    std::chrono::duration<double> detachRequestTime;
    std::chrono::duration<double> lastMsgTime;
    double msgPeriod_s;
    std::vector<Entity> collisionEntities;
};

  class SuctionPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// Documentation inherited
    public: SuctionPlugin() = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Gazebo communication node.
    private: transport::Node node;

    /// \brief A publisher to send state of the detachment
    private: transport::Node::Publisher outputPub;

    /// \brief A publisher to send command to the leg
    private: transport::Node::Publisher legPub;

    /// \brief Helper function to publish the state of the detachment
    private: void PublishJointState(bool attached);

    /// \brief Helper function to publish the command to the leg
    private: void PublishLegCommand(double cmd);

    /// \brief Callback for detach request topic
    private: void OnDetachRequest(const msgs::Empty &_msg);

    /// \brief The model associated with this system.
    private: Model model;

    /// \brief Topics for attach, detach, leg, and failure topics
    private: std::string detachTopic;
    private: std::string attachTopic;
    private: std::string outputTopic;
    private: std::string legTopic;
    private: std::string failureTopic;

    /// \brief Entity of attachment link in the parent model
    private: Entity parentLinkEntity{kNullEntity};

    /// \brief Entity of attachment link in the parent model
    private: Entity parentLinkEntity2{kNullEntity};

    /// \brief Whether detachment has been requested
    private: std::atomic<bool> detachRequested{false};

    /// \brief Whether attachment has been requested
    private: std::atomic<bool> attachRequested{false};

    /// \brief Whether all parameters are valid and the system can proceed
    private: bool validConfig{false};

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private: std::string findNumNextToKeyword(std::string &receivedMessage, const std::string &keyword);

    private: bool use_leg{false};

    private: double leg_extended_pos{0};

    private: double leg_retracted_pos{0};

    private: std::vector<SuctionHandler>suctionHandlers;

    private: bool _allAttached{false};

    private: bool _disattached{false};

    private: bool _disdetached{false};

    const std::string no_attach_keyword = "noattach";
    const std::string no_detach_keyword = "nodetach";
    const std::string disattached_keyword = "disattached";
    const std::string disdetached_keyword = "disdetached";
    
    // const std::string clear_faults_keyword = "clear";
    };
  }
}
}
}




#endif