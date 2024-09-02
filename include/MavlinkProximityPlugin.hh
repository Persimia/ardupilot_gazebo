
#ifndef MAVLINKPROXIMITYPLUGIN_HH_
#define MAVLINKPROXIMITYPLUGIN_HH_

#include <array>
#include <memory>

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace gz {
namespace sim {
namespace systems {

/// \brief Plugin to stream lidar sensor data using Mavlink.
/// \class ProximityPlugin
///



class MavlinkProximityPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{

    /// \brief Constructor
    public: MavlinkProximityPlugin();

    public: void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm);

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &);


    /// \internal
    /// \brief Private implementation
    private: class Impl;
    private: std::unique_ptr<Impl> impl;

};

}
}
}
#endif //MAVLINKPROXIMITYPLUGIN_HH_