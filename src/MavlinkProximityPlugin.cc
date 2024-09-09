
#include "MavlinkProximityPlugin.hh"

#include<iostream>
#include<string>
#include<vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>

#include <mavsdk/mavsdk.h>
#include <mavsdk/mavlink/common/mavlink.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


namespace gz{
namespace sim{
namespace systems{

class MavlinkProximityPlugin::Impl {
   public:

    void onData(const msgs::LaserScan &msg);

    std::string tcpHost{"127.0.0.1"};
    int tcpPort{5762};
    std::string lidarTopic;
    std::string enableTopic;

    // Unused by actual pipeline since it's based on the gazebo topic rate?
    uint64_t sim_time;


    bool is_initialised{false};
    Sensor parentSensor;
    std::string sensorName;
    std::vector<common::ConnectionPtr> connections;
    transport::Node node;

    mavsdk::Mavsdk mavlink{mavsdk::Mavsdk::Configuration{1,MAV_COMP_ID_PERIPHERAL,false}};
    std::shared_ptr<mavsdk::MavlinkPassthrough> mav_stream;
    bool mavlink_connected{false};
    bool mav_found{false};
    int hush_timer{0};

};



void MavlinkProximityPlugin::Configure(const Entity &_entity,
                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                      EntityComponentManager &_ecm,
                                      EventManager &)
{

    impl->parentSensor = Sensor(_entity);

    if (!impl->parentSensor.Valid(_ecm))
    {
        gzerr << "ProximityPlugin: must be attached to a lidar sensor. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (auto maybeName = impl->parentSensor.Name(_ecm))
    {
        gzmsg << "ProximityPlugin: attached to sensor ["
              << maybeName.value() << "]" << std::endl;
    }
    else
    {
        gzerr << "ProximityPlugin: lidar sensor has invalid name. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (_sdf->HasElement("tcp_host"))
    {
        impl->tcpHost = _sdf->Get<std::string>("tcp_host");
    }

    if (_sdf->HasElement("tcp_port"))
    {
        impl->tcpPort = _sdf->Get<int>("tcp_port");
    }

    gzmsg << "ProximityPlugin: streaming video to "
          << impl->tcpHost << ":"
          << impl->tcpPort << std::endl;

}

 void MavlinkProximityPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
 {

    if (impl->sensorName.empty())
    {
        Entity sensorEntity = impl->parentSensor.Entity();
        impl->sensorName = removeParentScope(
            scopedName(sensorEntity, _ecm, "::", false), "::");
        gzmsg << "ProximityPlugin: sensor name ["
              << impl->sensorName << "]" << std::endl;
    }

    // complete initialisation deferred from Configure()
    if (!impl->is_initialised)
    {
        if (impl->lidarTopic.empty())
        {
            auto maybeTopic = impl->parentSensor.Topic(_ecm);
            if (!maybeTopic.has_value())
            {
                return;
            }
            impl->lidarTopic = maybeTopic.value();
        }
        
        // Start Mavlink
        if(impl->hush_timer++ >1000)
        {
            impl->hush_timer=0;
            mavsdk::ConnectionResult connection_result = impl->mavlink.add_tcp_connection(
                impl->tcpHost,
                impl->tcpPort,
                mavsdk::ForwardingOption::ForwardingOff
            );

            if(connection_result != mavsdk::ConnectionResult::Success){
                
                gzwarn<< "[ProximityPlugin]: Mavlink Connection Failed, tcp port not open" << std::endl;

                return;
            }
            impl->mavlink_connected = true;
        }

        if(impl->mavlink_connected){
            gzmsg << "[ProximityPlugin]: topic [" << impl->lidarTopic << "]" << std::endl;

            // subscribe to gazebo topics
            impl->node.Subscribe(impl->lidarTopic,
                &MavlinkProximityPlugin::Impl::onData, impl.get());


            impl->is_initialised = true; 
        }
        

    }

    impl->sim_time = _info.simTime.count()/1000;

}

void MavlinkProximityPlugin::Impl::onData(const msgs::LaserScan &msg)
{
    
    if(!mav_found){
        auto system = mavlink.first_autopilot(0.0);
        if (!system) {
            gzdbg << "[ProximityPlugin] waiting for autopilot" << std::endl;
            return;
        }
        mav_stream = std::make_shared<mavsdk::MavlinkPassthrough>(system.value());
        gzmsg << "[ProximityPlugin] connected to autopilot" << std::endl;
        mav_found = true;
    }
    // Extract Message   
    int N_pts = msg.ranges_size();
    uint16_t min_range = uint16_t(msg.range_min()*100.0);
    uint16_t max_range = uint16_t(msg.range_max()*100.0);
    float angle_step = float(msg.angle_step()*57.2957795131);

    std::vector<uint16_t> distances;
    distances.reserve(N_pts+72);

    for(int i = 0; i<N_pts; i++){
        distances.push_back(uint16_t(msg.ranges(i) * 100.0));
    }
    for(int i = 0; i<72; i++){
        distances.push_back(UINT16_MAX);
    }


    int index = 0;
    while(index < N_pts){
        float angle_start = float((msg.angle_min()*(N_pts - index) + msg.angle_max()*index)*57.2957795131)/float(N_pts);
        mav_stream->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_obstacle_distance_pack(
                mavlink_address.system_id,
                mavlink_address.component_id,
                &message,
                this->sim_time,
                MAV_DISTANCE_SENSOR_LASER, // Distance sensor type
                &distances[index],
                0,
                min_range,
                max_range,
                angle_step,
                angle_start,
                MAV_FRAME_BODY_FRD); // frame of reference
            return message;
        });
        index += 72;
    }
}


MavlinkProximityPlugin::MavlinkProximityPlugin()
    : impl(std::make_unique<MavlinkProximityPlugin::Impl>())
{
}


}
}
}

GZ_ADD_PLUGIN(
    gz::sim::systems::MavlinkProximityPlugin,
    gz::sim::System,
    gz::sim::systems::MavlinkProximityPlugin::ISystemConfigure,
    gz::sim::systems::MavlinkProximityPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::MavlinkProximityPlugin,
    "MavlinkProximityPlugin")