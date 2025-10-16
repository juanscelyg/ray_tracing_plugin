#ifndef RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_
#define RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_

#include <memory>
#include <string>
#include <utility>

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/sim/components/RaycastData.hh> 
#include <gz/sim/components/World.hh> 

#include <gz/physics/RequestFeatures.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/World.hh>
#include <gz/physics/detail/GetRayIntersection.hh>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>

#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

namespace ray_tracing_plugin
{
  class PointCloudGenerator:
      public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
  {
    public: 
    
      PointCloudGenerator();
      
      ~PointCloudGenerator() override;

      void Configure(
            const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &/*_eventMgr*/) override;

      void PreUpdate(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;
      
      void PostUpdate(const gz::sim::UpdateInfo &_info,
            const gz::sim::EntityComponentManager &_ecm) override;

      double center_x_;
      double center_y_;
      double center_z_;
      double min_scan_x_;
      double min_scan_y_;
      double min_scan_z_;
      double max_scan_x_;
      double max_scan_y_;
      double max_scan_z_;
      double resolution_;

    private:

      gz::sim::Entity rcEntity_;
      //gz::physics::GetRayIntersectionFromLastStepFeature::World<gz::physics::FeaturePolicy3d,
      //    gz::physics::FeatureList<gz::physics::GetRayIntersectionFromLastStepFeature>> physicsWorld_;
      gz::transport::Node node_;
      gz::transport::Node::Publisher pub_;
      gz::msgs::PointCloudPacked cloud_;

      std::string topic_{"/ray_tracing_cloud"};
      std::string frame_id_{"/world"};

      bool GeneratedCloud_ {false};
      int iteration_ {0};

  };
}  // namespace ray_tracing_plugin

#endif  // RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_