#ifndef RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_
#define RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ray_tracing_plugin
{
  class PointCloudGenerator:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
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
      double min_height_;
      double max_height_;
      double resolution_;

    private:
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ray_tracing_plugin");
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  };
}  // namespace ray_tracing_plugin

#endif  // RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_