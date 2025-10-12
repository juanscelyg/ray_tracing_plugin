#include <ray_tracing_plugin/ray_tracing_plugin.hpp>

namespace ray_tracing_plugin
{

PointCloudGenerator::PointCloudGenerator()
{
}
 
PointCloudGenerator::~PointCloudGenerator()
{
}

void PointCloudGenerator::Configure(
          const gz::sim::Entity &_entity,
          const std::shared_ptr<const sdf::Element> &_sdf,
          gz::sim::EntityComponentManager &_ecm,
          gz::sim::EventManager &/*_eventMgr*/)
          {
            gzmsg << "PointCloudGenerator::Configure" << std::endl;

            RCLCPP_INFO(node->get_logger(), "Node Initialized");

            pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
                              node->get_name() + std::string("/") + "pointcloud",
                              rclcpp::QoS(1).transient_local().reliable());
          }

void PointCloudGenerator::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "PointCloudGenerator::PostUpdate" << std::endl;
}

}  // namespace ray_tracing_plugin

GZ_ADD_PLUGIN(
    ray_tracing_plugin::PointCloudGenerator,
    gz::sim::System,
    ray_tracing_plugin::PointCloudGenerator::ISystemConfigure,
    ray_tracing_plugin::PointCloudGenerator::ISystemPostUpdate)