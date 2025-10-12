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