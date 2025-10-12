#ifndef RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_
#define RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

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
  };
}  // namespace ray_tracing_plugin

#endif  // RAY_TRACING_PLUGIN__RAY_TRACING_PLUGIN_HPP_