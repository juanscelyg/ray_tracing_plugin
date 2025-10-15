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
  
  if (!_sdf->HasElement("center_x")){
    gzmsg << "Missing <center_x> value. Its value will be set as '0.0'" << std::endl;
  }
  center_x_ = _sdf->Get<double>("center_x", 0.0).first;
  if (!_sdf->HasElement("center_y")){
    gzmsg << "Missing <center_y> value. Its value will be set as '0.0'" << std::endl;
  }
  center_y_ = _sdf->Get<double>("center_y", 0.0).first;
  if (!_sdf->HasElement("center_z")){
    gzmsg << "Missing <center_z> value. Its value will be set as '0.0'" << std::endl;
  }  
  center_z_ = _sdf->Get<double>("center_z", 0.0).first;
  if (!_sdf->HasElement("min_scan_x")){
    gzmsg << "Missing <min_scan_x> value. Its value will be set as '-10.0'" << std::endl;
  }
  min_scan_x_ = _sdf->Get<double>("min_scan_x", -10.0).first;
  if (!_sdf->HasElement("min_scan_y")){
    gzmsg << "Missing <min_scan_y> value. Its value will be set as '-10.0'" << std::endl;
  }
  min_scan_y_ = _sdf->Get<double>("min_scan_y", -10.0).first;
  if (!_sdf->HasElement("min_scan_z")){
    gzmsg << "Missing <min_scan_z> value. Its value will be set as '0.0'" << std::endl;
  }
  min_scan_z_ = _sdf->Get<double>("min_scan_z", 0.0).first;
  if (!_sdf->HasElement("max_scan_x")){
    gzmsg << "Missing <max_scan_x> value. Its value will be set as '10.0'" << std::endl;
  }
  max_scan_x_ = _sdf->Get<double>("max_scan_x", 10.0).first;
  if (!_sdf->HasElement("max_scan_y")){
    gzmsg << "Missing <max_scan_y> value. Its value will be set as '10.0'" << std::endl;
  }
  max_scan_y_ = _sdf->Get<double>("max_scan_y", 10.0).first;
  if (!_sdf->HasElement("max_scan_z")){
    gzmsg << "Missing <max_scan_z> value. Its value will be set as '10.0'" << std::endl;
  }
  max_scan_z_ = _sdf->Get<double>("max_scan_z", 10.0).first;
  if (!_sdf->HasElement("resolution")){
    gzmsg << "Missing <resolution> value. Its value will be set as '0.1'" << std::endl;
  }
  resolution_ = _sdf->Get<double>("resolution", 0.1).first;

  CreatePointCloud();
}

//////////////////////////////////////////////////////////////
void PointCloudGenerator::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
  _ecm.CreateComponent(worldEntity, gz::sim::components::PhysicsCollisionDetector("bullet"));

  rcEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(rcEntity, gz::sim::components::RaycastData());
  _ecm.CreateComponent(rcEntity, gz::sim::components::Pose(gz::math::Pose3d(0, 0, 10, 0, 0, 0)));

  auto &rays = _ecm.Component<gz::sim::components::RaycastData>(rcEntity)->Data().rays;

  if (this->resolution_ <= 0.0) this->resolution_ = 0.1;

  // for (double x = min_scan_x_; x <= max_scan_x_; x += resolution_)
  // {
  //   for (double y = min_scan_y_; y <= max_scan_y_; y += resolution_)
  //   {
  //     gz::sim::components::RayInfo r;
  //     r.start = gz::math::Vector3d(center_x_ + x, center_y_ + y, center_z_ + 5.0);
  //     r.end   = gz::math::Vector3d(center_x_ + x, center_y_ + y, center_z_ - 20.0);
  //     rays.push_back(r);
  //   }
  // }
  for (int i = 0; i < 5; ++i)
  {
    gz::sim::components::RayInfo ray;
    ray.start = gz::math::Vector3d(0, 0, -i);
    ray.end = gz::math::Vector3d(0, 0, -20);
    rays.push_back(ray);
  }
}

//////////////////////////////////////////////////////////////////////////////
void PointCloudGenerator::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  auto &rays = _ecm.Component<gz::sim::components::RaycastData>(rcEntity)->Data().rays;
  auto &results = _ecm.Component<gz::sim::components::RaycastData>(rcEntity)->Data().results;
  std::cout << "Rays: " << rays.size() << std::endl;
  std::cout << "Results: " << results.size() << std::endl;
  for (size_t i = 0; i < results.size(); ++i) {
  const double expFraction =
    (rays[i].start - results[i].point).Length() /
      (rays[i].start - rays[i].end).Length();
      std::cout << results[i].point << std::endl;
  }
}

void PointCloudGenerator::CreatePointCloud()
{

}

}  // namespace ray_tracing_plugin

GZ_ADD_PLUGIN(
    ray_tracing_plugin::PointCloudGenerator,
    gz::sim::System,
    ray_tracing_plugin::PointCloudGenerator::ISystemConfigure,
    ray_tracing_plugin::PointCloudGenerator::ISystemPreUpdate,
    ray_tracing_plugin::PointCloudGenerator::ISystemPostUpdate)