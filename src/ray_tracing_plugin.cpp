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

  entity = _entity; // Si el plugin está bajo <world>, esto es el "world entity" (marco mundo).
  // Asegura que el componente RaycastData exista en este entity.
  if (!_ecm.EntityHasComponentType(entity,
        gz::sim::components::RaycastData::typeId))
  {
    std::cout << "Traza si el rayo no existe" << std::endl;
    _ecm.CreateComponent(entity, gz::sim::components::RaycastData());
  }

  if (!_ecm.EntityHasComponentType(entity,
        gz::sim::components::World::typeId))
  {
    std::cout << "Traza si el mundo no existe" << std::endl;
    _ecm.CreateComponent(entity, gz::sim::components::World());
  }
  
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

  std::cout << "PointCloudGenerator Configurado. start=" << start
                << " end=" << end << std::endl;

  CreatePointCloud();
}

void PointCloudGenerator::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  auto rc = _ecm.Component<gz::sim::components::RaycastData>(entity);
  if (!rc)
    return;

  gz::sim::components::RaycastDataInfo data;
  gz::sim::components::RayInfo r;
  r.start = start;   // En marco del entity (mundo si el entity es el world)
  r.end   = end;     // En marco del entity (mundo si el entity es el world)

  data.rays.clear();
  data.results.clear();    // importante: limpiamos resultados anteriores
  data.rays.push_back(r);

  rc->Data() = std::move(data);
  //std::cout << rc->Data().rays[0].start << std::endl;
}

void PointCloudGenerator::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "PointCloudGenerator::PostUpdate" << std::endl;

  const auto rc = _ecm.Component<gz::sim::components::RaycastData>(entity);
  if (!rc) return;

  //std::cout << rc->Data().results.size() << std::endl;

  const auto &data = rc->Data();
  if (!data.results.empty())
  {
    const auto &res = data.results.front();
    std::cout << "fraction=" << res.fraction
              << " point=" << res.point
              << " normal=" << res.normal << std::endl;
  }
  else
  {
    //std::cout << "Sin impacto (resultados vacíos)\n";
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