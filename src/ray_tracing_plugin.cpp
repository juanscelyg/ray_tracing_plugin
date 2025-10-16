#include <ray_tracing_plugin/ray_tracing_plugin.hpp>

namespace ray_tracing_plugin
{

  PointCloudGenerator::PointCloudGenerator()
  {
    std::cout << "Creating PointCloudGenerator Plugin" << std::endl;
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
    std::cout << "PointCloudGenerator Plugin was created" << std::endl;
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

    pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(topic_);
    
  }

  //////////////////////////////////////////////////////////////
  void PointCloudGenerator::PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
  {
    if(!GeneratedCloud_){
      gzmsg << "PointCloudGenerator::PreUpdate" << std::endl;
      auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
      _ecm.CreateComponent(worldEntity, gz::sim::components::PhysicsCollisionDetector("bullet"));

      rcEntity_ = _ecm.CreateEntity();
      _ecm.CreateComponent(rcEntity_, gz::sim::components::RaycastData());
      _ecm.CreateComponent(rcEntity_, gz::sim::components::Pose(gz::math::Pose3d(center_x_, center_y_, center_z_, 0, 0, 0)));

      auto &rays = _ecm.Component<gz::sim::components::RaycastData>(rcEntity_)->Data().rays;

      for (double z = min_scan_z_ + resolution_; z <= max_scan_z_; z += 1.0)
      {
        for (double y = min_scan_y_; y <= max_scan_y_; y += resolution_)
        {
          for (double x = min_scan_x_; x <= max_scan_x_; x += resolution_)
          {
            gz::sim::components::RayInfo r;
            r.start = gz::math::Vector3d(x, y, z);
            r.end   = gz::math::Vector3d(x, y, min_scan_z_);
            rays.push_back(r);
          }
        }
      }
    }

  }

  //////////////////////////////////////////////////////////////////////////////
  void PointCloudGenerator::PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    if(!GeneratedCloud_){
      gzmsg << "PointCloudGenerator::PostUpdate" << std::endl;

      auto &rays = _ecm.Component<gz::sim::components::RaycastData>(rcEntity_)->Data().rays;
      auto &results = _ecm.Component<gz::sim::components::RaycastData>(rcEntity_)->Data().results;
      gzmsg << "Rays emitted: " << rays.size() << std::endl;
      gzmsg << "Results Intercepted: " << results.size() << std::endl;

      //gz::msgs::PointCloudPacked cloud;

      cloud_.set_width(static_cast<uint32_t>(results.size()));
      cloud_.set_height(1);
      cloud_.set_is_dense(true);
      cloud_.set_is_bigendian(false);

      gz::msgs::Header::Map *frame = cloud_.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(frame_id_);

      auto addField = [this](const std::string &name, uint32_t offset){
        auto *f = this->cloud_.add_field();
        f->set_name(name);
        f->set_offset(offset);
        f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);  
        f->set_count(1);
      };

      const uint32_t pstep = 3u * sizeof(float);
      addField("x", 0); addField("y", 4); addField("z", 8);
      cloud_.set_point_step(pstep);
      cloud_.set_row_step(cloud_.point_step() * cloud_.width());

      cloud_.mutable_data()->resize(cloud_.row_step() * cloud_.height());

      gz::msgs::PointCloudPackedIterator<float> iterX(cloud_, "x");
      gz::msgs::PointCloudPackedIterator<float> iterY(cloud_, "y");
      gz::msgs::PointCloudPackedIterator<float> iterZ(cloud_, "z");

      for (size_t i = 0; i < results.size(); ++i) {
        if (results[i].point.IsFinite()){
          //std::cout << results[i].point << std::endl;
          *iterX = static_cast<float>(results[i].point[0]);
          *iterY = static_cast<float>(results[i].point[1]);
          *iterZ = static_cast<float>(results[i].point[2]);
          ++iterX; ++iterY; ++iterZ;
        }
      }
      GeneratedCloud_ = true;
    }
    // should be a timer callback
    if(iteration_%10000 == 0 && GeneratedCloud_){
      pub_.Publish(cloud_);
      iteration_ = 1;
    } else {
      ++iteration_;
    }
  }
}  // namespace ray_tracing_plugin

GZ_ADD_PLUGIN(
    ray_tracing_plugin::PointCloudGenerator,
    gz::sim::System,
    ray_tracing_plugin::PointCloudGenerator::ISystemConfigure,
    ray_tracing_plugin::PointCloudGenerator::ISystemPreUpdate,
    ray_tracing_plugin::PointCloudGenerator::ISystemPostUpdate)