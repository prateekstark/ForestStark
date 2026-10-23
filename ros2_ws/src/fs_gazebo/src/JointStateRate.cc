// fs::JointStateRate — joint states at a fixed rate (gz.msgs.Model, same layout as the stock
// JointStatePublisher), so ROS consumers are not flooded with one message per physics step.
//
//   <plugin filename="fs_joint_state_rate_system" name="fs::JointStateRate">
//     <topic>/model/jaal_1/joint_states</topic> <update_rate>30</update_rate>
//   </plugin>
#include <gz/msgs/model.pb.h>

#include <chrono>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace fs
{
class JointStateRate : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    name_ = model_.Name(_ecm);
    const auto topic = _sdf->Get<std::string>("topic", "/model/" + name_ + "/joint_states").first;
    period_ = 1.0 / std::max(1.0, _sdf->Get<double>("update_rate", 30.0).first);
    pub_ = node_.Advertise<gz::msgs::Model>(topic);
  }

  // PostUpdate only gets a const ECM, so the position/velocity components are enabled here.
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!joints_.empty()) return;
    for (auto e : model_.Joints(_ecm))
    {
      gz::sim::Joint j(e);
      j.EnablePositionCheck(_ecm, true);
      j.EnableVelocityCheck(_ecm, true);
      joints_.push_back(j);
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (joints_.empty() || t - last_ < period_) return;
    last_ = t;
    gz::msgs::Model msg;
    msg.set_name(name_);
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    msg.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    for (const auto &j : joints_)
    {
      auto p = j.Position(_ecm);
      auto v = j.Velocity(_ecm);
      if (!p || p->empty()) continue;
      auto *jm = msg.add_joint();
      jm->set_name(j.Name(_ecm).value_or(""));
      jm->mutable_axis1()->set_position(p->front());
      jm->mutable_axis1()->set_velocity(v && !v->empty() ? v->front() : 0.0);
    }
    pub_.Publish(msg);
  }

private:
  gz::sim::Model model_;
  std::string name_;
  std::vector<gz::sim::Joint> joints_;
  double period_{1.0 / 30}, last_{-1e9};
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};
}  // namespace fs

GZ_ADD_PLUGIN(fs::JointStateRate, gz::sim::System, fs::JointStateRate::ISystemConfigure,
              fs::JointStateRate::ISystemPreUpdate, fs::JointStateRate::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(fs::JointStateRate, "fs::JointStateRate")
