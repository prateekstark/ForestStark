// fs::SwerveDrive — independent four-wheel steering for Jaal.
//
// Takes a body twist (vx, vy, wz) on <topic>, rate-limits it, and for each module computes the
// wheel velocity vector v + w x r_i. The steering joint is driven toward the vector's angle
// (flipping the wheel instead of turning it more than 90 deg), the wheel joint to its speed,
// scaled down while the module is still turning so the tyres are not dragged sideways.
//
// Wheel odometry is integrated from the measured steering angles and wheel speeds (least squares
// over the modules), so it carries the slip that the terramechanics system produces, as a real
// encoder-based odometry would.
//
//   <plugin filename="fs_swerve_drive_system" name="fs::SwerveDrive">
//     <module><steer_joint>steer_fl_joint</steer_joint><wheel_joint>wheel_fl_joint</wheel_joint>
//             <x>0.95</x><y>0.83</y></module>   x4
//     <wheel_radius>0.38</wheel_radius>
//     <topic>/model/jaal_1/cmd_vel</topic> <odom_topic>/model/jaal_1/odometry</odom_topic>
//     <odom_frame>jaal_1/odom</odom_frame> <base_frame>jaal_1/base_link</base_frame>
//     <max_speed>1.5</max_speed> <max_accel>0.6</max_accel> <max_yaw_rate>0.8</max_yaw_rate>
//     <max_yaw_accel>1.0</max_yaw_accel> <max_steer_rate>2.0</max_steer_rate> <steer_p_gain>8</steer_p_gain>
//     <steer_limit>1.9</steer_limit> <cmd_timeout>0.5</cmd_timeout> <odom_rate>50</odom_rate>
//   </plugin>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace fs
{
namespace
{
double Wrap(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}
double Toward(double cur, double target, double maxStep)
{
  return cur + std::clamp(target - cur, -maxStep, maxStep);
}
}  // namespace

class SwerveDrive : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
  struct Module
  {
    std::string steerName, wheelName;
    gz::sim::Joint steer, wheel;
    double x{0}, y{0};
  };

public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    for (auto m = _sdf->FindElement("module"); m; m = m->GetNextElement("module"))
    {
      Module mod;
      mod.steerName = m->Get<std::string>("steer_joint", "").first;
      mod.wheelName = m->Get<std::string>("wheel_joint", "").first;
      mod.x = m->Get<double>("x", 0.0).first;
      mod.y = m->Get<double>("y", 0.0).first;
      modules_.push_back(mod);
    }
    const std::string name = model_.Name(_ecm);
    r_ = _sdf->Get<double>("wheel_radius", 0.38).first;
    maxSpeed_ = _sdf->Get<double>("max_speed", 1.5).first;
    maxAccel_ = _sdf->Get<double>("max_accel", 0.6).first;
    maxYaw_ = _sdf->Get<double>("max_yaw_rate", 0.8).first;
    maxYawAcc_ = _sdf->Get<double>("max_yaw_accel", 1.0).first;
    maxSteerRate_ = _sdf->Get<double>("max_steer_rate", 2.0).first;
    steerKp_ = _sdf->Get<double>("steer_p_gain", 8.0).first;
    steerLimit_ = _sdf->Get<double>("steer_limit", 1.9).first;
    timeout_ = _sdf->Get<double>("cmd_timeout", 0.5).first;
    odomPeriod_ = 1.0 / std::max(1.0, _sdf->Get<double>("odom_rate", 50.0).first);
    odomFrame_ = _sdf->Get<std::string>("odom_frame", name + "/odom").first;
    baseFrame_ = _sdf->Get<std::string>("base_frame", name + "/base_link").first;
    const auto topic = _sdf->Get<std::string>("topic", "/model/" + name + "/cmd_vel").first;
    const auto odomTopic = _sdf->Get<std::string>("odom_topic", "/model/" + name + "/odometry").first;
    node_.Subscribe(topic, &SwerveDrive::OnCmd, this);
    odomPub_ = node_.Advertise<gz::msgs::Odometry>(odomTopic);
    gzmsg << "[fs::SwerveDrive] " << modules_.size() << " modules, cmd " << topic << ", odom " << odomTopic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!resolved_) Resolve(_ecm);
    if (!resolved_ || _info.paused) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    t_ += dt;

    double cvx, cvy, cwz;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const bool stale = t_ - cmdTime_ > timeout_;
      cvx = stale ? 0 : cmd_[0]; cvy = stale ? 0 : cmd_[1]; cwz = stale ? 0 : cmd_[2];
    }
    const double sp = std::hypot(cvx, cvy);
    if (sp > maxSpeed_) { cvx *= maxSpeed_ / sp; cvy *= maxSpeed_ / sp; }
    cwz = std::clamp(cwz, -maxYaw_, maxYaw_);
    vx_ = Toward(vx_, cvx, maxAccel_ * dt);
    vy_ = Toward(vy_, cvy, maxAccel_ * dt);
    wz_ = Toward(wz_, cwz, maxYawAcc_ * dt);

    // Measured state for odometry.
    double num = 0, den = 0;
    for (auto &m : modules_)
    {
      const double th = Pos(m.steer, _ecm);
      const double w = Vel(m.wheel, _ecm);
      const double mvx = w * r_ * std::cos(th), mvy = w * r_ * std::sin(th);
      num += m.x * mvy - m.y * mvx;
      den += m.x * m.x + m.y * m.y;

      // Command.
      const double ux = vx_ - wz_ * m.y, uy = vy_ + wz_ * m.x;
      double speed = std::hypot(ux, uy);
      double ang = speed > 1e-3 ? std::atan2(uy, ux) : th;
      if (std::fabs(Wrap(ang - th)) > M_PI / 2) { ang = Wrap(ang + M_PI); speed = -speed; }
      if (std::fabs(ang) > steerLimit_) { ang = Wrap(ang + M_PI); speed = -speed; }
      const double err = Wrap(ang - th);
      m.steer.SetVelocity(_ecm, {std::clamp(steerKp_ * err, -maxSteerRate_, maxSteerRate_)});
      const double align = std::max(0.0, std::cos(err));
      m.wheel.SetVelocity(_ecm, {speed * align * align / r_});
    }
    const double n = static_cast<double>(modules_.size());
    const double owz = den > 0 ? num / den : 0.0;
    double ovx = 0, ovy = 0;
    for (auto &m : modules_)
    {
      const double th = Pos(m.steer, _ecm), w = Vel(m.wheel, _ecm);
      ovx += w * r_ * std::cos(th) + owz * m.y;
      ovy += w * r_ * std::sin(th) - owz * m.x;
    }
    ovx /= n; ovy /= n;
    yaw_ += owz * dt;
    x_ += (ovx * std::cos(yaw_) - ovy * std::sin(yaw_)) * dt;
    y_ += (ovx * std::sin(yaw_) + ovy * std::cos(yaw_)) * dt;

    if (t_ - lastOdom_ >= odomPeriod_)
    {
      lastOdom_ = t_;
      gz::msgs::Odometry o;
      auto *h = o.mutable_header();
      const auto sec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
      h->mutable_stamp()->set_sec(sec / 1000000000);
      h->mutable_stamp()->set_nsec(sec % 1000000000);
      auto *f = h->add_data(); f->set_key("frame_id"); f->add_value(odomFrame_);
      auto *c = h->add_data(); c->set_key("child_frame_id"); c->add_value(baseFrame_);
      o.mutable_pose()->mutable_position()->set_x(x_);
      o.mutable_pose()->mutable_position()->set_y(y_);
      const gz::math::Quaterniond q(0, 0, yaw_);
      o.mutable_pose()->mutable_orientation()->set_w(q.W());
      o.mutable_pose()->mutable_orientation()->set_z(q.Z());
      o.mutable_twist()->mutable_linear()->set_x(ovx);
      o.mutable_twist()->mutable_linear()->set_y(ovy);
      o.mutable_twist()->mutable_angular()->set_z(owz);
      odomPub_.Publish(o);
    }
  }

private:
  void OnCmd(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_[0] = _msg.linear().x(); cmd_[1] = _msg.linear().y(); cmd_[2] = _msg.angular().z();
    cmdTime_ = t_;
  }

  static double Pos(const gz::sim::Joint &j, const gz::sim::EntityComponentManager &ecm)
  {
    auto p = j.Position(ecm);
    return p && !p->empty() ? p->front() : 0.0;
  }
  static double Vel(const gz::sim::Joint &j, const gz::sim::EntityComponentManager &ecm)
  {
    auto v = j.Velocity(ecm);
    return v && !v->empty() ? v->front() : 0.0;
  }

  void Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    for (auto &m : modules_)
    {
      auto s = model_.JointByName(_ecm, m.steerName), w = model_.JointByName(_ecm, m.wheelName);
      if (s == gz::sim::kNullEntity || w == gz::sim::kNullEntity)
      {
        gzerr << "[fs::SwerveDrive] joints '" << m.steerName << "' / '" << m.wheelName << "' not found\n";
        return;
      }
      m.steer = gz::sim::Joint(s); m.wheel = gz::sim::Joint(w);
      m.steer.EnablePositionCheck(_ecm, true);
      m.wheel.EnableVelocityCheck(_ecm, true);
    }
    resolved_ = true;
  }

  gz::sim::Model model_;
  std::vector<Module> modules_;
  bool resolved_{false};
  double r_{0.38}, maxSpeed_{1.5}, maxAccel_{0.6}, maxYaw_{0.8}, maxYawAcc_{1.0};
  double maxSteerRate_{2.0}, steerKp_{8.0}, steerLimit_{1.9}, timeout_{0.5}, odomPeriod_{0.02};
  double vx_{0}, vy_{0}, wz_{0}, x_{0}, y_{0}, yaw_{0}, t_{0}, lastOdom_{-1}, cmdTime_{-1e9};
  double cmd_[3]{0, 0, 0};
  std::string odomFrame_, baseFrame_;
  std::mutex mtx_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher odomPub_;
};

}  // namespace fs

GZ_ADD_PLUGIN(fs::SwerveDrive, gz::sim::System, fs::SwerveDrive::ISystemConfigure, fs::SwerveDrive::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fs::SwerveDrive, "fs::SwerveDrive")
