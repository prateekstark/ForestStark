// fs::Terramechanics — Gazebo system that replaces rigid-body tyre friction with soil mechanics.
//
// Attach to a model. Wheel (or track road-wheel) collisions should carry near-zero friction so
// the physics engine only supplies the normal support; this system then applies, at every
// ground contact and every physics step:
//
//   * a longitudinal soil force from slip (Janosi-Hanamoto), capped by Mohr-Coulomb shear capacity
//   * a lateral soil force from side-slip, same capacity scaled by <lateral_factor>
//   * compaction resistance from Bekker sinkage
//
// The soil's constants are taken at the local moisture (fs::soil::AtMoisture): a monsoon-wet
// vertisol loses most of its traction, damp sand gains a little. Ground that any robot in the
// world has already pressed down is remembered on a shared rut map, so a wheel that follows a
// rut (a trailer behind its tractor, a second Jaal on the first one's track) only pays for the
// extra compaction (fs::soil::MultiPassResistance).
//
// Normal loads come from a quasi-static distribution of the whole model's weight over the
// contacts that touch the ground (soil-map elevation + <contact_tolerance>). Soil type,
// moisture and elevation are read from the site soil map.
//
//   <plugin filename="fs_terramechanics_system" name="fs::Terramechanics">
//     <soil_map>/abs/path/site.soilmap</soil_map>   (optional; else <default_soil>)
//     <default_soil>sandy_loam</default_soil>
//     <topic>/model/jaal_1/terramechanics</topic>     (JSON telemetry, gz.msgs.StringMsg)
//     <publish_rate>10</publish_rate>
//     <wheel>
//       <link>wheel_fl_link</link> <radius>0.38</radius> <width>0.22</width>
//       <contact_length>0.2</contact_length> <axis>0 1 0</axis>
//     </wheel>
//     ...
//   </plugin>
#include <gz/msgs/stringmsg.pb.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "fs_gazebo/SoilMap.hh"
#include "fs_gazebo/SoilModel.hh"

namespace fs
{
using gz::math::Vector3d;
namespace soil_ns = fs::soil;

static soil_ns::Vec3 ToV(const Vector3d &v) { return {v.X(), v.Y(), v.Z()}; }
static Vector3d ToG(const soil_ns::Vec3 &v) { return {v.x, v.y, v.z}; }

/// One rut map per site, shared by every model's Terramechanics system in this server.
static std::shared_ptr<soil_ns::RutMap> SharedRuts(const std::string &site)
{
  static std::mutex mtx;
  static std::map<std::string, std::weak_ptr<soil_ns::RutMap>> maps;
  std::lock_guard<std::mutex> lk(mtx);
  auto r = maps[site].lock();
  if (!r) { r = std::make_shared<soil_ns::RutMap>(0.1); maps[site] = r; }
  return r;
}

class Terramechanics : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
  struct Wheel
  {
    std::string linkName;
    gz::sim::Link link;
    double r{0.3}, b{0.2}, l{0.15};
    Vector3d axisLocal{0, 1, 0};
    soil_ns::Bristle lon, lat;
    // telemetry
    double N{0}, z{0}, slip{0}, fx{0}, fy{0}, rc{0}, fmax{0}, rut{0};
    bool touching{false};
  };

public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    std::string mapPath = _sdf->Get<std::string>("soil_map", "").first;
    const std::string def = _sdf->Get<std::string>("default_soil", "sandy_loam").first;
    if (!mapPath.empty())
    {
      try { map_.Load(mapPath); }
      catch (const std::exception &e) { gzerr << "[fs::Terramechanics] " << e.what() << ", using " << def << "\n"; }
    }
    if (!map_.Loaded())
    {
      soil_ns::SoilParams s = soil_ns::DefaultSoils()[1];
      for (const auto &d : soil_ns::DefaultSoils()) if (d.name == def) s = d;
      map_.MakeUniform(s);
    }
    topic_ = _sdf->Get<std::string>("topic", "").first;
    publishPeriod_ = 1.0 / std::max(0.5, _sdf->Get<double>("publish_rate", 10.0).first);
    contactTol_ = _sdf->Get<double>("contact_tolerance", 0.03).first;
    bristle_ = _sdf->Get<double>("bristle_deflection", 0.005).first;
    latFactor_ = _sdf->Get<double>("lateral_factor", 0.85).first;
    ruts_ = SharedRuts(mapPath.empty() ? "uniform:" + def : mapPath);

    for (auto w = _sdf->FindElement("wheel"); w; w = w->GetNextElement("wheel"))
    {
      Wheel wh;
      wh.linkName = w->Get<std::string>("link", "").first;
      wh.r = w->Get<double>("radius", 0.3).first;
      wh.b = w->Get<double>("width", 0.2).first;
      wh.l = w->Get<double>("contact_length", 0.5 * wh.r).first;
      wh.axisLocal = w->Get<Vector3d>("axis", Vector3d(0, 1, 0)).first.Normalized();
      if (wh.linkName.empty()) { gzerr << "[fs::Terramechanics] <wheel> without <link>\n"; continue; }
      wheels_.push_back(wh);
    }
    if (topic_.empty())
      topic_ = "/model/" + model_.Name(_ecm) + "/terramechanics";
    pub_ = node_.Advertise<gz::msgs::StringMsg>(topic_);
    gzmsg << "[fs::Terramechanics] " << wheels_.size() << " contacts, soil map '" << map_.Name()
          << "', telemetry on " << topic_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!resolved_) Resolve(_ecm);
    if (!resolved_ || _info.paused) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0) return;

    // Whole-model mass and centre of gravity.
    double mass = 0;
    Vector3d cg;
    for (auto &l : links_)
    {
      auto in = l.WorldInertial(_ecm);
      if (!in) continue;
      const double m = in->MassMatrix().Mass();
      mass += m;
      cg += in->Pose().Pos() * m;
    }
    if (mass <= 0) return;
    cg /= mass;

    // Contact points and which of them touch the ground.
    std::vector<soil_ns::Vec3> pts(wheels_.size());
    std::vector<bool> touch(wheels_.size(), false);
    std::vector<Vector3d> normals(wheels_.size());
    for (size_t k = 0; k < wheels_.size(); ++k)
    {
      auto &w = wheels_[k];
      auto pose = w.link.WorldPose(_ecm);
      if (!pose) continue;
      const Vector3d c = pose->Pos();
      const Vector3d n = ToG(map_.Normal(c.X(), c.Y()));
      normals[k] = n;
      const Vector3d p = c - n * w.r;
      pts[k] = ToV(p);
      const double gap = p.Z() - map_.Elevation(p.X(), p.Y());
      touch[k] = gap < contactTol_;
    }
    const auto Nvert = soil_ns::DistributeLoad(pts, touch, ToV(cg), mass);

    double resTotal = 0, latUsedSum = 0, latCapSum = 0, slipSum = 0;
    int nTouch = 0;
    for (size_t k = 0; k < wheels_.size(); ++k)
    {
      auto &w = wheels_[k];
      w.touching = touch[k];
      w.fx = w.fy = w.rc = w.z = w.slip = w.fmax = w.rut = 0;
      w.N = 0;
      if (!touch[k]) { w.lon.z = 0; w.lat.z = 0; continue; }
      auto pose = w.link.WorldPose(_ecm);
      auto vel = w.link.WorldLinearVelocity(_ecm);
      auto omg = w.link.WorldAngularVelocity(_ecm);
      if (!pose || !vel || !omg) continue;

      const Vector3d n = normals[k];
      const Vector3d axis = pose->Rot().RotateVector(w.axisLocal);
      const Vector3d fwd = axis.Cross(n).Normalized();
      const Vector3d lat = n.Cross(fwd).Normalized();
      const Vector3d c = pose->Pos();
      const Vector3d p = c - n * w.r;

      const auto smp = map_.Sample(p.X(), p.Y());
      const auto soil = soil_ns::AtMoisture(*smp.soil, smp.moisture);
      lastSoil_ = soil.name;
      lastMoisture_ = smp.moisture;
      const double N = Nvert[k] * std::max(0.2, n.Z());
      w.N = N;
      w.z = soil_ns::Sinkage(soil, N, w.b, w.l);
      // ground just ahead of the patch tells whether it was pressed before; the wheel presses its
      // own rut behind the patch, a cell clear of where it looks, so it never meets itself
      const double dir = vel->Dot(fwd) >= 0 ? 1.0 : -1.0;
      const Vector3d ahead = p + fwd * (dir * (0.5 * w.l + 0.1));
      const Vector3d behind = p - fwd * (dir * 0.5 * w.l);
      w.rut = ruts_->Depth(ahead.X(), ahead.Y());
      w.rc = soil_ns::MultiPassResistance(soil, w.z, w.rut, w.b);
      if (std::fabs(vel->Dot(fwd)) > 0.02) ruts_->Press(behind.X(), behind.Y(), w.z, w.b, lat.X(), lat.Y());
      w.fmax = soil_ns::ShearCapacity(soil, N, w.b, w.l);

      const double vx = vel->Dot(fwd);
      const double vy = vel->Dot(lat);
      const double surf = omg->Dot(axis) * w.r;
      const double vSlip = vx - surf;                       // contact patch relative to soil
      w.slip = soil_ns::SlipRatio(vx, surf);
      const double den = std::max({std::fabs(vx), std::fabs(surf), 0.05});
      const double gLon = w.fmax * std::max(soil_ns::ShearFraction(std::fabs(vSlip) / den, w.l, soil.K), 0.12);
      const double gLat = latFactor_ * w.fmax *
                          std::max(soil_ns::ShearFraction(std::fabs(vy) / den, w.l, soil.K), 0.12);

      // Bristle gains limited for explicit-integration stability against the lighter of the
      // two bodies the force acts on: the wheel's spin inertia or its share of the chassis.
      double mSpin = 50.0;
      if (auto I = w.link.WorldInertiaMatrix(_ecm)) mSpin = std::max(1.0, ((*I) * axis).Dot(axis) / (w.r * w.r));
      const double mShare = std::max(5.0, N / soil_ns::kG);
      const double mLon = std::min(mSpin, mShare);
      const double s0Lon = std::min(std::max(w.fmax, 50.0) / bristle_, 0.1 * mLon / (dt * dt));
      const double s1Lon = std::min(2.0 * 0.7 * std::sqrt(s0Lon * mLon), 0.3 * mLon / dt);
      const double s0Lat = std::min(std::max(w.fmax, 50.0) / bristle_, 0.1 * mShare / (dt * dt));
      const double s1Lat = std::min(2.0 * 0.7 * std::sqrt(s0Lat * mShare), 0.3 * mShare / dt);

      w.fx = -w.lon.Step(vSlip, gLon, s0Lon, s1Lon, dt);
      w.fy = -w.lat.Step(vy, gLat, s0Lat, s1Lat, dt);
      const double res = w.rc * std::tanh(vx / 0.05);

      const Vector3d F = fwd * (w.fx - res) + lat * w.fy;
      const Vector3d torque = (p - c).Cross(F);
      w.link.AddWorldWrench(_ecm, F, torque);

      resTotal += w.rc;
      latUsedSum += std::fabs(w.fy);
      latCapSum += latFactor_ * w.fmax;
      slipSum += w.slip;
      ++nTouch;
    }

    simTime_ += dt;
    if (simTime_ - lastPub_ >= publishPeriod_)
    {
      lastPub_ = simTime_;
      Publish(mass, cg, nTouch ? slipSum / nTouch : 0.0, latCapSum > 0 ? latUsedSum / latCapSum : 0.0, resTotal);
    }
  }

private:
  void Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (!model_.Valid(_ecm)) return;
    links_.clear();
    for (auto e : model_.Links(_ecm))
    {
      gz::sim::Link l(e);
      l.EnableVelocityChecks(_ecm, true);
      links_.push_back(l);
    }
    for (auto &w : wheels_)
    {
      auto e = model_.LinkByName(_ecm, w.linkName);
      if (e == gz::sim::kNullEntity)
      {
        gzerr << "[fs::Terramechanics] link '" << w.linkName << "' not found\n";
        return;
      }
      w.link = gz::sim::Link(e);
      w.link.EnableVelocityChecks(_ecm, true);
    }
    resolved_ = true;
  }

  void Publish(double mass, const Vector3d &cg, double meanSlip, double latUtil, double res)
  {
    std::ostringstream js;
    js << std::fixed << std::setprecision(4);
    js << "{\"t\":" << simTime_ << ",\"mass\":" << mass << ",\"cg\":[" << cg.X() << "," << cg.Y() << "," << cg.Z()
       << "],\"moisture\":" << lastMoisture_ << ",\"rut_area\":" << ruts_->Area()
       << ",\"rut_max\":" << ruts_->MaxDepth() << ",\"mean_slip\":" << meanSlip << ",\"lateral_utilisation\":" << latUtil
       << ",\"motion_resistance\":" << res << ",\"wheels\":[";
    for (size_t k = 0; k < wheels_.size(); ++k)
    {
      const auto &w = wheels_[k];
      js << (k ? "," : "") << "{\"name\":\"" << w.linkName << "\",\"touch\":" << (w.touching ? "true" : "false")
         << ",\"N\":" << w.N << ",\"z\":" << w.z << ",\"slip\":" << w.slip << ",\"fx\":" << w.fx
         << ",\"fy\":" << w.fy << ",\"rc\":" << w.rc << ",\"rut\":" << w.rut << ",\"fmax\":" << w.fmax << "}";
    }
    js << "],\"soil\":\"" << lastSoil_ << "\"}";
    gz::msgs::StringMsg msg;
    msg.set_data(js.str());
    pub_.Publish(msg);
  }

  gz::sim::Model model_;
  std::vector<gz::sim::Link> links_;
  std::vector<Wheel> wheels_;
  soil_ns::SoilMap map_;
  bool resolved_{false};
  std::string topic_;
  double publishPeriod_{0.1}, contactTol_{0.06}, bristle_{0.005}, latFactor_{0.85};
  double simTime_{0}, lastPub_{-1};
  std::string lastSoil_{""};
  double lastMoisture_{0};
  std::shared_ptr<soil_ns::RutMap> ruts_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};

}  // namespace fs

GZ_ADD_PLUGIN(fs::Terramechanics, gz::sim::System, fs::Terramechanics::ISystemConfigure,
              fs::Terramechanics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fs::Terramechanics, "fs::Terramechanics")
