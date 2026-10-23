// fs::AugerSoil — Gazebo system for an auger (or percussive pick) cutting into the site soil.
//
// The system owns the feed joint (prismatic, positive = into the ground) and the spin joint.
// Above ground it feeds at <free_speed>. In soil the feed rate is the smaller of the flight
// limit and the power limit eta P / (e A), where e is the soil's specific boring energy scaled
// by moisture; if the thrust k e A exceeds <max_thrust> the feed slows in proportion. The soil's
// reaction (thrust along the tool, cutting torque about it) is applied to the tip link, so the
// joint servos carry it into the robot: auger thrust really unloads the tracks and shows up in
// the stability monitor.
//
// When the hole reaches the soil map's depth-to-rock the auger refuses ("refusal"). A "pick"
// command keeps going through rock at <rock_energy>, which is how Pahadi's percussive tool
// is modelled. A kankar (calcic) pan from the soil map's hardpan layer is cut at
// <hardpan_factor> times the soil's energy: the CSSRI auger-hole technique for sodic land bores
// through it so roots can reach the moister, less sodic subsoil. Re-boring an existing hole costs little energy.
//
// When the auger is withdrawn the hole's wall stands up to its critical height at the local moisture
// (fs::soil::CriticalWallHeight); below that it sloughs in and loose spoil fills the bottom
// ("collapse", "open_depth" in the status). Dry dune sand caves; pouring water into the pit
// ("wet <litres>") gives the wall capillary cohesion, and a re-bore then leaves an open hole.
//
// Commands (gz.msgs.StringMsg on <cmd_topic>):  "bore 0.30" | "pick 0.30" | "retract" | "stop"
//   | "hold" (safety: spin and feed stop where they are) | "resume" (carry on with the held phase)
//   | "wet 5" (litres poured into the last hole)
// Status   (gz.msgs.StringMsg JSON on <status_topic>, <publish_rate> Hz).
#include <gz/msgs/stringmsg.pb.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "fs_gazebo/SoilMap.hh"
#include "fs_gazebo/SoilModel.hh"

namespace fs
{
using gz::math::Vector3d;
namespace sm = fs::soil;

class AugerSoil : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate
{
  struct Hole { double x, y, depth; };
  struct Wet { double x, y, dm; };

public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    feedJointName_ = _sdf->Get<std::string>("feed_joint", "auger_feed_joint").first;
    spinJointName_ = _sdf->Get<std::string>("spin_joint", "").first;
    tipLinkName_ = _sdf->Get<std::string>("tip_link", "auger_link").first;
    // where the soil's reaction is applied: the tip by default, or a heavier mounting link (several
    // kN on a light bit link destabilises the solver; the joints carry it into the machine either way)
    reactionLinkName_ = _sdf->Get<std::string>("reaction_link", "").first;
    toolAxis_ = _sdf->Get<Vector3d>("tool_axis", Vector3d(0, 0, -1)).first.Normalized();
    tipOffset_ = _sdf->Get<Vector3d>("tip_offset", Vector3d(0, 0, 0)).first;
    p_.diameter = _sdf->Get<double>("diameter", 0.12).first;
    p_.maxPower = _sdf->Get<double>("max_power", 4000.0).first;
    p_.efficiency = _sdf->Get<double>("efficiency", 0.8).first;
    p_.maxThrust = _sdf->Get<double>("max_thrust", 3000.0).first;
    p_.thrustCoeff = _sdf->Get<double>("thrust_coeff", 0.012).first;
    p_.maxFeed = _sdf->Get<double>("max_feed", 0.15).first;
    p_.rpm = _sdf->Get<double>("rpm", 120.0).first;
    freeSpeed_ = _sdf->Get<double>("free_speed", 0.25).first;
    retractSpeed_ = _sdf->Get<double>("retract_speed", 0.3).first;
    maxStroke_ = _sdf->Get<double>("max_stroke", 1.0).first;
    rockEnergy_ = _sdf->Get<double>("rock_energy", 45.0).first * 1e6;
    pickEfficiency_ = _sdf->Get<double>("pick_efficiency", 0.5).first;
    hardpanFactor_ = _sdf->Get<double>("hardpan_factor", 3.5).first;

    const std::string mapPath = _sdf->Get<std::string>("soil_map", "").first;
    const std::string def = _sdf->Get<std::string>("default_soil", "sandy_loam").first;
    if (!mapPath.empty())
    {
      try { map_.Load(mapPath); }
      catch (const std::exception &e) { gzerr << "[fs::AugerSoil] " << e.what() << "\n"; }
    }
    if (!map_.Loaded())
    {
      sm::SoilParams s = sm::DefaultSoils()[1];
      for (const auto &d : sm::DefaultSoils()) if (d.name == def) s = d;
      map_.MakeUniform(s);
    }

    const std::string name = model_.Name(_ecm);
    cmdTopic_ = _sdf->Get<std::string>("cmd_topic", "/model/" + name + "/auger/cmd").first;
    statusTopic_ = _sdf->Get<std::string>("status_topic", "/model/" + name + "/auger/status").first;
    publishPeriod_ = 1.0 / std::max(1.0, _sdf->Get<double>("publish_rate", 20.0).first);
    pub_ = node_.Advertise<gz::msgs::StringMsg>(statusTopic_);
    node_.Subscribe(cmdTopic_, &AugerSoil::OnCmd, this);
    gzmsg << "[fs::AugerSoil] D=" << p_.diameter << " m, P=" << p_.maxPower << " W, thrust<=" << p_.maxThrust
          << " N, soil map '" << map_.Name() << "', cmd " << cmdTopic_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!resolved_) Resolve(_ecm);
    if (!resolved_ || _info.paused) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    simTime_ += dt;
    TakeCommand();

    const double q = feed_.Position(_ecm) && !feed_.Position(_ecm)->empty() ? feed_.Position(_ecm)->front() : 0.0;
    auto pose = tip_.WorldPose(_ecm);
    if (!pose) return;
    const Vector3d tipWorld = pose->Pos() + pose->Rot().RotateVector(tipOffset_);
    const Vector3d dir = pose->Rot().RotateVector(toolAxis_);

    double feedVel = 0.0;
    bool spinning = false;
    rate_ = sm::AugerRate{};

    switch (phase_)
    {
      case Phase::Lowering:
      {
        spinning = true;
        const double ground = map_.Elevation(tipWorld.X(), tipWorld.Y());
        if (tipWorld.Z() <= ground)
        {
          holeX_ = tipWorld.X(); holeY_ = tipWorld.Y(); holeTop_ = ground;
          SetPhase(Phase::Boring);
        }
        else if (q >= maxStroke_ - 1e-3) SetPhase(Phase::NoGround);
        else feedVel = freeSpeed_;
        break;
      }
      case Phase::Boring:
      {
        spinning = true;
        depth_ = std::max(0.0, holeTop_ - tipWorld.Z());
        const auto s = map_.Sample(holeX_, holeY_);
        soilName_ = s.soil->name; rockDepth_ = s.rockDepth; moisture_ = MoistureAt(holeX_, holeY_, s.moisture);
        if (depth_ >= target_) { SetPhase(Phase::Bored); break; }
        double e = s.soil->eb * 1e6 * sm::MoistureFactor(moisture_);
        inPan_ = depth_ >= s.hardpan && depth_ < s.hardpan + 0.3;      // a pan is ~0.3 m thick
        if (inPan_) e *= hardpanFactor_;
        sm::AugerParams p = p_;
        inRock_ = depth_ >= s.rockDepth;
        if (inRock_)
        {
          if (!pick_) { SetPhase(Phase::Refusal); break; }
          e = rockEnergy_;
          p.efficiency = pickEfficiency_;
        }
        for (const auto &h : holes_)
          if (std::hypot(h.x - holeX_, h.y - holeY_) < p_.diameter * 0.5 && depth_ < h.depth) e *= 0.05;
        rate_ = sm::BoreRate(p, e);
        if (q >= maxStroke_ - 1e-3) { SetPhase(Phase::Bored); break; }
        feedVel = rate_.feed;
        energy_ += rate_.power * dt;
        // Soil reaction on the bit: thrust back along the tool, cutting torque against the spin.
        (reaction_.Entity() != gz::sim::kNullEntity ? reaction_ : tip_)
            .AddWorldWrench(_ecm, dir * (-rate_.thrust), dir * (rate_.torque));
        break;
      }
      case Phase::Retracting:
      {
        spinning = true;
        if (q <= 0.003) SetPhase(Phase::Idle);
        else feedVel = -retractSpeed_;
        break;
      }
      default:
        break;
    }

    feed_.SetVelocity(_ecm, {feedVel});
    feedCmd_ = feedVel;
    if (spin_.Entity() != gz::sim::kNullEntity)
      spin_.SetVelocity(_ecm, {spinning ? p_.rpm * 2.0 * sm::kPi / 60.0 : 0.0});

    tipZ_ = tipWorld.Z();
    groundZ_ = map_.Elevation(tipWorld.X(), tipWorld.Y());
    if (simTime_ - lastPub_ >= publishPeriod_) { lastPub_ = simTime_; Publish(q); }
  }

private:
  enum class Phase { Idle, Lowering, Boring, Bored, Refusal, NoGround, Retracting, Stopped, Held };

  static const char *Name(Phase p)
  {
    switch (p)
    {
      case Phase::Idle: return "idle";
      case Phase::Lowering: return "lowering";
      case Phase::Boring: return "boring";
      case Phase::Bored: return "bored";
      case Phase::Refusal: return "refusal";
      case Phase::NoGround: return "no_ground";
      case Phase::Retracting: return "retracting";
      case Phase::Stopped: return "stopped";
      case Phase::Held: return "held";
    }
    return "?";
  }

  void SetPhase(Phase p)
  {
    if ((phase_ == Phase::Boring) && p != Phase::Boring && p != Phase::Held)
      holes_.push_back({holeX_, holeY_, depth_});
    phase_ = p;
    phaseStart_ = simTime_;
  }

  void OnCmd(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pending_.push_back(_msg.data());   // every command in order: "wet" then "bore" can land in one step
  }

  void TakeCommand()
  {
    std::deque<std::string> cmds;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cmds.swap(pending_);
    }
    for (const auto &cmd : cmds) Execute(cmd);
  }

  void Execute(const std::string &cmd)
  {
    std::istringstream in(cmd);
    std::string verb; double d = 0.3;
    in >> verb >> d;
    if (verb == "bore" || verb == "pick")
    {
      pick_ = verb == "pick";
      target_ = std::clamp(d, 0.02, maxStroke_);
      energy_ = 0; depth_ = 0; collapse_ = 0; inRock_ = false;
      SetPhase(Phase::Lowering);
    }
    else if (verb == "retract")
    {
      if (depth_ > 0.0 && phase_ != Phase::Retracting && phase_ != Phase::Idle)
      {
        // the wall loses the flight's support: what stands above its critical height falls in
        const auto s = map_.Sample(holeX_, holeY_);
        const double wall = std::min(depth_, s.rockDepth);
        collapse_ = sm::HoleCollapse(*s.soil, MoistureAt(holeX_, holeY_, s.moisture), wall);
      }
      SetPhase(Phase::Retracting);
    }
    else if (verb == "wet")
    {
      // water soaks the wall around the pit: volume over the wetted soil cylinder
      const double r = 0.5 * p_.diameter + 0.1;
      const double dm = std::min(0.2, d / 1000.0 / (sm::kPi * r * r * std::max(depth_, 0.3)));
      wets_.push_back({holeX_, holeY_, dm});
      gzmsg << "[fs::AugerSoil] " << d << " L into the pit: moisture +" << dm << "\n";
    }
    else if (verb == "stop") SetPhase(Phase::Stopped);
    else if (verb == "hold")
    {
      if (phase_ != Phase::Held) { heldFrom_ = phase_; SetPhase(Phase::Held); }
    }
    else if (verb == "resume")
    {
      if (phase_ == Phase::Held) { phase_ = heldFrom_; phaseStart_ = simTime_; }
    }
    else gzwarn << "[fs::AugerSoil] unknown command '" << cmd << "'\n";
  }

  /// Map moisture at (x, y) plus what has been poured in nearby.
  double MoistureAt(double x, double y, double base) const
  {
    double m = base;
    for (const auto &w : wets_) if (std::hypot(w.x - x, w.y - y) < 0.3) m += w.dm;
    return std::min(m, 0.45);
  }

  void Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (!model_.Valid(_ecm)) return;
    auto fj = model_.JointByName(_ecm, feedJointName_);
    auto tl = model_.LinkByName(_ecm, tipLinkName_);
    if (fj == gz::sim::kNullEntity || tl == gz::sim::kNullEntity)
    {
      if (!warned_) gzerr << "[fs::AugerSoil] joint '" << feedJointName_ << "' or link '" << tipLinkName_ << "' not found\n";
      warned_ = true;
      return;
    }
    feed_ = gz::sim::Joint(fj);
    feed_.EnablePositionCheck(_ecm, true);
    tip_ = gz::sim::Link(tl);
    if (!reactionLinkName_.empty())
    {
      auto rl = model_.LinkByName(_ecm, reactionLinkName_);
      if (rl != gz::sim::kNullEntity) reaction_ = gz::sim::Link(rl);
      else gzwarn << "[fs::AugerSoil] reaction link '" << reactionLinkName_ << "' not found; using the tip\n";
    }
    if (!spinJointName_.empty())
    {
      auto sj = model_.JointByName(_ecm, spinJointName_);
      if (sj != gz::sim::kNullEntity) spin_ = gz::sim::Joint(sj);
    }
    resolved_ = true;
  }

  void Publish(double q)
  {
    std::ostringstream js;
    js << std::fixed << std::setprecision(4)
       << "{\"t\":" << simTime_ << ",\"phase\":\"" << Name(phase_) << "\",\"phase_time\":" << (simTime_ - phaseStart_)
       << ",\"depth\":" << depth_ << ",\"collapse\":" << collapse_ << ",\"open_depth\":" << depth_ - collapse_
       << ",\"target\":" << target_ << ",\"stroke\":" << q
       << ",\"tip_z\":" << tipZ_ << ",\"ground_z\":" << groundZ_ << ",\"feed_cmd\":" << feedCmd_
       << ",\"feed\":" << rate_.feed << ",\"thrust\":" << rate_.thrust << ",\"torque\":" << rate_.torque
       << ",\"power\":" << rate_.power << ",\"energy_kj\":" << energy_ / 1000.0
       << ",\"thrust_limited\":" << (rate_.thrustLimited ? "true" : "false")
       << ",\"in_rock\":" << (inRock_ ? "true" : "false") << ",\"in_pan\":" << (inPan_ ? "true" : "false")
       << ",\"pick\":" << (pick_ ? "true" : "false")
       << ",\"soil\":\"" << soilName_ << "\",\"rock_depth\":" << rockDepth_ << ",\"moisture\":" << moisture_
       << ",\"hole\":[" << holeX_ << "," << holeY_ << "]}";
    gz::msgs::StringMsg msg;
    msg.set_data(js.str());
    pub_.Publish(msg);
  }

  gz::sim::Model model_;
  gz::sim::Joint feed_, spin_;
  gz::sim::Link tip_, reaction_;
  std::string feedJointName_, spinJointName_, tipLinkName_, reactionLinkName_, cmdTopic_, statusTopic_;
  Vector3d toolAxis_{0, 0, -1}, tipOffset_;
  sm::AugerParams p_;
  sm::AugerRate rate_;
  sm::SoilMap map_;
  double freeSpeed_{0.25}, retractSpeed_{0.3}, maxStroke_{1.0}, rockEnergy_{45e6}, pickEfficiency_{0.5};
  double hardpanFactor_{3.5};
  double tipZ_{0}, groundZ_{0}, feedCmd_{0};
  bool inPan_{false};
  bool resolved_{false}, warned_{false}, pick_{false}, inRock_{false};
  Phase phase_{Phase::Idle};
  Phase heldFrom_{Phase::Idle};
  double simTime_{0}, phaseStart_{0}, lastPub_{-1}, publishPeriod_{0.05};
  double target_{0.3}, depth_{0}, collapse_{0}, energy_{0}, holeX_{0}, holeY_{0}, holeTop_{0};
  double rockDepth_{0}, moisture_{0};
  std::string soilName_{""};
  std::vector<Hole> holes_;
  std::vector<Wet> wets_;
  std::mutex mtx_;
  std::deque<std::string> pending_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};

}  // namespace fs

GZ_ADD_PLUGIN(fs::AugerSoil, gz::sim::System, fs::AugerSoil::ISystemConfigure, fs::AugerSoil::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fs::AugerSoil, "fs::AugerSoil")
