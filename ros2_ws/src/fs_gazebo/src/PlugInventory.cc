// fs::PlugInventory — the saplings a robot carries, as visuals riding on their carrier links.
//
// Carriers are drum pockets, carousel cups or tray cells; holders are the places a plug is carried
// while being handled (gripper, duckbill). Each filled cell has one plug visual, created as a child
// of the carrier link, so it moves with the drum or tray at no physics cost. Commands move a plug
// between a cell and a holder or between holders, or drop it (the planting head then leaves the
// planted sapling in the hole).
//
//   <plugin filename="fs_plug_inventory_system" name="fs::PlugInventory">
//     <plug_mesh>file:///.../plug.obj</plug_mesh>
//     <cmd_topic>/model/jaal_1/plugs/cmd</cmd_topic> <status_topic>/model/jaal_1/plugs/status</status_topic>
//     <carrier><name>drum_f</name><link>drum_f_link</link>
//       <ring><radius>0.41</radius><count>45</count><tiers>4</tiers><z0>0.117</z0><dz>0.4</dz></ring>
//       <fill>180</fill></carrier>
//     <carrier><name>tray</name><link>base_link</link>
//       <grid><x0>-0.404</x0><y0>-0.234</y0><z>0.655</z><nx>5</nx><ny>10</ny><pitch>0.052</pitch></grid></carrier>
//     <holder><name>gripper</name><link>picker_y_link</link><xyz>0 -0.19 -0.06</xyz></holder>
//   </plugin>
//
// Commands (gz.msgs.StringMsg):
//   pick <carrier> <cell|next> <holder>    take the plug out of a cell into a holder
//   move <from_holder> <to_holder>         hand a plug on (gripper -> duckbill)
//   drop <holder>                          the plug has left the machine (it is in the ground)
//   fill <carrier> [n]                     restock: fill empty cells, n at most
//   empty <carrier> [n]                    unload: clear filled cells from the last one, n at most
// Status (JSON, 5 Hz): per carrier count, cells, occ ('1' per full cell), next full cell with its local position, ring
// angle and world position; per holder whether it holds a plug.
#include <gz/msgs/stringmsg.pb.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Visual.hh>

namespace fs
{
using gz::math::Pose3d;
using gz::math::Vector3d;

class PlugInventory : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate
{
  struct Carrier
  {
    std::string name, linkName;
    gz::sim::Entity link{gz::sim::kNullEntity};
    std::vector<Vector3d> cells;          // local positions (plug bottom)
    std::vector<double> angles;           // ring angle of each cell (0 for grids)
    std::vector<gz::sim::Entity> plugs;   // visual per cell, kNullEntity when empty
    int fill{0};
  };
  struct Holder
  {
    std::string name, linkName;
    gz::sim::Entity link{gz::sim::kNullEntity};
    Vector3d xyz;
    gz::sim::Entity plug{gz::sim::kNullEntity};
  };

public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_events) override
  {
    model_ = gz::sim::Model(_entity);
    events_ = &_events;
    name_ = model_.Name(_ecm);
    mesh_ = _sdf->Get<std::string>("plug_mesh", "").first;
    for (auto c = _sdf->FindElement("carrier"); c; c = c->GetNextElement("carrier"))
    {
      Carrier k;
      k.name = c->Get<std::string>("name", "").first;
      k.linkName = c->Get<std::string>("link", "").first;
      k.fill = c->Get<int>("fill", -1).first;
      if (c->HasElement("ring"))
      {
        auto r = c->FindElement("ring");
        const double rad = r->Get<double>("radius", 0.4).first, z0 = r->Get<double>("z0", 0.0).first;
        const double dz = r->Get<double>("dz", 0.0).first;
        const int n = r->Get<int>("count", 12).first, tiers = r->Get<int>("tiers", 1).first;
        for (int t = 0; t < tiers; ++t)
          for (int i = 0; i < n; ++i)
          {
            const double a = 2.0 * M_PI * i / n;
            k.cells.emplace_back(rad * std::cos(a), rad * std::sin(a), z0 + dz * t);
            k.angles.push_back(a);
          }
      }
      else if (c->HasElement("grid"))
      {
        auto g = c->FindElement("grid");
        const double x0 = g->Get<double>("x0", 0.0).first, y0 = g->Get<double>("y0", 0.0).first;
        const double z = g->Get<double>("z", 0.0).first, p = g->Get<double>("pitch", 0.052).first;
        const int nx = g->Get<int>("nx", 5).first, ny = g->Get<int>("ny", 10).first;
        for (int i = 0; i < nx; ++i)
          for (int j = 0; j < ny; ++j)
          {
            k.cells.emplace_back(x0 + i * p, y0 + j * p, z);
            k.angles.push_back(0.0);
          }
      }
      if (k.fill < 0) k.fill = static_cast<int>(k.cells.size());
      k.plugs.assign(k.cells.size(), gz::sim::kNullEntity);
      carriers_[k.name] = k;
    }
    for (auto h = _sdf->FindElement("holder"); h; h = h->GetNextElement("holder"))
    {
      Holder k;
      k.name = h->Get<std::string>("name", "").first;
      k.linkName = h->Get<std::string>("link", "").first;
      k.xyz = h->Get<Vector3d>("xyz", Vector3d::Zero).first;
      holders_[k.name] = k;
    }
    const auto cmd = _sdf->Get<std::string>("cmd_topic", "/model/" + name_ + "/plugs/cmd").first;
    const auto st = _sdf->Get<std::string>("status_topic", "/model/" + name_ + "/plugs/status").first;
    node_.Subscribe(cmd, &PlugInventory::OnCmd, this);
    pub_ = node_.Advertise<gz::msgs::StringMsg>(st);
    size_t cells = 0;
    for (auto &[n, c] : carriers_) cells += c.cells.size();
    gzmsg << "[fs::PlugInventory] " << carriers_.size() << " carriers (" << cells << " cells), " << holders_.size()
          << " holders, cmd " << cmd << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!resolved_)
    {
      if (!Resolve(_ecm)) return;
      for (auto &[n, c] : carriers_) Fill(_ecm, c, c.fill);
    }
    std::vector<std::string> cmds;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cmds.swap(pending_);
    }
    for (const auto &c : cmds) Execute(_ecm, c);
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (!resolved_ || t - lastPub_ < 0.2) return;
    lastPub_ = t;
    std::ostringstream js;
    js << std::fixed << std::setprecision(4) << "{\"t\":" << t << ",\"carriers\":{";
    bool first = true;
    for (const auto &[n, c] : carriers_)
    {
      int count = 0, next = -1;
      for (size_t i = 0; i < c.plugs.size(); ++i)
        if (c.plugs[i] != gz::sim::kNullEntity) { ++count; if (next < 0) next = static_cast<int>(i); }
      std::string occ(c.plugs.size(), '0');               // one character per cell, for species bookkeeping
      for (size_t i = 0; i < c.plugs.size(); ++i)
        if (c.plugs[i] != gz::sim::kNullEntity) occ[i] = '1';
      js << (first ? "" : ",") << "\"" << n << "\":{\"count\":" << count << ",\"cells\":" << c.cells.size()
         << ",\"next\":" << next << ",\"occ\":\"" << occ << "\"";
      if (next >= 0)
      {
        const auto &p = c.cells[static_cast<size_t>(next)];
        const Pose3d w = gz::sim::Link(c.link).WorldPose(_ecm).value_or(Pose3d::Zero) * Pose3d(p, {});
        js << ",\"next_local\":[" << p.X() << "," << p.Y() << "," << p.Z() << "],\"next_angle\":"
           << c.angles[static_cast<size_t>(next)] << ",\"next_world\":[" << w.Pos().X() << "," << w.Pos().Y() << ","
           << w.Pos().Z() << "]";
      }
      js << "}";
      first = false;
    }
    js << "},\"holders\":{";
    first = true;
    for (const auto &[n, h] : holders_)
    {
      js << (first ? "" : ",") << "\"" << n << "\":" << (h.plug != gz::sim::kNullEntity ? "true" : "false");
      first = false;
    }
    js << "}}";
    gz::msgs::StringMsg msg;
    msg.set_data(js.str());
    pub_.Publish(msg);
  }

private:
  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    for (auto &[n, c] : carriers_)
    {
      c.link = model_.LinkByName(_ecm, c.linkName);
      if (c.link == gz::sim::kNullEntity) { Warn("carrier link '" + c.linkName + "' not found"); return false; }
    }
    for (auto &[n, h] : holders_)
    {
      h.link = model_.LinkByName(_ecm, h.linkName);
      if (h.link == gz::sim::kNullEntity) { Warn("holder link '" + h.linkName + "' not found"); return false; }
    }
    resolved_ = true;
    return true;
  }

  void Warn(const std::string &m)
  {
    if (!warned_) gzerr << "[fs::PlugInventory] " << m << "\n";
    warned_ = true;
  }

  gz::sim::Entity MakePlug(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity parent, const Vector3d &at)
  {
    sdf::Visual vis;
    vis.SetName("plug_" + std::to_string(serial_++));
    vis.SetRawPose(Pose3d(at, {0, 0, 0}));
    sdf::Geometry geom;
    geom.SetType(sdf::GeometryType::MESH);
    sdf::Mesh mesh;
    mesh.SetUri(mesh_);
    geom.SetMeshShape(mesh);
    vis.SetGeom(geom);
    vis.SetCastShadows(false);
    gz::sim::SdfEntityCreator creator(_ecm, *events_);
    const auto e = creator.CreateEntities(&vis);
    creator.SetParent(e, parent);
    return e;
  }

  void Remove(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity &e)
  {
    if (e != gz::sim::kNullEntity) _ecm.RequestRemoveEntity(e);
    e = gz::sim::kNullEntity;
  }

  void Fill(gz::sim::EntityComponentManager &_ecm, Carrier &c, int n)
  {
    int added = 0;
    for (size_t i = 0; i < c.cells.size() && added < n; ++i)
      if (c.plugs[i] == gz::sim::kNullEntity)
      {
        c.plugs[i] = MakePlug(_ecm, c.link, c.cells[i]);
        ++added;
      }
  }

  void Execute(gz::sim::EntityComponentManager &_ecm, const std::string &cmd)
  {
    std::istringstream in(cmd);
    std::string verb, a, b, c;
    in >> verb >> a >> b >> c;
    if (verb == "pick")
    {
      auto ci = carriers_.find(a);
      auto hi = holders_.find(c);
      if (ci == carriers_.end() || hi == holders_.end()) { gzwarn << "[fs::PlugInventory] bad pick: " << cmd << "\n"; return; }
      auto &car = ci->second;
      int cell = -1;
      if (b == "next")
      {
        for (size_t i = 0; i < car.plugs.size(); ++i)
          if (car.plugs[i] != gz::sim::kNullEntity) { cell = static_cast<int>(i); break; }
      }
      else
        cell = std::stoi(b);
      if (cell < 0 || cell >= static_cast<int>(car.plugs.size()) || car.plugs[static_cast<size_t>(cell)] == gz::sim::kNullEntity)
      {
        gzwarn << "[fs::PlugInventory] " << a << " cell " << b << " is empty\n";
        return;
      }
      Remove(_ecm, car.plugs[static_cast<size_t>(cell)]);
      Remove(_ecm, hi->second.plug);
      hi->second.plug = MakePlug(_ecm, hi->second.link, hi->second.xyz);
    }
    else if (verb == "move")
    {
      auto from = holders_.find(a), to = holders_.find(b);
      if (from == holders_.end() || to == holders_.end() || from->second.plug == gz::sim::kNullEntity) return;
      Remove(_ecm, from->second.plug);
      Remove(_ecm, to->second.plug);
      to->second.plug = MakePlug(_ecm, to->second.link, to->second.xyz);
    }
    else if (verb == "drop")
    {
      auto h = holders_.find(a);
      if (h != holders_.end()) Remove(_ecm, h->second.plug);
    }
    else if (verb == "fill")
    {
      auto ci = carriers_.find(a);
      if (ci != carriers_.end()) Fill(_ecm, ci->second, b.empty() ? 1 << 30 : std::stoi(b));
    }
    else if (verb == "empty")
    {
      auto ci = carriers_.find(a);
      if (ci == carriers_.end()) return;
      int n = b.empty() ? 1 << 30 : std::stoi(b);
      auto &plugs = ci->second.plugs;
      for (size_t i = plugs.size(); i-- > 0 && n > 0;)
        if (plugs[i] != gz::sim::kNullEntity) { Remove(_ecm, plugs[i]); --n; }
    }
    else
      gzwarn << "[fs::PlugInventory] unknown command '" << cmd << "'\n";
  }

  void OnCmd(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pending_.push_back(_msg.data());
  }

  gz::sim::Model model_;
  std::string name_, mesh_;
  std::map<std::string, Carrier> carriers_;
  std::map<std::string, Holder> holders_;
  bool resolved_{false}, warned_{false};
  int serial_{0};
  double lastPub_{-1};
  gz::sim::EventManager *events_{nullptr};
  std::mutex mtx_;
  std::vector<std::string> pending_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};
}  // namespace fs

GZ_ADD_PLUGIN(fs::PlugInventory, gz::sim::System, fs::PlugInventory::ISystemConfigure,
              fs::PlugInventory::ISystemPreUpdate, fs::PlugInventory::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(fs::PlugInventory, "fs::PlugInventory")
