// fs_hardware/CanopenSystem — ros2_control hardware for every ForestStark joint drive on one CAN bus.
//
//   <ros2_control name="jaal_hw" type="system">
//     <hardware>
//       <plugin>fs_hardware/CanopenSystem</plugin>
//       <param name="bus">can0</param>            (or "mock": simulated drives, for the desk)
//       <param name="cycle_s">0.01</param>         (controller-manager period; the mock integrates with it)
//       <param name="timeout_s">0.1</param>        (no feedback from a drive for this long -> error)
//     </hardware>
//     <joint name="steer_fl_joint">
//       <command_interface name="position"/>      (or velocity; none for a sensor-only joint)
//       <state_interface name="position"/> <state_interface name="velocity"/> <state_interface name="effort"/>
//       <param name="node_id">1</param> <param name="counts_per_unit">20861</param>
//       <param name="offset">0</param> <param name="rated">40</param>
//       <param name="mock_contact">0.12</param>   (mock bus: where the tool meets the ground, joint units)
//       <param name="mock_start">0.5</param>      (mock bus: where the drive starts, inside the joint's range)
//     </joint>
//   </ros2_control>
//
// On activate: NMT start, then the CiA 402 state machine is walked to "operation enabled" one
// controlword per cycle, with every position target held at the measured position so nothing
// jumps. Each write() sends one RPDO per drive and a SYNC; each read() folds in the TPDOs.
// Drive counts are 32-bit and wrap (a wheel encoder after ~1 km): positions are unwrapped here, and
// position targets go out relative to the latest count, so a continuous joint never jumps either.
// A drive in fault, or one that stops answering, makes read() return ERROR: the controller
// manager then deactivates the whole system, every drive gets "switch on" (brakes hold) and the
// machine stops. A fault is reset only by deactivating and activating again (an operator action).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "fs_hardware/can_bus.hpp"
#include "fs_hardware/cia402.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class CanopenSystem : public hardware_interface::SystemInterface
{
  struct Joint
  {
    std::string name;
    uint8_t node{0};
    fs_hw::Scale scale;
    fs_hw::Mode mode{fs_hw::Mode::CyclicPosition};
    bool commanded{false};
    fs_hw::Feedback fb;
    double lastSeen{0};
    double hold{0};       // position target while enabling and when a command is NaN
    int32_t lastRaw{0};
    double counts{0};     // unwrapped position, counts
    bool started{false};

    void Unwrap()
    {
      if (!started) { counts = fb.position; started = true; }
      else counts += static_cast<int32_t>(static_cast<uint32_t>(fb.position) - static_cast<uint32_t>(lastRaw));
      lastRaw = fb.position;
    }
    double Position() const { return counts / scale.countsPerUnit + scale.offset; }
    /// Raw target count for joint position p, relative to the last count (wrap-safe).
    int32_t Target(double p) const
    {
      const double d = std::round((p - Position()) * scale.countsPerUnit);
      return static_cast<int32_t>(static_cast<uint32_t>(lastRaw) + static_cast<uint32_t>(static_cast<int32_t>(
        std::clamp(d, -2.0e9, 2.0e9))));
    }
  };

public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    const auto &hp = info_.hardware_parameters;
    auto get = [](const std::unordered_map<std::string, std::string> &m, const std::string &k, const std::string &d) {
      auto it = m.find(k);
      return it == m.end() ? d : it->second;
    };
    busName_ = get(hp, "bus", "can0");
    cycle_ = std::stod(get(hp, "cycle_s", "0.01"));
    timeout_ = std::stod(get(hp, "timeout_s", "0.1"));
    for (const auto &j : info_.joints)
    {
      Joint J;
      J.name = j.name;
      J.node = static_cast<uint8_t>(std::stoi(get(j.parameters, "node_id", "0")));
      J.scale.countsPerUnit = std::stod(get(j.parameters, "counts_per_unit", "1"));
      J.scale.offset = std::stod(get(j.parameters, "offset", "0"));
      J.scale.rated = std::stod(get(j.parameters, "rated", "1"));
      if (J.node == 0 || J.node > 127)
      {
        RCLCPP_ERROR(get_logger(), "joint %s: node_id must be 1..127", j.name.c_str());
        return CallbackReturn::ERROR;
      }
      for (const auto &ci : j.command_interfaces)
      {
        J.commanded = true;
        J.mode = ci.name == hardware_interface::HW_IF_VELOCITY ? fs_hw::Mode::CyclicVelocity : fs_hw::Mode::CyclicPosition;
      }
      mockContact_.push_back(std::stod(get(j.parameters, "mock_contact", "nan")));
      mockStart_.push_back(std::stod(get(j.parameters, "mock_start", "0")));
      joints_.push_back(J);
    }
    RCLCPP_INFO(get_logger(), "%zu CiA 402 drives on %s", joints_.size(), busName_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try
    {
      if (busName_ == "mock")
      {
        auto mock = std::make_unique<fs_hw::MockBus>(cycle_);
        for (size_t k = 0; k < joints_.size(); ++k)
        {
          auto &d = mock->Drive(joints_[k].node);
          d.maxVel = 5.0 * joints_[k].scale.countsPerUnit;          // 5 m/s or rad/s
          d.pos = d.target = joints_[k].scale.ToCounts(mockStart_[k]);
          if (std::isfinite(mockContact_[k]))
          {
            d.contact = joints_[k].scale.ToCounts(mockContact_[k]);
            d.contactStiffness = 2000.0 / joints_[k].scale.countsPerUnit;   // full load 0.5 m past contact
          }
        }
        bus_ = std::move(mock);
      }
      else
        bus_ = std::make_unique<fs_hw::SocketCanBus>(busName_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    bus_->Send(fs_hw::Nmt(fs_hw::kNmtStart));
    // first feedback, so every target starts at the measured position
    for (auto &j : joints_)
    {
      j.fb = fs_hw::Feedback{};
      bus_->Send(fs_hw::Rpdo(j.node, fs_hw::kCwShutdown, 0, j.mode));
    }
    bus_->Send(fs_hw::Sync());
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(300))
    {
      Drain();
      bool all = true;
      for (auto &j : joints_) all = all && j.fb.tpdo1;
      if (all) break;
    }
    for (auto &j : joints_)
    {
      if (!j.fb.tpdo1)
      {
        RCLCPP_ERROR(get_logger(), "drive %u (%s) does not answer", j.node, j.name.c_str());
        return CallbackReturn::ERROR;
      }
      j.started = false;
      j.Unwrap();
      j.hold = j.Position();
      j.lastSeen = now_;
      if (j.commanded)
        set_command(j.name + "/" + (fs_hw::IsVelocityMode(j.mode) ? hardware_interface::HW_IF_VELOCITY
                                                                   : hardware_interface::HW_IF_POSITION),
                    fs_hw::IsVelocityMode(j.mode) ? 0.0 : j.hold);
    }
    enable_ = true;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    enable_ = false;
    for (auto &j : joints_)   // switched on: power stage off, brakes hold
      bus_->Send(fs_hw::Rpdo(j.node, fs_hw::kCwSwitchOn, fs_hw::IsVelocityMode(j.mode) ? 0 : j.lastRaw, j.mode));
    bus_->Send(fs_hw::Sync());
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    now_ += period.seconds() > 0 ? period.seconds() : cycle_;
    Drain();
    for (auto &j : joints_)
    {
      if (j.fb.tpdo1 || j.fb.tpdo2) j.lastSeen = now_;
      if (j.fb.tpdo1) j.Unwrap();
      j.fb.tpdo1 = j.fb.tpdo2 = false;
      if (now_ - j.lastSeen > timeout_)
      {
        RCLCPP_ERROR(get_logger(), "drive %u (%s): no feedback for %.0f ms", j.node, j.name.c_str(), (now_ - j.lastSeen) * 1e3);
        return return_type::ERROR;
      }
      const auto s = fs_hw::Decode(j.fb.status);
      if (s == fs_hw::State::Fault || s == fs_hw::State::FaultReaction)
      {
        RCLCPP_ERROR(get_logger(), "drive %u (%s) in fault", j.node, j.name.c_str());
        return return_type::ERROR;
      }
      set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, j.Position());
      set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, j.scale.VelFromCounts(j.fb.velocity));
      set_state(j.name + "/" + hardware_interface::HW_IF_EFFORT, j.scale.Effort(j.fb.torque));
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto &j : joints_)
    {
      const auto s = fs_hw::Decode(j.fb.status);
      const uint16_t cw = fs_hw::NextControlword(s, enable_ && j.commanded, false);
      int32_t target = 0;
      if (fs_hw::IsVelocityMode(j.mode))
      {
        const double v = j.commanded ? get_command<double>(j.name + "/" + hardware_interface::HW_IF_VELOCITY) : 0.0;
        target = s == fs_hw::State::OperationEnabled && std::isfinite(v) ? j.scale.VelToCounts(v) : 0;
      }
      else
      {
        const double p = j.commanded ? get_command<double>(j.name + "/" + hardware_interface::HW_IF_POSITION) : j.hold;
        // until enabled, and whenever no command has arrived, hold where the joint is
        if (s != fs_hw::State::OperationEnabled || !std::isfinite(p)) j.hold = j.Position();
        target = j.Target(s == fs_hw::State::OperationEnabled && std::isfinite(p) ? p : j.hold);
      }
      bus_->Send(fs_hw::Rpdo(j.node, cw, target, j.mode));
    }
    bus_->Send(fs_hw::Sync());
    return return_type::OK;
  }

private:
  void Drain()
  {
    fs_hw::CanFrame f;
    while (bus_->Receive(f))
      for (auto &j : joints_)
        if (fs_hw::Parse(f, j.node, j.fb)) break;
  }

  std::string busName_;
  double cycle_{0.01}, timeout_{0.1}, now_{0};
  bool enable_{false};
  std::vector<Joint> joints_;
  std::vector<double> mockContact_, mockStart_;
  std::unique_ptr<fs_hw::CanBus> bus_;
};

}  // namespace fs_hardware

PLUGINLIB_EXPORT_CLASS(fs_hardware::CanopenSystem, hardware_interface::SystemInterface)
