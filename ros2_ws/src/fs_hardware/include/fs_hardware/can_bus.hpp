// CAN transports for the CiA 402 drives: Linux SocketCAN for the machine, and a mock bus of
// simulated drives for the desk (the whole hardware stack runs, with controllers, on bus:=mock).
#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "fs_hardware/cia402.hpp"

namespace fs_hw
{

class CanBus
{
public:
  virtual ~CanBus() = default;
  virtual bool Send(const CanFrame &f) = 0;
  /// Non-blocking; false when no frame is waiting.
  virtual bool Receive(CanFrame &f) = 0;
};

class SocketCanBus : public CanBus
{
public:
  explicit SocketCanBus(const std::string &ifname)
  {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("cannot open a CAN socket: " + std::string(strerror(errno)));
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
    {
      ::close(fd_);
      throw std::runtime_error("no CAN interface '" + ifname + "' (ip link set " + ifname + " up type can bitrate 500000)");
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
      ::close(fd_);
      throw std::runtime_error("cannot bind to " + ifname + ": " + std::string(strerror(errno)));
    }
    ::fcntl(fd_, F_SETFL, O_NONBLOCK);
  }
  ~SocketCanBus() override { if (fd_ >= 0) ::close(fd_); }

  bool Send(const CanFrame &f) override
  {
    struct can_frame c {};
    c.can_id = f.id;
    c.can_dlc = f.len;
    std::memcpy(c.data, f.data, 8);
    return ::write(fd_, &c, sizeof(c)) == static_cast<ssize_t>(sizeof(c));
  }

  bool Receive(CanFrame &f) override
  {
    struct can_frame c {};
    if (::read(fd_, &c, sizeof(c)) != static_cast<ssize_t>(sizeof(c))) return false;
    f.id = c.can_id & CAN_SFF_MASK;
    f.len = c.can_dlc;
    std::memcpy(f.data, c.data, 8);
    return true;
  }

private:
  int fd_{-1};
};

/// One simulated CiA 402 drive: follows its state machine, tracks position or velocity targets
/// with a first-order lag and a speed limit, and reports torque from acceleration, friction and a
/// contact load past `contact` counts (a tool meeting the ground), so the auger driver can be
/// exercised on the desk.
struct MockDrive
{
  State state{State::SwitchOnDisabled};
  Mode mode{Mode::CyclicPosition};
  bool operational{false};
  double pos{0}, vel{0}, target{0}, maxVel{2e5}, tau{0.05};
  double contact{1e18}, contactStiffness{0.0};   // load per mille per count past contact
  int16_t torque{0};

  void Command(uint16_t cw, int32_t tgt, Mode m)
  {
    state = Transition(state, cw);
    mode = m;
    target = tgt;
  }

  void Step(double dt)
  {
    double v = 0;
    if (state == State::OperationEnabled)
    {
      v = IsVelocityMode(mode) ? target : std::clamp((target - pos) / std::max(tau, dt), -maxVel, maxVel);
      v = std::clamp(v, -maxVel, maxVel);
    }
    const double a = (v - vel) / std::max(dt, 1e-6);
    vel += (v - vel) * std::min(1.0, dt / tau);
    double load = 0;
    if (pos > contact && vel > 0)          // a tool pushing into the ground stalls against it
    {
      load = contactStiffness * (pos - contact);
      vel *= std::max(0.0, 1.0 - load / 1000.0);
    }
    pos += vel * dt;
    torque = static_cast<int16_t>(std::clamp(0.02 * a / std::max(maxVel, 1.0) * 1000 + 20 * (vel > 0) - 20 * (vel < 0) + load,
                                             -3000.0, 3000.0));
  }
};

class MockBus : public CanBus
{
public:
  explicit MockBus(double cycle = 0.01) : cycle_(cycle) {}
  MockDrive &Drive(uint8_t node) { return drives_[node]; }

  bool Send(const CanFrame &f) override
  {
    if (f.id == kNmt)
    {
      for (auto &[n, d] : drives_)
        if (f.data[1] == 0 || f.data[1] == n) d.operational = f.data[0] == kNmtStart;
    }
    else if (f.id == kSync)
    {
      for (auto &[n, d] : drives_)
      {
        if (!d.operational) continue;
        d.Step(cycle_);
        CanFrame t1; t1.id = kTpdo1 + n; t1.len = 7;
        PutU16(t1.data, Encode(d.state)); PutI32(t1.data + 2, static_cast<int32_t>(std::lround(d.pos)));
        t1.data[6] = static_cast<uint8_t>(d.mode);
        CanFrame t2; t2.id = kTpdo2 + n; t2.len = 6;
        PutI32(t2.data, static_cast<int32_t>(std::lround(d.vel)));
        PutU16(t2.data + 4, static_cast<uint16_t>(d.torque));
        rx_.push_back(t1);
        rx_.push_back(t2);
      }
    }
    else if ((f.id & 0x780) == kRpdo1 || (f.id & 0x780) == kRpdo2)
    {
      auto it = drives_.find(static_cast<uint8_t>(f.id & 0x7F));
      if (it != drives_.end() && it->second.operational)
      {
        const bool vel = (f.id & 0x780) == kRpdo2;
        it->second.Command(GetU16(f.data), GetI32(f.data + 2),
                           vel ? Mode::CyclicVelocity : static_cast<Mode>(static_cast<int8_t>(f.data[6])));
      }
    }
    return true;
  }

  bool Receive(CanFrame &f) override
  {
    if (rx_.empty()) return false;
    f = rx_.front();
    rx_.pop_front();
    return true;
  }

private:
  double cycle_;
  std::map<uint8_t, MockDrive> drives_;
  std::deque<CanFrame> rx_;
};

}  // namespace fs_hw
