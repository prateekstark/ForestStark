// CANopen CiA 402 drive profile: the part ForestStark's joint drives use.
//
// Pure logic with no ROS or socket dependency, so it is unit-tested on its own (test/test_cia402.cpp).
//
// Every joint is one drive (electric servo, or a proportional-valve controller for the hydraulic
// joints, which expose the same CiA 402 objects). The commissioning tool maps the PDOs once, the
// same way on every drive:
//
//   RPDO1 0x200+id  controlword u16, target position i32, mode of operation i8     (7 bytes)
//   RPDO2 0x300+id  controlword u16, target velocity i32                          (6 bytes)
//   TPDO1 0x180+id  statusword u16, position actual i32, mode display i8          (7 bytes, on SYNC)
//   TPDO2 0x280+id  velocity actual i32, torque actual i16 (per mille of rated)   (6 bytes, on SYNC)
//
// The master sends NMT start, then one RPDO per drive and a SYNC every control cycle; the drives
// answer each SYNC with both TPDOs. Positions and velocities are in drive counts; Scale converts.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace fs_hw
{

struct CanFrame
{
  uint32_t id{0};
  uint8_t len{0};
  uint8_t data[8]{};
};

constexpr uint32_t kNmt = 0x000, kSync = 0x080, kTpdo1 = 0x180, kRpdo1 = 0x200, kTpdo2 = 0x280, kRpdo2 = 0x300,
                   kHeartbeat = 0x700;
constexpr uint8_t kNmtStart = 0x01, kNmtStop = 0x02, kNmtPreOp = 0x80, kNmtReset = 0x81;

enum class Mode : int8_t { ProfilePosition = 1, ProfileVelocity = 3, CyclicPosition = 8, CyclicVelocity = 9 };

inline bool IsVelocityMode(Mode m) { return m == Mode::ProfileVelocity || m == Mode::CyclicVelocity; }

enum class State { NotReady, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, QuickStop, FaultReaction, Fault };

inline const char *Name(State s)
{
  switch (s)
  {
    case State::NotReady: return "not ready";
    case State::SwitchOnDisabled: return "switch on disabled";
    case State::ReadyToSwitchOn: return "ready to switch on";
    case State::SwitchedOn: return "switched on";
    case State::OperationEnabled: return "operation enabled";
    case State::QuickStop: return "quick stop";
    case State::FaultReaction: return "fault reaction";
    case State::Fault: return "fault";
  }
  return "?";
}

/// Drive state from the statusword (CiA 402 table: bits 0-3, 5, 6).
inline State Decode(uint16_t sw)
{
  if ((sw & 0x4F) == 0x00) return State::NotReady;
  if ((sw & 0x4F) == 0x40) return State::SwitchOnDisabled;
  if ((sw & 0x6F) == 0x21) return State::ReadyToSwitchOn;
  if ((sw & 0x6F) == 0x23) return State::SwitchedOn;
  if ((sw & 0x6F) == 0x27) return State::OperationEnabled;
  if ((sw & 0x6F) == 0x07) return State::QuickStop;
  if ((sw & 0x4F) == 0x0F) return State::FaultReaction;
  if ((sw & 0x4F) == 0x08) return State::Fault;
  return State::NotReady;
}

/// Statusword a drive in state s reports (used by the mock drive and the tests).
inline uint16_t Encode(State s)
{
  switch (s)
  {
    case State::NotReady: return 0x00;
    case State::SwitchOnDisabled: return 0x40;
    case State::ReadyToSwitchOn: return 0x21;
    case State::SwitchedOn: return 0x23;
    case State::OperationEnabled: return 0x27;
    case State::QuickStop: return 0x07;
    case State::FaultReaction: return 0x0F;
    case State::Fault: return 0x08;
  }
  return 0;
}

constexpr uint16_t kCwDisableVoltage = 0x00, kCwShutdown = 0x06, kCwSwitchOn = 0x07, kCwEnable = 0x0F,
                   kCwQuickStop = 0x02, kCwFaultReset = 0x80;

/// The controlword that moves a drive one step toward `enable` (operation enabled) or away from it
/// (switched on, holding its brake). A drive in fault gets a fault-reset edge only when asked to.
inline uint16_t NextControlword(State s, bool enable, bool faultReset)
{
  if (s == State::Fault || s == State::FaultReaction) return faultReset ? kCwFaultReset : kCwDisableVoltage;
  if (!enable) return s == State::OperationEnabled ? kCwSwitchOn : (s == State::SwitchOnDisabled ? kCwDisableVoltage : kCwShutdown);
  switch (s)
  {
    case State::SwitchOnDisabled: return kCwShutdown;
    case State::ReadyToSwitchOn: return kCwSwitchOn;
    case State::SwitchedOn:
    case State::OperationEnabled: return kCwEnable;
    case State::QuickStop: return kCwDisableVoltage;
    default: return kCwShutdown;
  }
}

/// Drive state after receiving controlword cw (the drive side of the state machine; mock and tests).
inline State Transition(State s, uint16_t cw)
{
  if (s == State::Fault) return (cw & 0x80) ? State::SwitchOnDisabled : State::Fault;
  if ((cw & 0x02) == 0) return State::SwitchOnDisabled;                          // disable voltage
  if ((cw & 0x04) == 0 && s == State::OperationEnabled) return State::QuickStop;  // quick stop
  switch (cw & 0x0F)
  {
    case kCwShutdown: return State::ReadyToSwitchOn;
    case kCwSwitchOn: return (s == State::ReadyToSwitchOn || s == State::OperationEnabled) ? State::SwitchedOn : s;
    case kCwEnable: return (s == State::SwitchedOn || s == State::OperationEnabled) ? State::OperationEnabled : s;
    default: return s;
  }
}

inline void PutU16(uint8_t *d, uint16_t v) { d[0] = v & 0xFF; d[1] = v >> 8; }
inline void PutI32(uint8_t *d, int32_t v) { uint32_t u; std::memcpy(&u, &v, 4); for (int i = 0; i < 4; ++i) d[i] = (u >> (8 * i)) & 0xFF; }
inline uint16_t GetU16(const uint8_t *d) { return static_cast<uint16_t>(d[0] | (d[1] << 8)); }
inline int32_t GetI32(const uint8_t *d)
{
  const uint32_t u = d[0] | (d[1] << 8) | (d[2] << 16) | (static_cast<uint32_t>(d[3]) << 24);
  int32_t v; std::memcpy(&v, &u, 4); return v;
}
inline int16_t GetI16(const uint8_t *d) { const uint16_t u = GetU16(d); int16_t v; std::memcpy(&v, &u, 2); return v; }

inline CanFrame Nmt(uint8_t cmd, uint8_t node = 0)
{
  CanFrame f; f.id = kNmt; f.len = 2; f.data[0] = cmd; f.data[1] = node; return f;
}

inline CanFrame Sync() { CanFrame f; f.id = kSync; f.len = 0; return f; }

/// Command PDO: RPDO1 (position modes) or RPDO2 (velocity modes).
inline CanFrame Rpdo(uint8_t node, uint16_t cw, int32_t target, Mode mode)
{
  CanFrame f;
  if (IsVelocityMode(mode))
  {
    f.id = kRpdo2 + node; f.len = 6;
    PutU16(f.data, cw); PutI32(f.data + 2, target);
  }
  else
  {
    f.id = kRpdo1 + node; f.len = 7;
    PutU16(f.data, cw); PutI32(f.data + 2, target); f.data[6] = static_cast<uint8_t>(mode);
  }
  return f;
}

struct Feedback
{
  uint16_t status{0};
  int32_t position{0};
  int32_t velocity{0};
  int16_t torque{0};     // per mille of rated torque / force
  bool tpdo1{false}, tpdo2{false};
  bool heartbeat{false};
  uint8_t nmtState{0};
};

/// Fold a frame from `node` into its feedback. Returns true if the frame belonged to that node.
inline bool Parse(const CanFrame &f, uint8_t node, Feedback &fb)
{
  if (f.id == kTpdo1 + node && f.len >= 7) { fb.status = GetU16(f.data); fb.position = GetI32(f.data + 2); fb.tpdo1 = true; return true; }
  if (f.id == kTpdo2 + node && f.len >= 6) { fb.velocity = GetI32(f.data); fb.torque = GetI16(f.data + 4); fb.tpdo2 = true; return true; }
  if (f.id == kHeartbeat + node && f.len >= 1) { fb.heartbeat = true; fb.nmtState = f.data[0]; return true; }
  return false;
}

/// Drive counts <-> SI (m, rad; N, N m).
struct Scale
{
  double countsPerUnit{1.0};
  double offset{0.0};        // joint position at count 0
  double rated{1.0};         // rated torque (N m) or force (N) the per-mille value refers to

  int32_t ToCounts(double v) const { return static_cast<int32_t>(std::lround((v - offset) * countsPerUnit)); }
  int32_t VelToCounts(double v) const { return static_cast<int32_t>(std::lround(v * countsPerUnit)); }
  double FromCounts(int32_t c) const { return c / countsPerUnit + offset; }
  double VelFromCounts(int32_t c) const { return c / countsPerUnit; }
  double Effort(int16_t permille) const { return permille * rated / 1000.0; }
};

}  // namespace fs_hw
