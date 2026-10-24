// CiA 402 state machine, PDO coding and the mock drive the desk tests run on.
#include <gtest/gtest.h>

#include "fs_hardware/can_bus.hpp"
#include "fs_hardware/cia402.hpp"

using namespace fs_hw;

TEST(Cia402, StatuswordsDecode)
{
  for (State s : {State::NotReady, State::SwitchOnDisabled, State::ReadyToSwitchOn, State::SwitchedOn,
                  State::OperationEnabled, State::QuickStop, State::FaultReaction, State::Fault})
    EXPECT_EQ(Decode(Encode(s)), s) << Name(s);
  EXPECT_EQ(Decode(0x0237), State::OperationEnabled);   // manufacturer / target-reached bits ignored
  EXPECT_EQ(Decode(0x0218), State::Fault);
}

TEST(Cia402, MasterWalksADriveToOperationEnabledAndBack)
{
  State s = State::SwitchOnDisabled;
  for (int k = 0; k < 5 && s != State::OperationEnabled; ++k) s = Transition(s, NextControlword(s, true, false));
  EXPECT_EQ(s, State::OperationEnabled);
  s = Transition(s, NextControlword(s, false, false));
  EXPECT_EQ(s, State::SwitchedOn);                       // disabled = switched on, brake holds
  EXPECT_EQ(Transition(State::OperationEnabled, kCwQuickStop), State::QuickStop);
  EXPECT_EQ(Transition(State::Fault, NextControlword(State::Fault, true, false)), State::Fault);   // never resets itself
  EXPECT_EQ(Transition(State::Fault, NextControlword(State::Fault, true, true)), State::SwitchOnDisabled);
}

TEST(Cia402, PdosRoundTrip)
{
  const auto p = Rpdo(5, kCwEnable, -123456, Mode::CyclicPosition);
  EXPECT_EQ(p.id, 0x205u);
  EXPECT_EQ(GetU16(p.data), kCwEnable);
  EXPECT_EQ(GetI32(p.data + 2), -123456);
  EXPECT_EQ(Rpdo(5, kCwEnable, 10, Mode::CyclicVelocity).id, 0x305u);
  CanFrame t; t.id = kTpdo1 + 5; t.len = 7; PutU16(t.data, 0x27); PutI32(t.data + 2, 777);
  Feedback fb;
  EXPECT_TRUE(Parse(t, 5, fb));
  EXPECT_FALSE(Parse(t, 6, fb));
  EXPECT_EQ(fb.position, 777);
  EXPECT_EQ(Decode(fb.status), State::OperationEnabled);
  Scale sc{100000.0, 0.25, 40.0};
  EXPECT_NEAR(sc.FromCounts(sc.ToCounts(0.731)), 0.731, 1e-5);
  EXPECT_NEAR(sc.Effort(500), 20.0, 1e-9);
}

TEST(MockBus, DriveFollowsPositionAndStallsOnContact)
{
  MockBus bus(0.01);
  auto &d = bus.Drive(3);
  d.maxVel = 1e5;
  bus.Send(Nmt(kNmtStart));
  State s = State::SwitchOnDisabled;
  CanFrame f;
  Feedback fb;
  for (int k = 0; k < 400; ++k)
  {
    bus.Send(Rpdo(3, NextControlword(s, true, false), 50000, Mode::CyclicPosition));
    bus.Send(Sync());
    while (bus.Receive(f)) Parse(f, 3, fb);
    s = Decode(fb.status);
  }
  EXPECT_EQ(s, State::OperationEnabled);
  EXPECT_NEAR(fb.position, 50000, 200);
  // a feed into the ground: velocity mode, contact at 60000 counts
  d.contact = 60000; d.contactStiffness = 0.05;
  int16_t torque = 0;
  for (int k = 0; k < 300; ++k)
  {
    bus.Send(Rpdo(3, kCwEnable, 20000, Mode::CyclicVelocity));
    bus.Send(Sync());
    while (bus.Receive(f)) Parse(f, 3, fb);
    torque = fb.torque;
  }
  EXPECT_GT(fb.position, 60000);
  EXPECT_LT(fb.velocity, 20000 * 0.7);   // slowed by the load
  EXPECT_GT(torque, 300);
}
