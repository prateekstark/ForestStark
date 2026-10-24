// Unit tests for the soil core: checks against hand calculations from the Bekker/Wong
// formulas, plus the qualitative behaviour the simulators rely on.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include "fs_gazebo/SoilMap.hh"
#include "fs_gazebo/SoilModel.hh"

using namespace fs::soil;

static SoilParams Soil(const std::string &name)
{
  for (const auto &s : DefaultSoils()) if (s.name == name) return s;
  return DefaultSoils()[1];
}

TEST(Bekker, SinkageMatchesHandCalculation)
{
  // Dry sand, one Pahadi track segment: b = 0.28 m, l = 1.1 m, W = 3 kN.
  const auto s = Soil("dry_sand");
  const double p = 3000.0 / (0.28 * 1.1);
  const double expected = std::pow(p / ((0.99 / 0.28 + 1528.43) * 1e3), 1.0 / 1.1);
  EXPECT_NEAR(Sinkage(s, 3000.0, 0.28, 1.1), expected, 1e-9);
  EXPECT_NEAR(expected, 0.0101, 0.0005);  // about a centimetre
}

TEST(Bekker, SinkageGrowsWithLoadAndShrinksWithArea)
{
  const auto s = Soil("sandy_loam");
  EXPECT_LT(Sinkage(s, 2000, 0.3, 1.0), Sinkage(s, 4000, 0.3, 1.0));
  EXPECT_GT(Sinkage(s, 4000, 0.2, 0.5), Sinkage(s, 4000, 0.3, 1.0));
  EXPECT_EQ(Sinkage(s, 0.0, 0.3, 1.0), 0.0);
}

TEST(Bekker, CompactionResistanceSmallOnFirmSoil)
{
  const auto loam = Soil("sandy_loam"), sand = Soil("dry_sand");
  const double zl = Sinkage(loam, 3000, 0.28, 1.1), zs = Sinkage(sand, 3000, 0.28, 1.1);
  EXPECT_LT(CompactionResistance(loam, zl, 0.28), CompactionResistance(sand, zs, 0.28));
  EXPECT_LT(CompactionResistance(sand, zs, 0.28), 0.05 * 3000);   // well under 5 % of load
}

TEST(Shear, CapacityIsMohrCoulomb)
{
  const auto s = Soil("dry_sand");
  const double A = 0.28 * 1.1;
  EXPECT_NEAR(ShearCapacity(s, 3000, 0.28, 1.1), A * 1.04e3 + 3000 * std::tan(28.0 * kPi / 180), 1e-6);
}

TEST(Shear, JanosiHanamotoMonotoneAndBounded)
{
  double prev = 0;
  for (double i = 0.0; i <= 1.0; i += 0.05)
  {
    const double f = ShearFraction(i, 1.1, 0.025);
    EXPECT_GE(f, prev - 1e-12);
    EXPECT_LE(f, 1.0);
    prev = f;
  }
  EXPECT_GT(ShearFraction(0.2, 1.1, 0.025), 0.85);   // long track: most shear at modest slip
  EXPECT_LT(ShearFraction(0.2, 0.15, 0.025), 0.6);   // short tyre patch needs more slip
}

TEST(Bristle, HoldsStillThenSlidesAtCapacity)
{
  Bristle b;
  const double g = 1000.0, s0 = g / 0.005, s1 = 2000.0, dt = 0.001;
  // Creep slowly: force builds up to the capacity and saturates there.
  double f = 0;
  for (int k = 0; k < 2000; ++k) f = b.Step(0.02, g, s0, s1, dt);
  EXPECT_NEAR(f, g, 0.02 * g);
  // Stop: the bristle keeps its deflection, so the soil keeps holding.
  for (int k = 0; k < 500; ++k) f = b.Step(0.0, g, s0, s1, dt);
  EXPECT_NEAR(f, g, 0.02 * g);
}

TEST(Load, EqualSharingOnFlatSymmetricLayout)
{
  std::vector<Vec3> c = {{1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}};
  auto N = DistributeLoad(c, {true, true, true, true}, {0, 0, 0.5}, 1000);
  for (double n : N) EXPECT_NEAR(n, 1000 * kG / 4, 1e-6);
}

TEST(Load, CgOffsetLoadsNearSideAndUnloadsFarSide)
{
  std::vector<Vec3> c = {{1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}};
  auto N = DistributeLoad(c, {true, true, true, true}, {0, 0.5, 0.5}, 1000);
  EXPECT_GT(N[0], N[1]);
  double sum = 0;
  for (double n : N) { EXPECT_GE(n, 0); sum += n; }
  EXPECT_NEAR(sum, 1000 * kG, 1e-6);
}

TEST(Load, CgOutsideSupportDropsLiftedContacts)
{
  std::vector<Vec3> c = {{1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}};
  auto N = DistributeLoad(c, {true, true, true, true}, {0, 1.4, 0.5}, 1000);
  EXPECT_EQ(N[1], 0.0);
  EXPECT_EQ(N[3], 0.0);
}

TEST(Auger, HardSoilIsSlowerAndThrustLimited)
{
  AugerParams p;
  p.diameter = 0.12; p.maxPower = 2000; p.maxThrust = 2500;
  const auto loam = BoreRate(p, Soil("sandy_loam").eb * 1e6);
  const auto lat = BoreRate(p, Soil("lateritic_gravel").eb * 1e6);
  EXPECT_GT(loam.feed, lat.feed);
  EXPECT_LE(lat.thrust, p.maxThrust + 1e-9);
  EXPECT_LE(loam.power, p.maxPower + 1e-6);
  // 25 cm in sandy loam at 2 kW: a few seconds, as on the concept sheet.
  EXPECT_GT(0.25 / loam.feed, 1.0);
  EXPECT_LT(0.25 / loam.feed, 10.0);
}

TEST(Auger, MoistureMakesDrySoilHarder)
{
  EXPECT_GT(MoistureFactor(0.05), MoistureFactor(0.30));
  EXPECT_GE(MoistureFactor(0.6), 0.7);
}

TEST(SoilMap, LoadsAndInterpolates)
{
  const std::string path = std::string(::testing::TempDir()) + "/fs_test.soilmap";
  {
    std::ofstream f(path);
    f << "# fs_soilmap v1\nname t\norigin 0 0\nresolution 1\nsize 3 2\n"
         "soil 0 dry_sand 1.1 0.99 1528.43 1.04 28 0.025 2.5\n"
         "soil 1 heavy_clay 0.13 12.7 1555.95 68.95 34 0.01 14\n"
         "layer elevation\n0 1 2\n0 1 2\nlayer soil\n0 0 1\n0 0 1\n"
         "layer rock_depth\n0.5 0.5 0.1\n0.5 0.5 0.1\nlayer moisture\n0.2 0.2 0.2\n0.2 0.2 0.2\n";
  }
  SoilMap m;
  m.Load(path);
  EXPECT_NEAR(m.Elevation(0.5, 0.5), 0.5, 1e-9);
  EXPECT_EQ(m.Sample(2, 0).soil->name, "heavy_clay");
  EXPECT_EQ(m.Sample(0, 0).soil->name, "dry_sand");
  const auto n = m.Normal(1.0, 0.5);
  EXPECT_LT(n.x, 0.0);   // ground rises with x, normal leans back
  EXPECT_NEAR(m.Sample(1.5, 1).rockDepth, 0.3, 1e-9);
}

TEST(Moisture, WetClayLosesTractionDampSandGainsSome)
{
  const auto clay = Soil("heavy_clay"), sand = Soil("dry_sand");
  const auto wet = AtMoisture(clay, 0.40), ref = AtMoisture(clay, clay.refMoisture);
  EXPECT_NEAR(ref.c, clay.c, 1e-9);
  EXPECT_LT(ShearCapacity(wet, 5000, 0.3, 0.3), 0.6 * ShearCapacity(ref, 5000, 0.3, 0.3));
  // sinkage grows as f^-1.5 whatever n, so a vertisol's tiny exponent cannot blow it up
  const double f = StrengthFactor(clay, 0.40);
  EXPECT_NEAR(Sinkage(wet, 5000, 0.3, 0.3) / Sinkage(ref, 5000, 0.3, 0.3), std::pow(f, -1.5), 1e-6);
  EXPECT_GT(AtMoisture(sand, 0.10).c, AtMoisture(sand, 0.01).c + 2.0);
  EXPECT_GT(AtMoisture(Soil("sandy_loam"), 0.05).c, Soil("sandy_loam").c);   // hard-set when dry
}

TEST(Moisture, MapsWithoutMoistureColumnsGetTheTableValues)
{
  SoilParams s; s.name = "heavy_clay"; s.wetSoftening = 0;
  FillMoisture(s);
  EXPECT_DOUBLE_EQ(s.wetSoftening, Soil("heavy_clay").wetSoftening);
}

TEST(HoleWall, BoneDrySandCavesWetSandStands)
{
  const auto sand = Soil("dry_sand");
  EXPECT_GT(HoleCollapse(sand, 0.03, 0.5), 0.15);             // no cohesion at all: half the pit fills
  EXPECT_EQ(HoleCollapse(sand, 0.11, 0.5), 0.0);              // after pre-watering
  EXPECT_EQ(HoleCollapse(Soil("sandy_loam"), 0.13, 0.55), 0.0);
  EXPECT_EQ(HoleCollapse(Soil("clayey_soil"), 0.19, 1.2), 0.0);   // CSSRI auger holes stand in sodic clay
  EXPECT_GT(HoleCollapse(Soil("clayey_soil"), 0.35, 1.2), 0.0);   // but not when waterlogged
}

TEST(Ruts, SecondPassOnlyPaysForExtraCompaction)
{
  const auto s = Soil("sandy_loam");
  const double z = Sinkage(s, 4000, 0.22, 0.25);
  const double first = MultiPassResistance(s, z, 0.0, 0.22);
  EXPECT_DOUBLE_EQ(first, CompactionResistance(s, z, 0.22));
  EXPECT_NEAR(MultiPassResistance(s, z, z, 0.22), 0.15 * first, 1e-9);
  const double deeper = MultiPassResistance(s, 1.5 * z, z, 0.22);
  EXPECT_GT(deeper, 0.15 * CompactionResistance(s, 1.5 * z, 0.22));
  EXPECT_LT(deeper, CompactionResistance(s, 1.5 * z, 0.22));

  RutMap m(0.1);
  m.Press(1.0, 2.0, 0.03, 0.3, 0.0, 1.0);      // tread across y
  EXPECT_FLOAT_EQ(m.Depth(1.0, 2.12), 0.03f);
  EXPECT_EQ(m.Depth(1.0, 2.4), 0.0);
  EXPECT_EQ(m.Depth(1.2, 2.0), 0.0);           // nothing ahead of or behind the line
  m.Press(1.0, 2.0, 0.01, 0.3);
  EXPECT_FLOAT_EQ(m.Depth(1.0, 2.0), 0.03f);   // keeps the deepest
}

TEST(Ruts, SaveAndLoadRoundTrip)
{
  const std::string path = std::string(::testing::TempDir()) + "/fs_test.ruts";
  RutMap a(0.1);
  a.Press(-3.04, 7.21, 0.04, 0.3);
  EXPECT_GT(a.Changes(), 0u);
  ASSERT_TRUE(a.Save(path));
  EXPECT_EQ(a.Changes(), 0u);
  RutMap b;
  ASSERT_TRUE(b.Load(path));
  EXPECT_EQ(b.Cells(), a.Cells());
  EXPECT_FLOAT_EQ(b.Depth(-3.04, 7.21), 0.04f);
  EXPECT_NEAR(b.MaxDepth(), 0.04, 1e-6);
}

TEST(SoilMap, ReadsCalibratedMoistureColumns)
{
  const std::string path = std::string(::testing::TempDir()) + "/fs_cal.soilmap";
  {
    std::ofstream f(path);
    f << "name t\norigin 0 0\nresolution 1\nsize 2 2\n"
         "soil 0 sandy_loam 0.9 52.53 1127.97 7.5 20 0.025 4 0.18 5.5 0.5 0.6\n"
         "soil 1 heavy_clay 0.13 12.7 1555.95 68.95 34 0.01 14\n"
         "layer elevation\n0 0\n0 0\nlayer soil\n0 1\n0 1\n";
  }
  SoilMap m;
  m.Load(path);
  const auto loam = *m.Sample(0, 0).soil, clay = *m.Sample(1, 0).soil;
  EXPECT_DOUBLE_EQ(loam.c, 7.5);
  EXPECT_DOUBLE_EQ(loam.refMoisture, 0.18);
  EXPECT_DOUBLE_EQ(loam.wetSoftening, 5.5);
  EXPECT_DOUBLE_EQ(clay.wetSoftening, Soil("heavy_clay").wetSoftening);   // no columns: the table's
}
