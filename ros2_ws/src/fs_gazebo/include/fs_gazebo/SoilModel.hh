// Soil dynamics used by the ForestStark Gazebo systems.
//
// Pure maths with no Gazebo dependency, so it is unit-tested on its own (test/test_soil_model.cpp)
// and mirrored in Python (fs_autonomy/fs_autonomy/terramechanics.py).
//
//  * Bekker pressure-sinkage            p = (kc/b + kphi) z^n
//  * Compaction (motion) resistance     Rc = b (kc/b + kphi) z^(n+1) / (n+1)
//  * Mohr-Coulomb shear capacity        Fmax = A c + W tan(phi)
//  * Janosi-Hanamoto shear development  F(i) = Fmax [1 - K/(i l) (1 - e^(-i l / K))]
//  * LuGre-style bristle state so a wheel or track can stand still on a slope without creeping
//  * Quasi-static normal-load distribution over the ground contacts (least-norm, non-negative)
//  * Auger cutting: specific boring energy -> power- and thrust-limited feed rate
//
// Units are SI throughout. Bekker constants are stored as published (kN based) and converted here.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace fs::soil
{

constexpr double kG = 9.81;
constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double Dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 Cross(const Vec3 &o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double Length() const { return std::sqrt(Dot(*this)); }
  Vec3 Normalized() const { const double l = Length(); return l > 1e-12 ? *this * (1.0 / l) : Vec3{0, 0, 1}; }
};

/// One soil type. Bekker constants as tabulated by Wong (Theory of Ground Vehicles).
struct SoilParams
{
  std::string name{"sandy_loam"};
  double n{0.9};          ///< sinkage exponent
  double kc{52.53};       ///< cohesive modulus, kN/m^(n+1)
  double kphi{1127.97};   ///< frictional modulus, kN/m^(n+2)
  double c{4.83};         ///< cohesion, kPa
  double phiDeg{20.0};    ///< internal friction angle, deg
  double K{0.025};        ///< shear deformation modulus, m
  double eb{4.0};         ///< specific boring energy at 15 % moisture, MJ/m^3

  /// Pressure-sinkage modulus for a contact of width b, in Pa/m^n.
  double Modulus(double b) const { return (kc / std::max(b, 1e-3) + kphi) * 1e3; }
  double TanPhi() const { return std::tan(phiDeg * kPi / 180.0); }
};

/// Built-in table, used when a soil map does not define its own.
inline std::vector<SoilParams> DefaultSoils()
{
  return {
    {"dry_sand",        1.10,  0.99, 1528.43,  1.04, 28.0, 0.025,  2.5},
    {"sandy_loam",      0.90, 52.53, 1127.97,  4.83, 20.0, 0.025,  4.0},
    {"clayey_soil",     0.50, 13.19,  692.15,  4.14, 13.0, 0.020,  9.0},
    {"heavy_clay",      0.13, 12.70, 1555.95, 68.95, 34.0, 0.010, 14.0},
    {"lateritic_gravel", 0.80, 30.00, 4000.00, 10.00, 35.0, 0.015, 22.0},
  };
}

/// Boring energy multiplier for moisture (volumetric fraction 0..0.45). Dry, hard-set soil
/// takes more energy to cut; wet soil less, down to a floor.
inline double MoistureFactor(double moisture)
{
  return std::clamp(1.6 - 2.4 * moisture, 0.7, 1.6);
}

/// Static sinkage z (m) of a rectangular contact b x l carrying normal load W (N).
inline double Sinkage(const SoilParams &s, double W, double b, double l)
{
  if (W <= 0.0) return 0.0;
  const double p = W / (std::max(b, 1e-3) * std::max(l, 1e-3));
  return std::pow(p / s.Modulus(b), 1.0 / s.n);
}

/// Compaction resistance (N) of one contact of width b at sinkage z.
inline double CompactionResistance(const SoilParams &s, double z, double b)
{
  if (z <= 0.0) return 0.0;
  return b * s.Modulus(b) * std::pow(z, s.n + 1.0) / (s.n + 1.0);
}

/// Maximum soil shear force (N) under a contact b x l with normal load W.
inline double ShearCapacity(const SoilParams &s, double W, double b, double l)
{
  return std::max(0.0, b * l * s.c * 1e3 + std::max(W, 0.0) * s.TanPhi());
}

/// Janosi-Hanamoto: fraction of the shear capacity developed at slip i over contact length l.
inline double ShearFraction(double i, double l, double K)
{
  i = std::clamp(std::fabs(i), 0.0, 1.0);
  const double a = i * l / std::max(K, 1e-4);
  if (a < 1e-6) return 0.5 * a;
  return 1.0 - (1.0 - std::exp(-a)) / a;
}

/// Longitudinal slip ratio. Positive while driving (wheel surface faster than the ground),
/// negative while braking or skidding.
inline double SlipRatio(double groundSpeed, double surfaceSpeed, double eps = 0.05)
{
  const double den = std::max({std::fabs(groundSpeed), std::fabs(surfaceSpeed), eps});
  return std::clamp((surfaceSpeed - groundSpeed) / den, -1.0, 1.0);
}

/// Bristle (LuGre-type) shear state for one direction of one contact.
/// Steady state force equals g(v) * sign(v); at v = 0 the bristle holds its deflection, so a
/// stopped machine stays put on a slope as long as the load stays inside the shear capacity.
struct Bristle
{
  double z{0.0};

  /// v: slip velocity of the contact relative to the soil (m/s); g: force the soil can
  /// develop at this slip (N); sigma0 (N/m), sigma1 (N s/m). Returns the soil force opposing v.
  double Step(double v, double g, double sigma0, double sigma1, double dt)
  {
    g = std::max(g, 1e-6);
    const double zOld = z;
    const double rate = sigma0 * std::fabs(v) / g;
    const double zSs = (v >= 0 ? 1.0 : -1.0) * g / sigma0;
    if (rate * dt > 1e-9)
      z = zSs + (z - zSs) * std::exp(-rate * dt);
    // Never let a stale deflection exceed what the soil can carry now.
    z = std::clamp(z, -g / sigma0, g / sigma0);
    const double zDot = (z - zOld) / std::max(dt, 1e-9);
    return sigma0 * z + sigma1 * zDot;
  }
};

/// Quasi-static distribution of a body's weight over ground contact points.
/// Vertical force balance plus zero moment about the centre of gravity (horizontal axes),
/// least-norm solution, contacts that would pull are dropped and the rest re-solved.
/// Returns the vertical load on each contact in N (0 for contacts flagged out of touch).
inline std::vector<double> DistributeLoad(const std::vector<Vec3> &contacts,
                                          const std::vector<bool> &touching,
                                          const Vec3 &cg, double mass)
{
  const size_t n = contacts.size();
  std::vector<double> N(n, 0.0);
  std::vector<bool> active(n);
  for (size_t i = 0; i < n; ++i) active[i] = i < touching.size() ? touching[i] : true;
  const double W = mass * kG;

  for (int iter = 0; iter < static_cast<int>(n) + 1; ++iter)
  {
    std::vector<size_t> idx;
    for (size_t i = 0; i < n; ++i) if (active[i]) idx.push_back(i);
    if (idx.empty()) return N;
    // A = rows [1, dx, dy]; solve (A A^T) lambda = b, N = A^T lambda.
    double M[3][3] = {{0}};
    for (size_t k : idx)
    {
      const double r[3] = {1.0, contacts[k].x - cg.x, contacts[k].y - cg.y};
      for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) M[a][b] += r[a] * r[b];
    }
    const double det =
      M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    std::fill(N.begin(), N.end(), 0.0);
    if (std::fabs(det) < 1e-9 * std::max(1.0, M[1][1] * M[2][2]))
    {
      // Collinear or single contact: the body is balancing on a line; share equally.
      for (size_t k : idx) N[k] = W / idx.size();
      return N;
    }
    // lambda = M^-1 [W, 0, 0]^T  -> first column of the inverse times W.
    const double inv0 = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
    const double inv1 = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) / det;
    const double inv2 = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
    bool negative = false;
    for (size_t k : idx)
    {
      N[k] = W * (inv0 + inv1 * (contacts[k].x - cg.x) + inv2 * (contacts[k].y - cg.y));
      if (N[k] < 0.0) negative = true;
    }
    if (!negative) return N;
    // Drop the most negative contact and solve again.
    size_t worst = idx[0];
    for (size_t k : idx) if (N[k] < N[worst]) worst = k;
    active[worst] = false;
  }
  return N;
}

/// Auger head limits.
struct AugerParams
{
  double diameter{0.12};     ///< m
  double maxPower{4000.0};   ///< W at the bit
  double efficiency{0.8};    ///< hydraulic/electric to cutting
  double maxThrust{3000.0};  ///< N the feed actuator and machine weight can push
  double thrustCoeff{0.012}; ///< thrust needed per unit of cutting energy per metre (dimensionless)
  double maxFeed{0.15};      ///< m/s, flight geometry limit
  double rpm{120.0};
};

struct AugerRate
{
  double feed{0};    ///< m/s
  double thrust{0};  ///< N
  double power{0};   ///< W drawn
  double torque{0};  ///< N m at the bit
  bool thrustLimited{false};
  bool powerLimited{false};
};

/// Feed rate through a material with specific cutting energy e (J/m^3).
inline AugerRate BoreRate(const AugerParams &p, double e)
{
  AugerRate r;
  const double A = kPi * p.diameter * p.diameter / 4.0;
  const double energyPerMetre = std::max(e, 1.0) * A;   // J/m = N
  const double feedPower = p.efficiency * p.maxPower / energyPerMetre;
  r.feed = std::min(p.maxFeed, feedPower);
  r.powerLimited = feedPower < p.maxFeed;
  r.thrust = p.thrustCoeff * energyPerMetre;
  if (r.thrust > p.maxThrust)
  {
    r.feed *= p.maxThrust / r.thrust;
    r.thrust = p.maxThrust;
    r.thrustLimited = true;
  }
  r.power = energyPerMetre * r.feed / p.efficiency;
  const double omega = std::max(p.rpm, 1.0) * 2.0 * kPi / 60.0;
  r.torque = r.power * p.efficiency / omega;
  return r;
}

/// Energy (J) to bore a hole of depth d through soil with specific energy e (J/m^3).
inline double BoreEnergy(const AugerParams &p, double e, double d)
{
  const double A = kPi * p.diameter * p.diameter / 4.0;
  return e * A * d / p.efficiency;
}

}  // namespace fs::soil
