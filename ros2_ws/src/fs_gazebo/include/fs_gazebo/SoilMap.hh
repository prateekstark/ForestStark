// Site soil map: elevation, soil type, depth to rock and moisture on a regular grid.
//
// Text format (written by fs_gazebo/scripts/generate_sites.py, read here and by
// fs_autonomy.site.SoilMap):
//
//   # fs_soilmap v1
//   name hill_slope
//   origin -35.0 -20.0        x, y of cell (0, 0) centre in the world frame, m
//   resolution 0.5            m per cell
//   size 141 81               nx ny
//   soil <index> <name> <n> <kc> <kphi> <c_kPa> <phi_deg> <K_m> <eb_MJ_m3>
//   layer elevation           followed by ny lines of nx values (row j is y = y0 + j r)
//   layer soil                integer soil index
//   layer rock_depth          m of soil above rock or hardpan
//   layer moisture            volumetric fraction
#pragma once

#include <string>
#include <vector>

#include "fs_gazebo/SoilModel.hh"

namespace fs::soil
{

struct SoilSample
{
  const SoilParams *soil{nullptr};
  int soilIndex{0};
  double elevation{0.0};
  double rockDepth{1.0};
  double moisture{0.15};
};

class SoilMap
{
public:
  /// Loads a map file. Throws std::runtime_error with the reason when the file is unusable.
  void Load(const std::string &path);

  /// A uniform map, used when no file is given.
  void MakeUniform(const SoilParams &soil, double rockDepth = 1.0, double moisture = 0.15);

  bool Loaded() const { return loaded_; }
  const std::string &Name() const { return name_; }

  SoilSample Sample(double x, double y) const;
  double Elevation(double x, double y) const;
  /// Unit ground normal from the elevation gradient.
  Vec3 Normal(double x, double y) const;

  const std::vector<SoilParams> &Soils() const { return soils_; }

private:
  double Bilinear(const std::vector<float> &layer, double x, double y) const;
  int Nearest(const std::vector<float> &layer, double x, double y) const;

  bool loaded_{false};
  std::string name_{"uniform"};
  double x0_{0}, y0_{0}, res_{1.0};
  int nx_{1}, ny_{1};
  std::vector<SoilParams> soils_;
  std::vector<float> elevation_, soil_, rock_, moisture_;
};

}  // namespace fs::soil
