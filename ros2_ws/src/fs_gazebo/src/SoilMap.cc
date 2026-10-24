#include "fs_gazebo/SoilMap.hh"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fs::soil
{

void SoilMap::Load(const std::string &path)
{
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open soil map '" + path + "'");

  std::map<int, SoilParams> table;
  std::string line;
  int lineNo = 0;
  auto fail = [&](const std::string &why) {
    throw std::runtime_error(path + ":" + std::to_string(lineNo) + ": " + why);
  };
  auto readLayer = [&](std::vector<float> &dst) {
    dst.assign(static_cast<size_t>(nx_) * ny_, 0.0f);
    for (int j = 0; j < ny_; ++j)
    {
      if (!std::getline(in, line)) fail("layer ends early");
      ++lineNo;
      std::istringstream ls(line);
      for (int i = 0; i < nx_; ++i)
        if (!(ls >> dst[static_cast<size_t>(j) * nx_ + i])) fail("expected " + std::to_string(nx_) + " values");
    }
  };

  bool haveSize = false;
  while (std::getline(in, line))
  {
    ++lineNo;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key == "name") ls >> name_;
    else if (key == "origin") ls >> x0_ >> y0_;
    else if (key == "resolution") ls >> res_;
    else if (key == "size") { ls >> nx_ >> ny_; haveSize = nx_ > 1 && ny_ > 1; }
    else if (key == "soil")
    {
      int idx; SoilParams s;
      if (!(ls >> idx >> s.name >> s.n >> s.kc >> s.kphi >> s.c >> s.phiDeg >> s.K >> s.eb)) fail("bad soil line");
      // optional moisture response (refMoisture wetSoftening capillaryC trueCohesion); else the built-in table's
      double mr, ms, mc, mt;
      if (ls >> mr >> ms >> mc >> mt)
      {
        s.refMoisture = mr; s.wetSoftening = ms; s.capillaryC = mc; s.trueCohesion = mt;
      }
      else FillMoisture(s);
      table[idx] = s;
    }
    else if (key == "layer")
    {
      if (!haveSize) fail("layer before size");
      std::string layer; ls >> layer;
      if (layer == "elevation") readLayer(elevation_);
      else if (layer == "soil") readLayer(soil_);
      else if (layer == "rock_depth") readLayer(rock_);
      else if (layer == "moisture") readLayer(moisture_);
      else if (layer == "hardpan") readLayer(hardpan_);
      else { std::vector<float> skip; readLayer(skip); }
    }
  }
  if (!haveSize || res_ <= 0) throw std::runtime_error(path + ": missing size or resolution");
  if (elevation_.empty()) elevation_.assign(static_cast<size_t>(nx_) * ny_, 0.0f);
  if (soil_.empty()) soil_.assign(static_cast<size_t>(nx_) * ny_, 1.0f);
  if (rock_.empty()) rock_.assign(static_cast<size_t>(nx_) * ny_, 1.0f);
  if (moisture_.empty()) moisture_.assign(static_cast<size_t>(nx_) * ny_, 0.15f);
  if (hardpan_.empty()) hardpan_.assign(static_cast<size_t>(nx_) * ny_, 9.0f);

  soils_ = DefaultSoils();
  if (!table.empty())
  {
    soils_.assign(static_cast<size_t>(table.rbegin()->first) + 1, DefaultSoils()[1]);
    for (auto &[k, v] : table) soils_[static_cast<size_t>(k)] = v;
  }
  loaded_ = true;
}

void SoilMap::MakeUniform(const SoilParams &soil, double rockDepth, double moisture)
{
  name_ = "uniform";
  x0_ = -1e4; y0_ = -1e4; res_ = 2e4; nx_ = 2; ny_ = 2;
  soils_ = {soil};
  elevation_.assign(4, 0.0f);
  soil_.assign(4, 0.0f);
  rock_.assign(4, static_cast<float>(rockDepth));
  moisture_.assign(4, static_cast<float>(moisture));
  loaded_ = true;
}

double SoilMap::Bilinear(const std::vector<float> &layer, double x, double y) const
{
  const double fx = std::clamp((x - x0_) / res_, 0.0, nx_ - 1.0);
  const double fy = std::clamp((y - y0_) / res_, 0.0, ny_ - 1.0);
  const int i = std::min(static_cast<int>(fx), nx_ - 2);
  const int j = std::min(static_cast<int>(fy), ny_ - 2);
  const double u = fx - i, v = fy - j;
  auto at = [&](int a, int b) { return static_cast<double>(layer[static_cast<size_t>(b) * nx_ + a]); };
  return (1 - u) * (1 - v) * at(i, j) + u * (1 - v) * at(i + 1, j) + (1 - u) * v * at(i, j + 1) + u * v * at(i + 1, j + 1);
}

int SoilMap::Nearest(const std::vector<float> &layer, double x, double y) const
{
  const int i = std::clamp(static_cast<int>(std::lround((x - x0_) / res_)), 0, nx_ - 1);
  const int j = std::clamp(static_cast<int>(std::lround((y - y0_) / res_)), 0, ny_ - 1);
  return static_cast<int>(layer[static_cast<size_t>(j) * nx_ + i]);
}

SoilSample SoilMap::Sample(double x, double y) const
{
  SoilSample s;
  if (!loaded_) return s;
  s.soilIndex = std::clamp(Nearest(soil_, x, y), 0, static_cast<int>(soils_.size()) - 1);
  s.soil = &soils_[static_cast<size_t>(s.soilIndex)];
  s.elevation = Bilinear(elevation_, x, y);
  s.rockDepth = std::max(0.0, Bilinear(rock_, x, y));
  s.moisture = std::clamp(Bilinear(moisture_, x, y), 0.0, 0.6);
  s.hardpan = std::max(0.0, Bilinear(hardpan_, x, y));
  return s;
}

double SoilMap::Elevation(double x, double y) const
{
  return loaded_ ? Bilinear(elevation_, x, y) : 0.0;
}

Vec3 SoilMap::Normal(double x, double y) const
{
  const double h = std::max(res_, 0.05);
  const double dzdx = (Elevation(x + h, y) - Elevation(x - h, y)) / (2 * h);
  const double dzdy = (Elevation(x, y + h) - Elevation(x, y - h)) / (2 * h);
  return Vec3{-dzdx, -dzdy, 1.0}.Normalized();
}

}  // namespace fs::soil
