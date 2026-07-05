#ifndef MOBILE_MANIPULATOR_FKIE_NBV__NBV_GRID_HPP_
#define MOBILE_MANIPULATOR_FKIE_NBV__NBV_GRID_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace mobile_manipulator_fkie_nbv
{

struct MeasurementValue
{
  double mean = 0.0;
  double variance = 0.0;

  MeasurementValue() = default;
  MeasurementValue(const double mean_in, const double variance_in)
  : mean(mean_in), variance(variance_in) {}

  double get_value() const {return mean;}

  MeasurementValue operator-(const MeasurementValue & other) const
  {
    return MeasurementValue(mean - other.mean, variance - other.variance);
  }
};

struct VisitedValue
{
  bool visited = false;

  VisitedValue() = default;
  explicit VisitedValue(const bool visited_in) : visited(visited_in) {}

  double get_value() const {return visited ? 1.0 : 0.0;}
};

struct PositionGrid
{
  double x = std::numeric_limits<double>::max();
  double y = std::numeric_limits<double>::max();
  double z = 0.0;

  PositionGrid() = default;
  PositionGrid(const double x_in, const double y_in, const double z_in = 0.0)
  : x(x_in), y(y_in), z(z_in) {}

  explicit PositionGrid(const geometry_msgs::msg::Pose & pose)
  : x(pose.position.x), y(pose.position.y), z(pose.position.z) {}

  explicit PositionGrid(const geometry_msgs::msg::Point & point)
  : x(point.x), y(point.y), z(point.z) {}

  bool is_valid() const
  {
    return std::abs(x) < 2147483000.0 && std::abs(y) < 2147483000.0 &&
           std::abs(z) < 2147483000.0;
  }
};

struct IndexGrid
{
  int x = std::numeric_limits<int>::max();
  int y = std::numeric_limits<int>::max();
  int z = std::numeric_limits<int>::max();

  IndexGrid() = default;
  IndexGrid(const int x_in, const int y_in, const int z_in)
  : x(x_in), y(y_in), z(z_in) {}

  bool operator==(const IndexGrid & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool is_valid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000 &&
           std::abs(z) < 2147483000;
  }

  static IndexGrid position_to_index(const PositionGrid & position, const double grid_size)
  {
    return IndexGrid(
      static_cast<int>(std::round(position.x / grid_size)),
      static_cast<int>(std::round(position.y / grid_size)),
      static_cast<int>(std::round(position.z / grid_size)));
  }

  static PositionGrid index_to_position(const IndexGrid & index, const double grid_size)
  {
    return PositionGrid(index.x * grid_size, index.y * grid_size, index.z * grid_size);
  }
};

struct IndexGridHasher
{
  std::size_t operator()(const IndexGrid & key) const
  {
    return ((std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1)) >> 1) ^
           (std::hash<int>()(key.z) << 1);
  }
};

struct IndexGrid2D
{
  int x = std::numeric_limits<int>::max();
  int y = std::numeric_limits<int>::max();

  IndexGrid2D() = default;
  IndexGrid2D(const int x_in, const int y_in) : x(x_in), y(y_in) {}

  bool operator==(const IndexGrid2D & other) const
  {
    return x == other.x && y == other.y;
  }

  bool is_valid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000;
  }

  static IndexGrid2D position_to_index(const PositionGrid & position, const double grid_size)
  {
    return IndexGrid2D(
      static_cast<int>(std::round(position.x / grid_size)),
      static_cast<int>(std::round(position.y / grid_size)));
  }

  static PositionGrid index_to_position(const IndexGrid2D & index, const double grid_size)
  {
    return PositionGrid(index.x * grid_size, index.y * grid_size, 0.0);
  }
};

struct IndexGrid2DHasher
{
  std::size_t operator()(const IndexGrid2D & key) const
  {
    return ((std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1)) >> 1);
  }
};

template<typename Container, typename Index = IndexGrid, typename IndexHasher = IndexGridHasher>
class SparseGrid
{
public:
  explicit SparseGrid(std::string name = "grid") : name_(std::move(name)) {}

  void set_grid_size(const double grid_size)
  {
    grid_size_ = grid_size;
    clear();
  }

  void clear()
  {
    data_.clear();
    max_value_ = Container();
  }

  std::size_t size() const {return data_.size();}

  bool add_value(const PositionGrid & position, const Container & value)
  {
    const Index index = position_to_index(position);
    if (!index.is_valid() || data_.count(index) > 0) {
      return false;
    }
    data_.insert({index, value});
    if (value.get_value() > max_value_.get_value()) {
      max_value_ = value;
    }
    return true;
  }

  Container get_value(const PositionGrid & position) const
  {
    return get_value(position_to_index(position));
  }

  Container get_value(const Index & index) const
  {
    const auto iter = data_.find(index);
    if (iter == data_.end()) {
      return Container();
    }
    return iter->second;
  }

  PositionGrid get_position(const Index & index) const
  {
    return Index::index_to_position(index, grid_size_);
  }

  void get_index_neighbors(
    const PositionGrid & position, const int index_radius, std::vector<Index> & indices) const
  {
    indices.clear();
    const Index center = position_to_index(position);
    for (int x = center.x - index_radius; x <= center.x + index_radius; ++x) {
      for (int y = center.y - index_radius; y <= center.y + index_radius; ++y) {
        for (int z = center.z - index_radius; z <= center.z + index_radius; ++z) {
          indices.push_back(Index(x, y, z));
        }
      }
    }
  }

private:
  Index position_to_index(const PositionGrid & position) const
  {
    return Index::position_to_index(position, grid_size_);
  }

  std::string name_;
  double grid_size_ = 1.0;
  std::unordered_map<Index, Container, IndexHasher> data_;
  Container max_value_;
};

}  // namespace mobile_manipulator_fkie_nbv

#endif  // MOBILE_MANIPULATOR_FKIE_NBV__NBV_GRID_HPP_
