#ifndef MOBILE_MANIPULATOR_FKIE_NBV__TREE_NANOFLANN_ADAPTER_HPP_
#define MOBILE_MANIPULATOR_FKIE_NBV__TREE_NANOFLANN_ADAPTER_HPP_

#include <memory>
#include <vector>

#include "mobile_manipulator_fkie_nbv/rrt_node.hpp"

namespace mobile_manipulator_fkie_nbv
{

struct TreeNanoflannAdapter
{
  std::vector<std::shared_ptr<RRTNode>> nodes;

  void add_node(const std::shared_ptr<RRTNode> & node)
  {
    nodes.push_back(node);
  }

  void clear()
  {
    nodes.clear();
  }

  inline std::size_t kdtree_get_point_count() const
  {
    return nodes.size();
  }

  inline double kdtree_get_pt(const size_t index, const size_t dimension) const
  {
    if (dimension == 0) {
      return nodes[index]->get_pose().position.x;
    }
    if (dimension == 1) {
      return nodes[index]->get_pose().position.y;
    }
    return nodes[index]->get_pose().position.z;
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX &) const
  {
    return false;
  }
};

}  // namespace mobile_manipulator_fkie_nbv

#endif  // MOBILE_MANIPULATOR_FKIE_NBV__TREE_NANOFLANN_ADAPTER_HPP_
