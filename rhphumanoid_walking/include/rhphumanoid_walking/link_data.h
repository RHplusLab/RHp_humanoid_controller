#ifndef LINK_DATA_H_
#define LINK_DATA_H_

#include <string>
#include <eigen3/Eigen/Eigen>

namespace rhp
{

class LinkData
{
public:
  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::Vector3d relative_position_;
  Eigen::Vector3d joint_axis_;
  Eigen::Vector3d center_of_mass_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::Vector3d    position_;
  Eigen::Matrix3d    orientation_;
  Eigen::Matrix4d    transformation_;
};

} // namespace rhp

#endif /* LINK_DATA_H_ */
