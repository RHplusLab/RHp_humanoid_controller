#include "rhphumanoid_walking/link_data.h"

namespace rhp
{

LinkData::LinkData()
  : parent_(-1),
    sibling_(-1),
    child_(-1),
    mass_(0.0),
    joint_limit_max_(100.0),
    joint_limit_min_(-100.0),
    joint_angle_(0.0),
    joint_velocity_(0.0),
    joint_acceleration_(0.0)
{
  relative_position_.setZero();
  joint_axis_.setZero();
  center_of_mass_.setZero();
  position_.setZero();
  orientation_.setIdentity();
  transformation_.setIdentity();
}

LinkData::~LinkData()
{
}

} // namespace rhp
