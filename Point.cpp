#include "Point.h"

Point::Point(double x, double y, double z)
{
    p_.x() = x;
    p_.y() = y;
    p_.z() = z;
}
Point::Point(const Eigen::Vector3d &p) : p_(p) {}

void Point::Plus(const Eigen::Vector3d &delta)
{
    p_ += delta;
}
void Point::SetId(const int id)
{
    id_ = id;
}
int Point::GetId()
{
    return id_;
}
void Point::SetPosition(const Eigen::Vector3d &pos)
{
    p_ = pos;
}
Eigen::Vector3d Point::GetPosition()
{
    return p_;
}
const vector<shared_ptr<KeyFrame>> &Point::GetKeyFrames()
{
    return kfs_;
}
void Point::AddKeyFrame(shared_ptr<KeyFrame> pkf)
{
    kfs_.push_back(pkf);
}
