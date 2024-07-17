#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "KeyFrame.h"

using namespace std;

class KeyFrame;

class Point
{
public:
    Point(double x, double y, double z);
    Point(const Eigen::Vector3d &p);
    void Plus(const Eigen::Vector3d &delta);
    void SetId(const int id);
    int GetId();
    void SetPosition(const Eigen::Vector3d &pos);
    Eigen::Vector3d GetPosition();
    const vector<shared_ptr<KeyFrame>> &GetKeyFrames();
    void AddKeyFrame(shared_ptr<KeyFrame> pkf);
    static int size()
    {
        return 3;
    }
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero(); // p_w
    int id_ = -1;
    vector<shared_ptr<KeyFrame>> kfs_;
};
