#pragma once

#include <iostream>
#include <unordered_map>

#include "Point.h"
#include "Utils.h"

using namespace std;

class Point;

class KeyFrame
{
public:
    KeyFrame(const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &p_wb);
    void SetPose(const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &p_wb);
    void GetPose(Eigen::Quaterniond &q_wb, Eigen::Vector3d &p_wb);
    Eigen::Vector3d GetPosition();
    Eigen::Quaterniond GetQuat();
    void SetId(const int id);
    int GetId();
    void SetPrevKF(shared_ptr<KeyFrame> last, const Eigen::Quaterniond &q_b1b2,
                    const Eigen::Vector3d &p_b1b2);
    shared_ptr<KeyFrame> GetPrevKF() {return prevKF_;}
    void GetRelativePoseConstraint(Eigen::Quaterniond &q_b1b2, Eigen::Vector3d &p_b1b2);
    void AddObv(Point *p, const Eigen::Vector2d &obv);
    void PlusR(const Eigen::Vector3d &delta);
    void PlusP(const Eigen::Vector3d &delta);
    bool ObservePoint(Point *p_w);
    const Eigen::Vector2d GetObservation(Point *p_w);
    const Eigen::Vector2d ProjectPw2NormPlane(Point *p_w);
    bool Jacobian(Point *p_w, Eigen::Matrix<double, 2, 6> &J_obv_pose,
                  Eigen::Matrix<double, 2, 3> &J_obv_pw);
    bool Jacobian(shared_ptr<KeyFrame> last, Eigen::Matrix<double, 6, 6> &J_pose1,
                    Eigen::Matrix<double, 6, 6> &J_pose2);
    
    static int size()
    {
        return 6;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // unordered_map<shared_ptr<Point>, Eigen::Vector2d> obvs_;
    unordered_map<Point*, Eigen::Vector2d> obvs_; // 由于相互引用，所以只能使用指针
    Eigen::Quaterniond q_wb_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_wb_ = Eigen::Vector3d::Zero();
    shared_ptr<KeyFrame> prevKF_{nullptr};
    Eigen::Quaterniond q_b1b2_obv_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_b1b2_obv_ = Eigen::Vector3d::Zero();
    int id_ = -1;
};
