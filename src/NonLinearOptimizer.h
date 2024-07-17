#pragma once

#include "Point.h"
#include "KeyFrame.h"

class NonLinearOptimizer
{
public:
    NonLinearOptimizer(const int maxIter = 100, const double convergeValue = 1e-4);
    void AddPoint(shared_ptr<Point> p);
    void AddKeyFrame(shared_ptr<KeyFrame> kf);
    int ComputeResidualNum();
    bool ComputeJacobianFull(const int bSize, Eigen::MatrixXd &Jfull);
    bool ComputeResidualFull(const int bSize, Eigen::VectorXd &bFull);
    void ComputeHmatrix();
    const double ComputeCost(const Eigen::VectorXd &b);
    Eigen::VectorXd SolveHxEqual2b();
    void StoreLastStatus();
    bool UpdateAllVariables(const Eigen::VectorXd &delta_x);
    bool RestoreStatus();
    bool Optimize();
    bool iterate();
	void PrintBasicMessage();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::VectorXd b_;
    Eigen::MatrixXd J_;
    Eigen::MatrixXd H_;
    unordered_map<shared_ptr<Point>, int> ps_;
    unordered_map<shared_ptr<KeyFrame>, int> kfs_; // 保存当前参与BA的所有关键帧信息及在H矩阵中的位置
    int pId_ = 0;
    int kfId_ = 0;
    int pointStartColInJacobian_ = -1;
    double lastCost_ = -1;
    double lambda_ = 1.0;
    double convergeValue_ = 1e-4;
    int maxIterate_ = 100;
    unordered_map<shared_ptr<Point>, Eigen::Vector3d> lastPoint_;
    unordered_map<shared_ptr<KeyFrame>, Eigen::Quaterniond> lastKFquat_;
    unordered_map<shared_ptr<KeyFrame>, Eigen::Vector3d> lastKFpos_;
};
