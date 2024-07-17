#include "NonLinearOptimizer.h"

using namespace std;

NonLinearOptimizer::NonLinearOptimizer(const int maxIter, const double convergeValue)
    : maxIterate_(maxIter), convergeValue_(convergeValue) {}

void NonLinearOptimizer::AddPoint(shared_ptr<Point> p)
{
    if (!ps_.count(p))
    {
        ps_.insert({ p, pId_++ });
    }
}
void NonLinearOptimizer::AddKeyFrame(shared_ptr<KeyFrame> kf)
{
    if (!kfs_.count(kf))
    {
        kfs_.insert({ kf, kfId_++ });
    }
}

int NonLinearOptimizer::ComputeResidualNum()
{
    auto pit = ps_.begin();
    int sum = 0;
    while (pit != ps_.end())
    {
        const vector<shared_ptr<KeyFrame>> &kfs = (*pit).first->GetKeyFrames();
        for (int i = 0; i < kfs.size(); ++i)
        {
            if (kfs_.count(kfs[i]))
            {
                sum += 2; // 添加一个重投影残差
            }
        }
        ++pit;
    }

    auto kfit = kfs_.begin();
    while (kfit != kfs_.end()) 
    {
        if(kfit->first->GetPrevKF()){
            sum += 6; // 添加一个相对位姿约束残差
        }
        ++kfit;
    }
    return sum;
}

bool NonLinearOptimizer::ComputeJacobianFull(const int bSize, Eigen::MatrixXd &Jfull)
{
    if (bSize == 0)
    {
        cerr << "residual num is 0, Jacobian can't be evaluated!" << endl;
        return false;
    }

    int varSize = kfs_.size() * KeyFrame::size() + ps_.size() * Point::size();
    Jfull = Eigen::MatrixXd(bSize, varSize);
    Jfull.setZero();

    int resId = 0;
    auto pit = ps_.begin();
    while (pit != ps_.end())
    {
        const vector<shared_ptr<KeyFrame>> &kfs = pit->first->GetKeyFrames();
        for (int i = 0; i < kfs.size(); ++i)
        {
            if (kfs_.count(kfs[i]))
            {
                const shared_ptr<Point> &p_w = pit->first;
                Eigen::Matrix<double, 2, 6> J_obv_pose;
                Eigen::Matrix<double, 2, 3> J_obv_pw;
                if (kfs[i]->Jacobian(p_w.get(), J_obv_pose, J_obv_pw)) {
                    const int poseCol = kfs_[kfs[i]] * KeyFrame::size();
                    const int pointCol = pointStartColInJacobian_ + ps_[p_w] * Point::size();
                    // 填充Jacobian
                    Jfull.block(resId, poseCol, 2, 6) = J_obv_pose;
                    // TODO: debug只优化 point
                    //Jfull.block(resId, poseCol, 2, 6).setZero();

                    Jfull.block(resId, pointCol, 2, 3) = J_obv_pw;
                    // TODO: debug只优化pose
                    //Jfull.block(resId, pointCol, 2, 3).setZero();

                    // 更新残差id
                    resId += 2;
                }
                else {
                    cerr << "[ComputeJacobianFull] Error, point isn't seen!";
                    exit(-1);
                }

            }
        }
        ++pit;
    }

    auto kfit2 = kfs_.begin();
    while (kfit2 != kfs_.end()) {
        // 添加帧间pose变换的Jacobian
        shared_ptr<KeyFrame> prev = kfit2->first->GetPrevKF();
        if (kfs_.count(prev)) {
            Eigen::Matrix<double, 6, 6> J_pose1, J_pose2;
            if (kfit2->first->Jacobian(prev, J_pose1, J_pose2)) {
                const int poseCol1 = kfs_[prev] * KeyFrame::size();
                const int poseCol2 = kfs_[kfit2->first] * KeyFrame::size();
                Jfull.block(resId, poseCol1, 6, 6) = J_pose1;
                Jfull.block(resId, poseCol2, 6, 6) = J_pose2;
                resId += 6;
            }
        }
        ++kfit2;
    }

    return true;
}

bool NonLinearOptimizer::ComputeResidualFull(const int bSize, Eigen::VectorXd &bFull)
{
    if (bSize == 0)
    {
        cerr << "residual num is 0, Residual doesn't exist!" << endl;
        return false;
    }

    bFull = Eigen::VectorXd(bSize);
    bFull.setZero();

    // 填充 residuals
    int resId = 0;
    auto pit = ps_.begin();
    while (pit != ps_.end())
    {
        const vector<shared_ptr<KeyFrame>> &kfs = pit->first->GetKeyFrames();
        for (int i = 0; i < kfs.size(); ++i)
        {
            if (kfs_.count(kfs[i]))
            {
                const shared_ptr<Point> &p_w = pit->first;
                // 填充residuals
                bFull.middleRows(resId, 2) = kfs[i]->ProjectPw2NormPlane(p_w.get()) -
                    kfs[i]->GetObservation(p_w.get());

                // 更新残差id
                resId += 2;
            }
        }
        ++pit;
    }

    auto kfit2 = kfs_.begin();
    while (kfit2 != kfs_.end()) {
        // 添加帧间pose变换的Jacobian
        shared_ptr<KeyFrame> prev = kfit2->first->GetPrevKF();
        shared_ptr<KeyFrame> cur = kfit2->first;
        if (kfs_.count(prev)) {
            Eigen::Quaterniond q_b1b2_obv;
            Eigen::Vector3d p_b1b2_obv;
            cur->GetRelativePoseConstraint(q_b1b2_obv, p_b1b2_obv);
            const Eigen::Quaterniond deltaQ = cur->GetQuat().inverse() * prev->GetQuat();
            const Eigen::Vector3d residualR = LogSO3((q_b1b2_obv * deltaQ).toRotationMatrix());
            const Eigen::Vector3d deltaP = prev->GetQuat().inverse() *
                (cur->GetPosition() - prev->GetPosition());
            const Eigen::Vector3d residualP = deltaP - p_b1b2_obv;
            bFull.middleRows(resId, 3) = residualR;
            resId += 3;
            bFull.middleRows(resId, 3) = residualP;
            resId += 3;
        }
        ++kfit2;
    }
    //cout.precision(3);
    //cout << "residual: " << bFull.transpose().head(6) << " | "
    //    << bFull.transpose().tail(6) << endl;
    return true;
}

void NonLinearOptimizer::ComputeHmatrix()
{
    // TODO：稀疏矩阵计算简化
    H_ = J_.transpose() * J_;
}

const double NonLinearOptimizer::ComputeCost(const Eigen::VectorXd &b)
{
    double cost = 0;
    for (int i = 0; i < b.size(); i += 2)
    {
        const Eigen::Vector2d &res = b.middleRows(i, 2);
        cost += res.norm();
    }
    return cost;
}

Eigen::VectorXd NonLinearOptimizer::SolveHxEqual2b()
{
    Eigen::MatrixXd H = H_;
    H.setIdentity();
    H = H * lambda_;
    H += H_;
    Eigen::MatrixXd b = J_.transpose() * b_;
    // TODO：使用舒尔补进行边缘化计算
    return H.colPivHouseholderQr().solve(-b);
}

void NonLinearOptimizer::StoreLastStatus()
{
    lastPoint_.clear();
    lastKFquat_.clear();
    lastKFpos_.clear();

    // 保留Point状态
    auto pit = ps_.begin();
    while (pit != ps_.end())
    {
        lastPoint_.insert({ pit->first, pit->first->GetPosition() });
        ++pit;
    }
    // 保留KeyFrame状态
    auto kfit = kfs_.begin();
    while (kfit != kfs_.end())
    {
        Eigen::Quaterniond q;
        Eigen::Vector3d p;
        kfit->first->GetPose(q, p);
        lastKFquat_.insert({ kfit->first, q });
        lastKFpos_.insert({ kfit->first, p });
        ++kfit;
    }
}

bool NonLinearOptimizer::UpdateAllVariables(const Eigen::VectorXd &delta_x)
{
    if (!delta_x.rows()) {
        cerr << "[Error] current step's delta_x is NULL!" << endl;
        return false;
    }
    //cout.precision(3);
    //cout << "delta_x: " << delta_x.transpose() << endl;
    // 更新pose
    auto kfit = kfs_.begin();
    while (kfit != kfs_.end())
    {
        const int id = kfit->second * KeyFrame::size();
        const Eigen::Vector3d delta_rot = delta_x.middleRows(id, 3);
        const Eigen::Vector3d delta_pos = delta_x.middleRows(id + 3, 3);
        // TODO: 研究Lambda如何设置初值以及如何更新
        // 发现Lambda的值将影响pose的收敛速度
        kfit->first->PlusR(delta_rot);
        kfit->first->PlusP(delta_pos);
        ++kfit;
    }

    // 更新point
    auto pit = ps_.begin();
    while (pit != ps_.end())
    {
        const int id = pointStartColInJacobian_ + pit->second * Point::size();
        const Eigen::Vector3d delta_p = delta_x.middleRows(id, 3);
        pit->first->Plus(delta_p);
        ++pit;
    }

    return true;
}

bool NonLinearOptimizer::RestoreStatus()
{
    if (lastPoint_.empty() || lastKFquat_.empty() || lastKFpos_.empty())
    {
        cerr << "last size of point, quat, pos: " << lastPoint_.size() << " " << lastKFquat_.size() << " " << lastKFpos_.size() << endl;
        return false;
    }

    // 恢复Point状态
    auto pit = ps_.begin();
    while (pit != ps_.end())
    {
        pit->first->SetPosition(lastPoint_[pit->first]);
        ++pit;
    }
    // 恢复KeyFrame状态
    auto kfit = kfs_.begin();
    while (kfit != kfs_.end())
    {
        Eigen::Quaterniond q = lastKFquat_[kfit->first];
        Eigen::Vector3d p = lastKFpos_[kfit->first];
        kfit->first->SetPose(q, p);
        ++kfit;
    }
    return true;
}

bool NonLinearOptimizer::Optimize()
{
    // 关键帧在Jacobian的起始索引从0开始
    const int bSize = ComputeResidualNum();
    ComputeJacobianFull(bSize, J_);
    ComputeResidualFull(bSize, b_);
    if (lastCost_ < 0) {
        lastCost_ = ComputeCost(b_);
    }
    ComputeHmatrix();
    Eigen::MatrixXd delta_x = SolveHxEqual2b();
    // 使用LM算法计算增量，需要保存当前各变量的值，用于恢复上一次状态
    StoreLastStatus();
    UpdateAllVariables(delta_x);
    ComputeResidualFull(bSize, b_);
    const double cost1 = ComputeCost(b_);
    cout << "[Cost] before & after optimization: " << lastCost_ << " " << cost1 << endl;
    cout << "[Lambda]: " << lambda_ << endl;
    if (abs(cost1 - lastCost_) < convergeValue_ || cost1 < convergeValue_)
    {
        return true;
    }
    if (cost1 >= lastCost_)
    {
        //cout << "[Failed] current optimization step rejected" << endl;
        // 增大lambda，以减小步长
        lambda_ *= 1.5;
        RestoreStatus();
        return false;
    }
    else
    {
        //cout << "[Successful] current optimization step accepted" << endl;
        // 减小lambda，近似G-N方法以加速收敛
        lambda_ *= 0.5;
        lastCost_ = cost1;
        return false;
    }
}

void NonLinearOptimizer::PrintBasicMessage() {
    cout << "[NonLinearOptimizer]: Print basic message: " << endl;
    cout << "points: " << endl;
    auto pit = ps_.begin();
    while (pit != ps_.end()) {
        cout << " " << pit->second << ": " << pit->first->GetPosition().transpose() << endl;
        ++pit;
    }
    cout << "pose: " << endl;
    auto kfit = kfs_.begin();
    while (kfit != kfs_.end()) {
        cout << " " << kfit->second << ": " << kfit->first->GetPosition().transpose() << endl;
        ++kfit;
    }
    cout << "others: " << endl;
    cout << " " << "pointStartColInJacobian_: " << pointStartColInJacobian_ << endl;
    cout << "[ComputeResidualNum]: residual num: " << ComputeResidualNum() << endl;
    cout << "##############################" << endl;
}

bool NonLinearOptimizer::iterate()
{
    pointStartColInJacobian_ = kfs_.size() * KeyFrame::size();
    PrintBasicMessage();
    for (int i = 0; i < maxIterate_; ++i)
    {
        if (Optimize())
        {
            cout << "[Finally] We finally succeed to run the BA!" << endl;
            return true;
        }
    }
    cerr << "[Error] We finally failed to run the BA!" << endl;
    return false;
}
