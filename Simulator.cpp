#include "Simulator.h"

#include <random>

Simulator::Simulator(const int pointNum, const int KeyFrameNum, const int iterate,
    const double convergeValue)
    : optimizer_(NonLinearOptimizer(iterate, convergeValue))
{
    Eigen::MatrixXd ps;
    GeneratePoints(ps, pointNum);
    vector<Eigen::Vector3d> pos;
    vector<Eigen::Quaterniond> quat;
    GeneratePoeses(quat, pos);
    GenerateMap(ps, quat, pos);
    auto kfit = map_.GetKeyFrames().begin();
    while (kfit != map_.GetKeyFrames().end())
    {
        optimizer_.AddKeyFrame(*kfit);
        ++kfit;
    }
    auto pit = map_.GetPoints().begin();
    while (pit != map_.GetPoints().end())
    {
        optimizer_.AddPoint(*pit);
        ++pit;
    }
    cout << "[Sim] Data Prepared OK!" << endl;
}

bool Simulator::Optimize()
{
    ReportDifference(false);
    cout << "[Sim] Start Optimize!" << endl;
    const bool succeed = optimizer_.iterate();
    ReportDifference(true);
    return succeed;
}


void Simulator::GeneratePoints(Eigen::MatrixXd &ps, const int pointNum)
{
    cout << "[GeneratePoints]..." << endl;
    const int num = pointNum > 6 ? pointNum : 6;
    ps = Eigen::MatrixXd(3, num);
    std::random_device rd;                             // 生成随机数种子
    std::mt19937 gen(rd());                            // 定义随机数生成引擎
    std::uniform_real_distribution<> distrib_real(1.0, 6.0); // [1.0,6.0]均匀分布实数
    for (int i = 0; i < num; ++i) {
        ps.col(i) << distrib_real(gen), distrib_real(gen), distrib_real(gen);
    }
}
void Simulator::GeneratePoeses(vector<Eigen::Quaterniond> &quat, vector<Eigen::Vector3d> &pos,
    const int KeyFrameNum)
{
    cout << "[GeneratePoeses]..." << endl;
    // 仅生成2个pose用于测试
    pos.push_back({ 0.1, 0.2, 0.3 });
    pos.push_back({ 2, 0.5, 0.2 });
    Eigen::Quaterniond q1(Eigen::AngleAxisd(30 * kDeg2Rad, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    Eigen::Quaterniond q2(Eigen::AngleAxisd(60 * kDeg2Rad, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    quat.push_back(q1);
    quat.push_back(q2);
}
void Simulator::GenerateMap(const Eigen::MatrixXd &ps, const vector<Eigen::Quaterniond> &quat,
    const vector<Eigen::Vector3d> &pos)
{
    cout << "[GenerateMap]..." << endl;
    for (int i = 0; i < ps.cols(); ++i)
    {
        shared_ptr<Point> p = make_shared<Point>(ps.col(i));
        map_.AddPoint(p);
        // 保存真值用于优化后评估
        truePointPos_.insert({ p, ps.col(i) });
    }
    for (int i = 0; i < pos.size(); ++i)
    {
        shared_ptr<KeyFrame> kf = make_shared<KeyFrame>(quat[i], pos[i]);
        map_.AddKeyFrame(kf);
        truePoseQuat_.insert({ kf, quat[i] });
        truePosePos_.insert({ kf, pos[i] });
    }

    // 添加keyframe & point相互观测信息
    auto pit = map_.GetPoints().begin();
    while (pit != map_.GetPoints().end())
    {
        cout << "[GenerateMap]: (*pit)->pos: " << (*pit)->GetPosition().transpose() << endl;
        auto kfit = map_.GetKeyFrames().begin();
        while (kfit != map_.GetKeyFrames().end())
        {
            // KeyFrame添加Point观测
            cout << "[GenerateMap]: (*kfit)->pos: " << (*kfit)->GetPosition().transpose() << endl;
            const Eigen::Vector2d trueObv = (*kfit)->ProjectPw2NormPlane((*pit).get());
            (*kfit)->AddObv((*pit).get(), trueObv);
            // Point添加KeyFrame观测
            (*pit)->AddKeyFrame(*kfit);
            ++kfit;
        }
        ++pit;
    }

    AddNoise2Map();
}

void Simulator::AddNoise2Map()
{
    cout << "[AddNoise2Map]: Add some noise to the dataset." << endl;
    auto pit = map_.GetPoints().begin();
    while (pit != map_.GetPoints().end()) {
        // 给地图点添加噪声, TODO:理论上应该是给量测添加噪声
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> distrib_normal(0., 0.1);
        const double n = distrib_normal(gen);
        Eigen::Vector3d noise(n, n, n);
        (*pit)->SetPosition((*pit)->GetPosition() + noise);
        ++pit;
    }

    auto kfit = map_.GetKeyFrames().begin();
    while (kfit != map_.GetKeyFrames().end()) {
        // 给关键帧pose添加噪声
        Eigen::Vector3d posNoise(0.5, 0.3, 0.1);
        Eigen::Quaterniond quatNoise(Eigen::AngleAxisd(5 * kDeg2Rad, Eigen::Vector3d::UnitY()).toRotationMatrix());
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
        (*kfit)->GetPose(quat, pos);
        pos += posNoise;
        quat *= quatNoise;
        (*kfit)->SetPose(quat, pos);
        ++kfit;
    }
}

void Simulator::ReportDifference(bool optimized) {
    if (optimized) {
        cout << "Report after optimized: " << endl;
    }
    else {
        cout << "Report before optimizing: " << endl;
    }

    cout << "points difference: " << endl;
    auto pit = map_.GetPoints().begin();
    int pointNum = 0;
    while (pit != map_.GetPoints().end() && pointNum < 10) {
        cout << " " << (*pit)->GetId() << " " << ((*pit)->GetPosition() - truePointPos_[(*pit)]).norm()
            << " meters." << endl;
        ++pit;
        ++pointNum;
    }

    cout << "pose difference: " << endl;
    auto kfit = map_.GetKeyFrames().begin();
    while (kfit != map_.GetKeyFrames().end()) {
        cout << " " << (*kfit)->GetId() << " " << ((*kfit)->GetPosition() - truePosePos_[(*kfit)]).norm()
            << " meters." << endl;
        cout << "   " << Quat2RPY((*kfit)->GetQuat().inverse() * truePoseQuat_[(*kfit)]).norm() * kRad2Deg
            << " degs." << endl;
        ++kfit;
    }
    cout << "##############################" << endl;
}
