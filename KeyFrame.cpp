#include "KeyFrame.h"

KeyFrame::KeyFrame(const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &p_wb)
{
    q_wb_ = q_wb;
    p_wb_ = p_wb;
}
void KeyFrame::SetPose(const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &p_wb)
{
    q_wb_ = q_wb;
    p_wb_ = p_wb;
}
void KeyFrame::GetPose(Eigen::Quaterniond &q_wb, Eigen::Vector3d &p_wb)
{
    q_wb = q_wb_;
    p_wb = p_wb_;
}

Eigen::Vector3d KeyFrame::GetPosition()
{
    return p_wb_;
}

Eigen::Quaterniond KeyFrame::GetQuat()
{
	return q_wb_;
}

void KeyFrame::SetId(const int id)
{
    id_ = id;
}

int KeyFrame::GetId(){
    return id_;
}

void KeyFrame::AddObv(Point *p, const Eigen::Vector2d &obv)
{
    if (obvs_.count(p))
    {
        obvs_[p] = obv; // update obv
    }
    else
    {
        obvs_.insert({p, obv});
    }
}

void KeyFrame::PlusR(const Eigen::Vector3d &delta)
{
    q_wb_ = q_wb_ * Vector2Rotation(delta);
    q_wb_.normalize();
}
void KeyFrame::PlusP(const Eigen::Vector3d &delta)
{
    p_wb_ += delta;
}
bool KeyFrame::ObservePoint(Point *p_w)
{
    return obvs_.count(p_w);
}

const Eigen::Vector2d KeyFrame::GetObservation(Point *p_w)
{
    if (!obvs_.count(p_w))
    {
        cerr << "pw: " << p_w->GetId() << " was not saw by kf: " << id_ << endl;
        exit(-1);
    }
    return obvs_.at(p_w);
}
const Eigen::Vector2d KeyFrame::ProjectPw2NormPlane(Point *p_w)
{
    // p_c = q_wb_.inverse() * p_w - q_wb.inverse() * p_wb_
    // obv = (p_c / p_c[2]).head(2)
    const Eigen::Vector3d delta_p = p_w->GetPosition() - p_wb_;
    const Eigen::Vector3d p_c = q_wb_.inverse() * delta_p;
    return (p_c / p_c.z()).head(2);
}
// TODO: return jacobian of obv w.r.t pose & obv w.r.t p_w
bool KeyFrame::Jacobian(Point *p_w, Eigen::Matrix<double, 2, 6> &J_obv_pose,
                Eigen::Matrix<double, 2, 3> &J_obv_pw)
{
    if (!ObservePoint(p_w))
    {
        return false;
    }
    // p_c = q_wb_.inverse() * p_w - q_wb.inverse() * p_wb_
    // obv = (p_c / p_c[2]).head(2)
    const Eigen::Vector3d delta_p = p_w->GetPosition() - p_wb_;
    const Eigen::Vector3d p_c = q_wb_.inverse() * delta_p;

    Eigen::Matrix<double, 2, 3> J_obv_pc;
    J_obv_pc.setZero();
    const double squareZ = p_c.z() * p_c.z();
    // jacobian of obv w.r.t. pc
    J_obv_pc << 1 / p_c.z(), 0, -p_c.x() / squareZ,
        0, 1 / p_c.z(), -p_c.y() / squareZ;

    // jacobian of pc w.r.t rotation & position
    Eigen::Matrix<double, 3, 6> J_pc_R_p;
    J_pc_R_p.setZero();
    // about rotation
    J_pc_R_p.block(0, 0, 3, 3) = SkewSymmetricMatrix(q_wb_.inverse() * delta_p);
    // about translation
    J_pc_R_p.block(0, 3, 3, 3) = -q_wb_.toRotationMatrix().transpose();
    J_obv_pose = J_obv_pc * J_pc_R_p;

    // about p_w
    Eigen::Matrix3d J_pc_pw;
    J_pc_pw.setZero();
    J_pc_pw = q_wb_.toRotationMatrix().transpose();
    J_obv_pw = J_obv_pc * J_pc_pw;

    return true;
}
