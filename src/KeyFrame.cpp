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

int KeyFrame::GetId() {
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
		obvs_.insert({ p, obv });
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

void KeyFrame::SetPrevKF(shared_ptr<KeyFrame> last, const Eigen::Quaterniond &q_b1b2,
                    const Eigen::Vector3d &p_b1b2)
{
	prevKF_ = last;
	q_b1b2_obv_ = q_b1b2;
	p_b1b2_obv_ = p_b1b2;
}

void KeyFrame::GetRelativePoseConstraint(Eigen::Quaterniond &q_b1b2, Eigen::Vector3d &p_b1b2)
{
	q_b1b2 = q_b1b2_obv_;
	p_b1b2 = p_b1b2_obv_;
}


bool KeyFrame::Jacobian(shared_ptr<KeyFrame> last, Eigen::Matrix<double, 6, 6> &J_pose1,
	Eigen::Matrix<double, 6, 6> &J_pose2)
{
	if (last != prevKF_ || !last) {
		return false;
	}
	Eigen::Quaterniond q_wb1;
	Eigen::Vector3d p_wb1;
	last->GetPose(q_wb1, p_wb1);
	// const Eigen::Quaterniond q_b2b1 = q_wb_.inverse() * q_wb1;
	// ΔR = LogSO3(q_b1b2_ * q_b2b1)
	// const Eigen::Vector3d p_b1b2 = q_wb1.inverse() * p_wb_ - q_wb1.inverse() * p_wb1;
	// ΔP = q_wb1.inverse() * (p_wb_ - p_wb1);
	J_pose1.setZero();
	// ΔR w.r.t R1, 使用BCH近似
	const Eigen::Quaterniond deltaQ = q_b1b2_obv_ * q_wb_.inverse() * q_wb1;
	const Eigen::Vector3d residualR = LogSO3(deltaQ.toRotationMatrix());
	J_pose1.block(0, 0, 3, 3) = InverseRightJacobianSO3(residualR);
	// ΔP w.r.t R1
	const Eigen::Vector3d delta_p = p_wb_ - p_wb1;
	J_pose1.block(3, 0, 3, 3) = SkewSymmetricMatrix(q_wb1.inverse() * delta_p);
	// ΔP w.r.t P1
	J_pose1.block(3, 3, 3, 3) = -q_wb1.toRotationMatrix().transpose();

	J_pose2.setZero();
	// ΔR w.r.t R2, 先使用伴随性质，再使用BCH近似
	J_pose2.block(0, 0, 3, 3) = InverseRightJacobianSO3(residualR) *
		-q_wb1.toRotationMatrix().transpose() * q_wb_.toRotationMatrix();
	// ΔP w.r.t P2
	J_pose2.block(3, 3, 3, 3) = q_wb1.toRotationMatrix().transpose();
	return true;
}
