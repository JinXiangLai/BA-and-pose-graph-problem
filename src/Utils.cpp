#include "Utils.h"

Eigen::Matrix3d Vector2Rotation(const Eigen::Vector3d &r)
{
	const double angle = r.norm();
	if (angle * kRad2Deg < 1e-6)
	{
		return Eigen::Matrix3d::Identity();
	}
	else
	{
		return Eigen::AngleAxisd(angle, r.normalized()).toRotationMatrix();
	}
}

Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d &v)
{
	Eigen::Matrix3d res = Eigen::Matrix3d::Zero();
	res << 0, -v[2], v[1],
		v[2], 0, -v[0],
		-v[1], v[0], 0;
	return res;
}

Eigen::Vector3d Quat2RPY(const Eigen::Quaterniond &_q){
	const Eigen::Quaterniond q = _q.normalized();
	const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
	
	// 防止除以零  
    double epsilon = 1e-6;  
      
    // roll (x-axis rotation)  
    double sinr_cosp = 2.0 * (w * x + y * z);  
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);  
    const double roll = std::atan2(sinr_cosp, cosr_cosp);  
  
    // pitch (y-axis rotation)  
    double sinp = 2.0 * (w * y - z * x); 
	double pitch = 0;
    if (std::abs(sinp) >= 1)  
        pitch = std::copysign(M_PI / 2, sinp); // 使用90度或-90度  
    else  
        pitch = std::asin(sinp);  
  
    // yaw (z-axis rotation)  
    double siny_cosp = 2.0 * (w * z + x * y);  
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);  
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
	return {roll, pitch, yaw};
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &r)
{
    const double x = r[0], y = r[1], z = r[2];
	const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
}


//Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R){
//    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    return svd.matrixU() * svd.matrixV().transpose();
//}

//Eigen::Matrix3d Vector2Rotation(const Eigen::Vector3d &r) {
	//const double x = r[0], y = r[1], z = r[2];
	//const double d2 = x*x+y*y+z*z;
	//const double d = sqrt(d2);
	//Eigen::Matrix3d W;
	//W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
	//if(d<1e-5)
	//{
	//    Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
	//    return NormalizeRotation(res);
	//}
	//else
	//{
	//    Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
	//    return NormalizeRotation(res);
	//}
//}
