#include <math.h>

#include <Eigen/Dense>

#define M_PI 3.14159265358979323846
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180;

// 罗德里格斯公式求解
Eigen::Matrix3d Vector2Rotation(const Eigen::Vector3d &r);
Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d &v);
Eigen::Vector3d Quat2RPY(const Eigen::Quaterniond &q);
