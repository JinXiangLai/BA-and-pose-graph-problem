#include "Map.h"
#include "NonLinearOptimizer.h"

class Simulator
{
public:
    Simulator(const int pointNum = 3, const int KeyFrameNum = 2, const int iterate = 100,
        const double convergeValue = 1e-4);

    bool Optimize();

	void ReportDifference(bool optimized = true);

private:
    void GeneratePoints(Eigen::MatrixXd &ps, const int pointNum = 3);
    void GeneratePoeses(vector<Eigen::Quaterniond> &quat, vector<Eigen::Vector3d> &pos, const int KeyFrameNum = 2);
    void GenerateMap(const Eigen::MatrixXd &ps, const vector<Eigen::Quaterniond> &quat, const vector<Eigen::Vector3d> &pos);
    void AddNoise2Map();

    Map map_;
    NonLinearOptimizer optimizer_;
	unordered_map<shared_ptr<Point>, Eigen::Vector3d> truePointPos_;
	unordered_map<shared_ptr<KeyFrame>, Eigen::Vector3d> truePosePos_;
	unordered_map<shared_ptr<KeyFrame>, Eigen::Quaterniond> truePoseQuat_;
};