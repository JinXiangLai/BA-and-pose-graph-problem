#pragma once
#include <unordered_set>

#include "Point.h"
#include "KeyFrame.h"

using namespace std;

class Map
{
public:
    Map(){};
    void AddPoint(shared_ptr<Point> p)
    {
        if (!points_.count(p))
        {
            p->SetId(++pid_);
            points_.insert(p);
        }
    }
    void AddKeyFrame(shared_ptr<KeyFrame> kf)
    {
        if(!kfs_.count(kf)){
            kf->SetId(++kfid_);
            kfs_.insert(kf);
        }
    }

    const unordered_set<shared_ptr<Point>> &GetPoints()
    {
        return points_;
    }

    const unordered_set<shared_ptr<KeyFrame>> &GetKeyFrames()
    {
        return kfs_;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    unordered_set<shared_ptr<Point>> points_;
    unordered_set<shared_ptr<KeyFrame>> kfs_;
    int pid_ = -1; // point id
    int kfid_ = -1;
};
