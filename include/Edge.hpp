#pragma once
#include <Eigen/Core>
#include <queue>

// 投影边
class Edge: public std::deque<Eigen::Vector2d> {
public:
    std::pair<int, int> proj_ids;           // 待投影点的id
    std::pair<double, double> angles;       // 用于快速判定角度
    double min_dist = 1e9;
    bool valid = true;                      // 是否完全被遮挡
    bool projected = false;                 // 是否投影过
public:
    Edge() {reset();}
    bool angleInRange(double angle) const {
        if (angles.first > 0 && angles.second < 0) {        // 跨越奇异
            return (angle > angles.first) || (angle < angles.second);
        } else {
            return (angle > angles.first) && (angle < angles.second);
        }
    }
    
    bool notInRangeSmaller(double angle) const {
        if (angles.first > 0 && angles.second < 0) {        // 跨越奇异
            return (angle < angles.first) && (angle > angles.second);
        } else {
            return (angle < angles.first);
        }
    }

    bool notInRangeBigger(double angle) const {
        if (angles.first > 0 && angles.second < 0) {        // 跨越奇异
            return (angle < angles.first) && (angle > angles.second);
        } else {
            return (angle > angles.second);
        }
    }

    void reset() {
        clear();
        proj_ids.first = 0;
        proj_ids.second = 0;
        angles.first = 0.0;
        angles.second = 0.0;
        min_dist = 1e9;
        valid = true;
        projected = false;
    }

    void initWithObs(Eigen::Vector2d obs);            // 根据观测点，初始化角度以及ids
};
