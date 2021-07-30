#pragma once
#include <Eigen/Core>
#include <queue>

// 投影边
class Edge: public std::deque<Eigen::Vector3d> {
public:
    std::pair<int, int> proj_ids;           // 待投影点的id
    double min_dist = 1e9;
    bool valid = true;                      // 是否完全被遮挡
public:
    Edge() {reset();}

    bool angleInRange(double angle) const {
        if (this->front().z() > this->back().z()) {        // 跨越奇异
            return (angle > this->front().z()) || (angle < this->back().z());
        } else {
            return (angle > this->front().z()) && (angle < this->back().z());
        }
    }

    void reset() {
        clear();
        proj_ids.first = 0;
        proj_ids.second = 0;
        min_dist = 1e9;
        valid = true;
    }

    void initWithObs(Eigen::Vector2d obs);            // 根据观测点，初始化角度以及ids
    
    int rotatedBinarySearch(double angle) const;            // binary search acceleration
private:
    int binarySearch(std::pair<int, int> range, double angle) const;
};
