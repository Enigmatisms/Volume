#pragma once
#include "Object.hpp"

typedef std::vector<Eigen::Vector2d> Obstacle;

struct ObjCompFunctor{
    bool operator() (const Object* const o1, const Object* const o2) const {
        return o1->min_dist > o2->min_dist;
    }
};

/// @brief 体积光主函数，包括交互 / 可视化 / debug测试 / 计算
/// @todo 边界处理 / 可视化与交互
class Volume {
    Volume() {}
    ~Volume() {}
public:
    void calculateVisualSpace(const std::vector<Obstacle>& obstcs, cv::Point obs);
    
    void visualizeVisualSpace(const Eigen::Vector2d& obs, cv::Mat& dst) const;
private:
    std::priority_queue<Object*, std::vector<Object*>, ObjCompFunctor> heap;
    std::vector<Object> objs;
}; 