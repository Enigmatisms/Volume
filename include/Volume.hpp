#pragma once
#include <fstream>
#include "Object.hpp"

typedef std::vector<cv::Point> Obstacle;



/// @brief 体积光主函数，包括交互 / 可视化 / debug测试 / 计算
/// @todo 边界处理 / 可视化与交互
class Volume {
public:
    Volume(): heap(ObjCompFunctor(objs)) {}
    ~Volume() {}
public:
    void calculateVisualSpace(const std::vector<Obstacle>& _obstcs, cv::Point obs, cv::Mat& src);
    
    void visualizeVisualSpace(const std::vector<Obstacle>& _obstcs, const Eigen::Vector2d& obs, cv::Mat& dst) const;

    void simplePreVisualize(cv::Mat& src, const cv::Point& obs) const;
private:

    struct ObjCompFunctor{
        ObjCompFunctor(const std::vector<Object>& _objs): objs(_objs) {}
        const std::vector<Object>& objs;
        bool operator() (size_t o1, size_t o2) const {
            return objs[o1].min_dist > objs[o2].min_dist;
        }
    };

    std::priority_queue<size_t, std::vector<size_t>, ObjCompFunctor> heap;
    std::vector<Object> objs;
}; 