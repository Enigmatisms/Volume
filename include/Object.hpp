#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include "edge.hpp"

struct EdgeCompFunctor{
    bool operator() (const Edge* const e1, const Edge* const e2) const {
        return e1->min_dist > e2->min_dist;
    }
};

// 物体定义，物体内部有很多edge
class Object {
public:
    Object();
    ~Object();
public:
    // 内部投影，自我更新
    // 内部投影之后，一个object内部最多还有两条可以投影的edge，只需要遍历即可
    void internalProjection(const Eigen::Vector2d& obs);

    // 根据观测位置，进行背面剔除，建立对应的object
    void intialize(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& obs);

    // make polygons for render(ing)，输出供opencv使用的多边形
    void makePolygons4Render(Eigen::Vector2d obs, std::vector<std::vector<cv::Point>>& polygons) const;

    void externalOcclusion(Object& obj, Eigen::Vector2d obs);

    // 以一个edge进行投影，修改余下的edge
    // 内部三投影方式不修改堆
private:
    void externalProjector(Edge* const src, const Eigen::Vector2d& obs);

    template<bool SET_SRC = true>
    void projectEdge2Edge(Edge* const src, const Eigen::Vector2d& obs, Edge& dst);

    void breakEdge(Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d obs, Edge& dst, Object* const obj);

    // 根据光源位置，计算光线与线段的交点
    static Eigen::Vector2d getIntersection(
        const Eigen::Vector2d& vec,
        const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2, 
        const Eigen::Vector2d& obs
    );

public:
    bool valid;
    bool projected;
    double min_dist;
    std::vector<Edge> edges;
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompFunctor> heap;
    std::pair<double, double> angle_range;
};