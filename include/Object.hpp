#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "Edge.hpp"
#include "LOG.hpp"

/// @ref https://stackoverflow.com/questions/8372918/c-std-list-sort-with-custom-comparator-that-depends-on-an-member-variable-for 
struct EdgeCompFunctor{
    EdgeCompFunctor(const std::vector<Edge>& _egs): egs(_egs) {
        LOG_GAY("Constructing functor: egs size: %lu, address: %x", _egs.size(), &egs);
    }
    const std::vector<Edge>& egs;
    bool operator() (const size_t& e1, const size_t& e2) const {
        return egs[e1].min_dist > egs[e2].min_dist;
    }
};

// 物体定义，物体内部有很多edge
class Object {
using HeapType = std::priority_queue<size_t, std::vector<size_t>, EdgeCompFunctor>;
public:
    Object(int _id = 0): id(_id){
        min_dist = 1e9;
    }
    ~Object() {}
public:
    // 内部投影之后，一个object内部最多还有两条可以投影的edge，只需要遍历即可
    void internalProjection(const Eigen::Vector2d& obs);

    // 根据观测位置，进行背面剔除，建立对应的object
    void intialize(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& obs);

    void visualizeEdges(cv::Mat& src, cv::Point obs) const;

    // make polygons for render(ing)，输出供opencv使用的多边形
    void makePolygons4Render(Eigen::Vector2d obs, std::vector<std::vector<cv::Point>>& polygons) const;

    void externalOcclusion(Object& obj, Eigen::Vector2d obs);

    // 以一个edge进行投影，修改余下的edge
    // 内部三投影方式不修改堆
private:
    void externalProjector(Edge& src, const Eigen::Vector2d& obs);

    /// @brief returns whether a false choice is being operated
    void projectEdge2Edge(const Edge& src, const Eigen::Vector2d& obs, Edge& dst, HeapType& heap);

    void breakEdge(Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d obs, Edge& dst, HeapType& heap);

    // 根据光源位置，计算光线与线段的交点
    static Eigen::Vector2d getIntersection(
        const Eigen::Vector2d& vec,
        const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2, 
        const Eigen::Vector2d& obs
    );

public:
    int id;
    bool valid;
    bool projected;
    double min_dist;
    std::vector<Edge> edges;
    std::pair<double, double> angle_range;
};