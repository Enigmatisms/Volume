#include <opencv2/imgproc.hpp>
#include "volume.hpp"

void Volume::calculateVisualSpace(const std::vector<Obstacle>& obstcs, cv::Point obs) {
    objs.clear();
    Eigen::Vector2d observing_point(obs.x, obs.y); 
    for (const Obstacle& obstacle: obstcs) {            // 构建objects
        objs.emplace_back();
        Object& obj = objs.back();
        obj.intialize(obstacle, observing_point);
    }
    for (size_t i = 0; i < objs.size(); i++)            // 构建堆
        heap.emplace(&objs[i]);
    while (heap.empty() == false) {
        Object* obj = heap.top();
        obj->projected = true;
        heap.pop();
        obj->internalProjection(observing_point);
        for (Object& projectee: objs) {
            // 不查看完全被遮挡的，不投影已经投影过的
            if (projectee.valid == false || projectee.projected == true) continue;
            obj->externalOcclusion(projectee, observing_point);
        }
    }
}

void Volume::visualizeVisualSpace(const Eigen::Vector2d& obs, cv::Mat& dst) const {
    std::vector<std::vector<cv::Point>> polygons;
    for (const Object& obj: objs)
        obj.makePolygons4Render(obs, polygons);
    for (const std::vector<cv::Point>& polygon: polygons) {
        cv::fillConvexPoly(dst, polygon, cv::Scalar(254, 254, 254));
    }
}
