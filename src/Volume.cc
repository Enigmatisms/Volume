#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "Volume.hpp"
#include "LOG.hpp"

void Volume::visualizeVisualSpace(const std::vector<Obstacle>& _obstcs, const Eigen::Vector2d& obs, cv::Mat& dst) const {
    cv::Rect rect(0, 0, dst.cols, dst.rows);
    cv::rectangle(dst, rect, cv::Scalar(40, 40, 40), -1);
    std::vector<std::vector<cv::Point>> polygons;
    for (const Object& obj: objs)
        obj.makePolygons4Render(obs, polygons);
    for (const std::vector<cv::Point>& polygon: polygons)
        cv::fillConvexPoly(dst, polygon, cv::Scalar(254, 254, 254));
    for (const Obstacle& ob: _obstcs)
        cv::fillConvexPoly(dst, ob, cv::Scalar(0, 0, 0));
}

void Volume::calculateVisualSpace(const std::vector<Obstacle>& _obstcs, cv::Point obs, cv::Mat& src) {
    objs.clear();
    Eigen::Vector2d observing_point(obs.x, obs.y); 
    std::vector<std::vector<Eigen::Vector2d>> obstcs;
    for (const Obstacle& obstacle: _obstcs) {            // 构建objects
        obstcs.emplace_back();
        for (const cv::Point& pt: obstacle)
            obstcs.back().emplace_back(pt.x, pt.y);
    }
    int obj_cnt = 0;
    for (const std::vector<Eigen::Vector2d>& obstacle: obstcs) {            // 构建objects
        Object obj(obj_cnt);
        obj.intialize(obstacle, observing_point);
        objs.push_back(obj);
        obj_cnt++;
    }
    for (const Object& obj: objs) {
        obj.visualizeEdges(src, obs);
    }
    cv::imshow("tmp", src);
    cv::waitKey(0);
    for (size_t i = 0; i < objs.size(); i++)            // 构建堆
        heap.emplace(&objs[i]);
    while (heap.empty() == false) {
        Object* obj = heap.top();
        obj->projected = true;
        heap.pop();
        LOG_ERROR("Object %d, started to internal project.", obj->id);
        obj->internalProjection(observing_point);
        LOG_MARK("Object %d, started to external project.", obj->id);
        size_t i = 0;
        for (Object& projectee: objs) {
            // 不查看完全被遮挡的，不投影已经投影过的
            if (projectee.valid == false || projectee.projected == true) continue;
            LOG_GAY("External occ, object %lu", i);
            // 选择projectee被投影
            obj->externalOcclusion(projectee, observing_point);
            i++;
        }
    }
}
