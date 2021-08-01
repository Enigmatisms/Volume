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
    // simplePreVisualize(src, obs);
    for (size_t i = 0; i < objs.size(); i++)            // 构建堆
        heap.emplace(i);
    while (heap.empty() == false) {
        size_t obj_id = heap.top();
        Object& obj = objs[obj_id];
        heap.pop();
        LOG_ERROR("Object %d, started to internal project.", obj_id);
        obj.internalProjection(observing_point);
        LOG_MARK("Object %d, started to external project.", obj_id);
        for (Object& projectee: objs) {
            // 不查看完全被遮挡的，不投影已经投影过的
            if (projectee.valid == false || obj.id == projectee.id) continue;
            LOG_GAY("External occ, object %lu", projectee.id);
            // 选择projectee被投影
            obj.externalOcclusion(projectee, observing_point);
            LOG_CHECK("Projectee (%d) processed, edges (valid):", projectee.id);
            for (const Edge& eg: projectee.edges)
                printf("%d, ", eg.valid);
            printf("\n");
        }
        LOG_SHELL("After external proj, valids in object %d are:", obj_id);
        for (const Edge& eg: obj.edges)
            printf("%d, ", eg.valid);
        printf("\n");
    }
}

void Volume::simplePreVisualize(cv::Mat& src, const cv::Point& obs) const {
    for (const Object& obj: objs) {
        obj.visualizeEdges(src, obs);
        char str[8];
        snprintf(str, 8, "%d", obj.id);
		cv::putText(src, str, cv::Point(obj.edges.front().back().x(), obj.edges.front().back().y()) + cv::Point(10, 10),
				cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255));
    }
    cv::imshow("tmp", src);
    cv::waitKey(0);
}

