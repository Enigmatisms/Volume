#include "mapEdit.h"

const cv::Rect rect(0, 0, 1000, 800);

void mapDraw(const std::vector<Obstacle>& obstalces, cv::Mat& src) {
    cv::rectangle(src, rect, cv::Scalar(100, 100, 100), -1);
    int size = static_cast<int>(obstalces.size()) - 1;
    for (int i = 0; i < size; i++) {
        cv::drawContours(src, obstalces, i, cv::Scalar(255, 0, 0), 3);
        for (const cv::Point& pt: obstalces[i]) {
            cv::circle(src, pt, 2, cv::Scalar(0, 0, 0), -1);
        }
    }
    cv::drawContours(src, obstalces, size, cv::Scalar(255, 0, 0), 3);
    for (const cv::Point& pt: obstalces.back()) {
        cv::circle(src, pt, 2, cv::Scalar(0, 0, 0), -1);
    }
}

void mapLoad(std::string path, std::vector<std::vector<cv::Point>>& obstacles) {

}

void mapSave(const std::vector<std::vector<cv::Point>>& obstacles, std::string path) {

}