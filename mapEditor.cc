#include <array>
#include "mapEdit.h"

std::vector<std::vector<cv::Point>> obsts;
std::vector<cv::Point> obstacle;
cv::Point start_point(0, 0);
bool update_flag = false, start_set = false;

bool pointInBound(const std::array<int, 4>& minmax, const cv::Point& pt) {
    
}

void on_mouse(int event, int x,int y, int flags, void *ustc) {
    if (event == cv::EVENT_LBUTTONUP) {
        if (flags == cv::EVENT_FLAG_SHIFTKEY) {
            start_set = false;
            cv::Point end_point(x, y);
            int max_x = 0, min_x = 0, max_y = 0, min_y = 0;
            if (start_point.x >= end_point.x) {
                min_x = end_point.x;
                max_x = start_point.x;
            } else {
                min_x = start_point.x;
                max_x = end_point.x;
            }
            if (start_point.y >= end_point.y) {
                min_y = end_point.y;
                max_y = start_point.y;
            } else {
                min_y = start_point.y;
                max_y = end_point.y;
            }
            for (std::vector<Obstacle>::iterator it = obsts.begin(); it != obsts.end();) {
                bool break_flag = false;
                for (size_t i = 0; i < it->size(); i++) {
                    const cv::Point& pt = it->at(i);
                    if (pt.x <= max_x && pt.x >= min_x && pt.y >= min_y && pt.y <= max_y) {
                        break_flag = true;
                        break;
                    }
                }
                if (break_flag == true) {
                    it = obsts.erase(it);
                } else {
                    it++;
                }
            }
        } else {
            obstacle.emplace_back(x, y);
            update_flag = true;
        }
    } else if (event == cv::EVENT_LBUTTONDOWN && flags == cv::EVENT_FLAG_SHIFTKEY) {
        start_set = true;
        start_point = cv::Point(x, y);
    }
}

int main(int argc, char** argv) {
    cv::Mat src(cv::Size(1000, 800), CV_8UC3);

    while (true) {
        char key = cv::waitKey(10);
        if (key == 27) {
            printf("Exiting directly, map not saved.\n");
            break;
        } else if (key == 'e') {

        } else if (key == 's') {

        } else if (key == 'd') {

        }
    }


    return 0;
}