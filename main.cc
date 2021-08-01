#include "Volume.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "mapEdit.h"

cv::Mat src;
cv::Point obs;
bool obs_set = false;

void on_mouse(int event, int x,int y, int flags, void *ustc) {
    if (event == cv::EVENT_LBUTTONDOWN && obs_set == false) {
        printf("cv::Point(%d, %d),\n", x, y);
        obs.x = x;
        obs.y = y;
        cv::circle(src, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
        obs_set = true;
    }
}

int main(int argc, char** argv) {
    std::vector<std::vector<cv::Point>> obstacles;
    std::string name = "test";
    if (argc < 2) {
        std::cerr << "Usage: ./main <Map name> <optional: int, whether to specify the view point.>\n";
        return -1;
    }
    name = std::string(argv[1]);
    mapLoad("../maps/" + name + ".txt", obstacles);
    src.create(cv::Size(1200, 900), CV_8UC3);
    cv::drawContours(src, obstacles, -1, cv::Scalar(80, 80, 80), -1);
    for (const Obstacle& egs: obstacles) {
        cv::circle(src, egs.front(), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(src, egs.back(), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("disp", on_mouse, NULL);
    Volume vol;
    obs = cv::Point(610, 368);
    // obs = cv::Point(532, 84);
    bool use_fixed = false;
    if (argc > 2) 
        use_fixed = true;
    if (use_fixed) {
        while (obs_set == false) {
            cv::imshow("disp", src);
            char key = cv::waitKey(10);
            if (key == 27)
                return 0;
        }
    }
    vol.calculateVisualSpace(obstacles, obs, src);
    vol.simplePreVisualize(src, obs);
    // vol.visualizeVisualSpace(obstacles, Eigen::Vector2d(obs.x, obs.y), src);
    cv::imshow("disp", src);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}