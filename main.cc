#include "Volume.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    // std::vector<cv::Point> obstacle = {cv::Point(256, 129), cv::Point(189, 154), cv::Point(145, 215), cv::Point(138, 264), cv::Point(146, 356), cv::Point(230, 280), cv::Point(254, 318), cv::Point(246, 387), cv::Point(269, 447), cv::Point(339, 390), cv::Point(411, 358), cv::Point(482, 324), cv::Point(449, 300), cv::Point(388, 316), cv::Point(447, 249), cv::Point(367, 263), cv::Point(451, 192), cv::Point(369, 189), cv::Point(327, 165), cv::Point(340, 125), cv::Point(300, 121)
    // };
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();
    // obstacle = {cv::Point(774, 308), cv::Point(777, 333), cv::Point(802, 364), cv::Point(853, 345), cv::Point(920, 333), cv::Point(969, 363), cv::Point(986, 401), cv::Point(948, 436), cv::Point(929, 476), cv::Point(974, 473), cv::Point(1062, 434), cv::Point(1061, 384), cv::Point(1023, 318), cv::Point(971, 281), cv::Point(840, 218), cv::Point(790, 217), cv::Point(760, 245), 
    // };
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();
    // obstacle = {cv::Point(582, 435), cv::Point(555, 464), cv::Point(561, 522), cv::Point(599, 521), cv::Point(668, 503), cv::Point(718, 471), cv::Point(672, 447), cv::Point(637, 437)
    // };
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();
    // obstacle = {cv::Point(202, 646), cv::Point(192, 674), cv::Point(196, 755), cv::Point(325, 780), cv::Point(360, 745), cv::Point(327, 690), cv::Point(329, 682), cv::Point(384, 661), cv::Point(418, 681), cv::Point(466, 738), cv::Point(471, 800), cv::Point(513, 810), cv::Point(525, 717), cv::Point(501, 659), cv::Point(434, 613), cv::Point(324, 588), cv::Point(246, 597)
    // };
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();
    // obstacle = {cv::Point(752, 645), cv::Point(770, 719), cv::Point(796, 781), cv::Point(821, 814), cv::Point(839, 833), cv::Point(848, 770), cv::Point(860, 740), cv::Point(889, 757), cv::Point(916, 790), cv::Point(944, 811), cv::Point(967, 826), cv::Point(1003, 837), cv::Point(991, 752), cv::Point(978, 681), cv::Point(954, 616), cv::Point(928, 586), cv::Point(893, 633), cv::Point(861, 670), cv::Point(821, 649), cv::Point(778, 610), cv::Point(760, 621)};
    // obstacles.emplace_back(obstacle);
    std::vector<cv::Point> obstacle = {cv::Point(467, 469), cv::Point(422, 531), cv::Point(468, 578), cv::Point(546, 589), cv::Point(633, 564), cv::Point(651, 513), cv::Point(583, 472)};
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(217, 271), cv::Point(226, 360), cv::Point(316, 365), cv::Point(389, 362), cv::Point(491, 370), cv::Point(580, 368), cv::Point(674, 370), cv::Point(773, 365), cv::Point(835, 347), cv::Point(854, 259), cv::Point(827, 203), cv::Point(742, 215), cv::Point(685, 269), cv::Point(631, 191), cv::Point(581, 247), cv::Point(504, 177), cv::Point(439, 256), cv::Point(272, 150), cv::Point(227, 228)};
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    printf("Obstacle initialized, obstacle size: %lu\n", obstacles.size());

    src.create(cv::Size(1200, 900), CV_8UC3);
    cv::drawContours(src, obstacles, -1, cv::Scalar(80, 80, 80), -1);
    for (const Obstacle& egs: obstacles) {
        cv::circle(src, egs.front(), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(src, egs.back(), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("disp", on_mouse, NULL);
    Volume vol;
    obs = cv::Point(568, 171);
    // obs = cv::Point(416, 512);
    // obs = cv::Point(648, 254);
    // obs = cv::Point(452, 472);
    // while (obs_set == false) {
    //     cv::imshow("disp", src);
    //     char key = cv::waitKey(10);
    //     if (key == 27)
    //         return 0;
    // }
    vol.calculateVisualSpace(obstacles, obs, src);
    vol.simplePreVisualize(src, obs);
    // vol.visualizeVisualSpace(obstacles, Eigen::Vector2d(obs.x, obs.y), src);
    cv::imshow("disp", src);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}