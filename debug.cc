#include "Volume.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat src;

void on_mouse(int event, int x,int y, int flags, void *ustc) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        printf("cv::Point(%d, %d),\n", x, y);
        cv::circle(src, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
    }
}

int main(int argc, char** argv) {
    std::vector<std::vector<cv::Point>> obstacles;
    std::vector<cv::Point> obstacle = {cv::Point(262, 167), cv::Point(216, 195), cv::Point(188, 223),
        cv::Point(182, 271), cv::Point(218, 260), cv::Point(249, 258), cv::Point(292, 281),
        cv::Point(243, 312), cv::Point(190, 342), cv::Point(170, 383), cv::Point(186, 448),
        cv::Point(234, 433), cv::Point(296, 398), cv::Point(345, 348), cv::Point(415, 385),
        cv::Point(449, 415), cv::Point(507, 350), cv::Point(489, 291), cv::Point(537, 259),
        cv::Point(557, 300), cv::Point(602, 251), cv::Point(607, 227), cv::Point(582, 195),
        cv::Point(552, 182), cv::Point(503, 192), cv::Point(442, 206), cv::Point(406, 209),
        cv::Point(454, 175), cv::Point(505, 148), cv::Point(511, 122), cv::Point(480, 85),
        cv::Point(428, 95), cv::Point(367, 112), cv::Point(319, 135)
    };
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(835, 245), cv::Point(856, 282), cv::Point(862, 320),
        cv::Point(822, 330), cv::Point(759, 332), cv::Point(758, 291), cv::Point(759, 244),
        cv::Point(733, 273), cv::Point(724, 322), cv::Point(735, 381), cv::Point(761, 423),
        cv::Point(821, 436), cv::Point(888, 428), cv::Point(943, 408), cv::Point(978, 374),
        cv::Point(992, 314), cv::Point(1008, 252), cv::Point(1005, 184), cv::Point(973, 176),
        cv::Point(938, 160), cv::Point(893, 149), cv::Point(841, 186), cv::Point(839, 215)
    };
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(209, 694), cv::Point(246, 677), cv::Point(312, 691),
        cv::Point(321, 721), cv::Point(325, 769), cv::Point(344, 793), cv::Point(368, 798),
        cv::Point(319, 809), cv::Point(270, 784), cv::Point(215, 762), cv::Point(179, 727),
        cv::Point(133, 684), cv::Point(143, 634), cv::Point(189, 627), cv::Point(189, 657)
    };
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(539, 562), cv::Point(528, 593), cv::Point(560, 635), cv::Point(603, 620),
        cv::Point(628, 578), cv::Point(667, 550), cv::Point(648, 520), cv::Point(614, 522),
        cv::Point(579, 521), cv::Point(559, 525)
    };
    obstacle = {cv::Point(672, 118), cv::Point(693, 142), cv::Point(730, 138), cv::Point(760, 106),
        cv::Point(746, 78), cv::Point(700, 64), cv::Point(669, 76)};
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(1001, 664), cv::Point(893, 683), cv::Point(884, 718),
        cv::Point(907, 761), cv::Point(940, 758), cv::Point(970, 728), cv::Point(1003, 714),
        cv::Point(1052, 679), cv::Point(1039, 616), cv::Point(998, 588), cv::Point(955, 571),
        cv::Point(938, 590), cv::Point(976, 638)
    };
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    obstacle = {cv::Point(570, 821), cv::Point(613, 793), cv::Point(663, 784), cv::Point(688, 807),
        cv::Point(682, 842), cv::Point(639, 839), cv::Point(592, 843), cv::Point(554, 842)
    };
    obstacles.emplace_back(obstacle);
    obstacle.clear();
    printf("Obstacle initialized, obstacle size: %lu\n", obstacles.size());
    src.create(cv::Size(1200, 900), CV_8UC3);
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("disp", on_mouse, NULL);
    while (true) {
        cv::imshow("disp", src);
        char key = cv::waitKey(10);
        if (key == 27)
            break;
        else if (key == ' ') {
            printf("\n");
        }
    }
    return 0;
}