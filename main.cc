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
    std::vector<cv::Point> obstacle = {cv::Point(364, 345),
cv::Point(429, 365),
cv::Point(498, 363),
cv::Point(549, 360),
cv::Point(613, 341),
cv::Point(608, 310),
cv::Point(564, 308),
cv::Point(517, 314),
cv::Point(441, 315),
cv::Point(409, 291),
cv::Point(405, 247),
cv::Point(442, 213),
cv::Point(501, 181),
cv::Point(590, 177),
cv::Point(692, 197),
cv::Point(761, 238),
cv::Point(796, 279),
cv::Point(811, 330),
cv::Point(816, 393),
cv::Point(809, 434),
cv::Point(786, 478),
cv::Point(740, 527),
cv::Point(679, 571),
cv::Point(545, 636),
cv::Point(480, 663),
cv::Point(491, 675),
cv::Point(567, 662),
cv::Point(620, 648),
cv::Point(727, 598),
cv::Point(811, 532),
cv::Point(870, 369),
cv::Point(825, 214),
cv::Point(695, 125),
cv::Point(482, 107),
cv::Point(332, 210),
cv::Point(335, 301),
    };




//     std::vector<cv::Point> obstacle = {cv::Point(612, 288),
// cv::Point(544, 348),
// cv::Point(377, 469),
// cv::Point(360, 631),
// cv::Point(396, 737),
// cv::Point(450, 652),
// cv::Point(473, 563),
// cv::Point(548, 472),
// cv::Point(619, 413),
// cv::Point(664, 353),
// cv::Point(677, 292),
// cv::Point(654, 274),
//     };
    obstacles.emplace_back(obstacle);
    obstacle.clear();


//     obstacle = {cv::Point(420, 127),
// cv::Point(355, 213),
// cv::Point(424, 212),
// cv::Point(344, 310),
// cv::Point(401, 298),
// cv::Point(445, 270),
// cv::Point(488, 247),
// cv::Point(523, 214),
// cv::Point(560, 165),
// cv::Point(569, 139),
// cv::Point(510, 148),
// cv::Point(474, 153),
// cv::Point(506, 98),
// cv::Point(440, 114),
//     };
//     obstacles.emplace_back(obstacle);
//     obstacle.clear();
//     obstacle = {cv::Point(828, 207),
// cv::Point(836, 243),
// cv::Point(869, 255),
// cv::Point(928, 239),
// cv::Point(939, 198),
// cv::Point(898, 155),
// cv::Point(815, 153),
//     };
//     obstacles.emplace_back(obstacle);
//     obstacle.clear();






//     obstacle = {cv::Point(525, 134),
// cv::Point(526, 153),
// cv::Point(540, 157),
// cv::Point(558, 145),
// cv::Point(580, 140),
// cv::Point(606, 144),
// cv::Point(642, 164),
// cv::Point(667, 197),
// cv::Point(689, 243),
// cv::Point(695, 285),
// cv::Point(687, 342),
// cv::Point(671, 387),
// cv::Point(644, 440),
// cv::Point(616, 489),
// cv::Point(596, 535),
// cv::Point(574, 593),
// cv::Point(563, 660),
// cv::Point(576, 733),
// cv::Point(602, 716),
// cv::Point(631, 632),
// cv::Point(654, 557),
// cv::Point(701, 471),
// cv::Point(742, 364),
// cv::Point(743, 281),
// cv::Point(734, 185),
// cv::Point(661, 120),
// cv::Point(563, 106)};
//     obstacles.emplace_back(obstacle);
//     obstacle.clear();





//     obstacle = {cv::Point(165, 593),
// cv::Point(182, 654),
// cv::Point(252, 652),
// cv::Point(270, 590),
// cv::Point(217, 532)};
//     obstacles.emplace_back(obstacle);

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
    // std::vector<cv::Point> obstacle = {cv::Point(460, 501), cv::Point(551, 525), cv::Point(725, 489), cv::Point(646, 454), cv::Point(735, 402), cv::Point(645, 353), cv::Point(740, 310), cv::Point(644, 277), cv::Point(752, 228), cv::Point(681, 174), cv::Point(538, 175), cv::Point(424, 220), cv::Point(493, 272), cv::Point(394, 287), cv::Point(494, 344), cv::Point(397, 380), cv::Point(470, 424), cv::Point(393, 467),};
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();

    // std::vector<cv::Point> obstacle = {cv::Point(936, 526), cv::Point(967, 552), cv::Point(969, 604), cv::Point(896, 622), cv::Point(839, 603), cv::Point(856, 651), cv::Point(833, 686), cv::Point(763, 681), cv::Point(729, 668), cv::Point(749, 725), cv::Point(851, 765), cv::Point(980, 734), cv::Point(1049, 676), cv::Point(1034, 591), cv::Point(981, 540)};
    // obstacles.emplace_back(obstacle);
    // obstacle.clear();
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
    obs = cv::Point(165, 339);
    // obs = cv::Point(416, 512);
    // obs = cv::Point(648, 254);
    // obs = cv::Point(452, 472);
    while (obs_set == false) {
        cv::imshow("disp", src);
        char key = cv::waitKey(10);
        if (key == 27)
            return 0;
    }
    vol.calculateVisualSpace(obstacles, obs, src);
    vol.simplePreVisualize(src, obs);
    // vol.visualizeVisualSpace(obstacles, Eigen::Vector2d(obs.x, obs.y), src);
    cv::imshow("disp", src);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}