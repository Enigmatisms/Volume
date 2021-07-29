#include "Edge.hpp"

void Edge::initWithObs(Eigen::Vector2d obs) {
    proj_ids.first = -1;                            // 打断式投影的第二个edge中少一个投影点
    proj_ids.second = static_cast<int>(this->size()) - 1;
    Eigen::Vector2d front_beam = this->front() - obs, back_beam = this->back() - obs;
    angles.first = atan2(front_beam(1), front_beam(0));
    angles.second = atan2(back_beam(1), back_beam(0));
    int max_size = static_cast<int>(this->size()) - 1;
    min_dist = std::min(front_beam.norm(), back_beam.norm());
    for (int i = 1; i < max_size; i++) {
        Eigen::Vector2d vec = this->at(i) - obs;
        min_dist = std::min(vec.norm(), min_dist);   
    }
    valid = true;
    projected = false;
}