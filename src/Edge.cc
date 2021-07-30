#include "Edge.hpp"
#include "LOG.hpp"

void Edge::initWithObs(Eigen::Vector2d obs) {
    LOG_CHECK("Init with obs, this address is: %x, size is %lu", this, this->size());
    proj_ids.first = -1;                            // 打断式投影的第二个edge中少一个投影点
    proj_ids.second = static_cast<int>(this->size()) - 1;
    Eigen::Vector2d front_beam = this->front().block<2, 1>(0, 0) - obs, back_beam = this->back().block<2, 1>(0, 0) - obs;
    int max_size = static_cast<int>(this->size()) - 1;
    min_dist = std::min(front_beam.norm(), back_beam.norm());
    for (int i = 1; i < max_size; i++) {
        Eigen::Vector2d vec = this->at(i).block<2, 1>(0, 0) - obs;
        min_dist = std::min(vec.norm(), min_dist);   
    }
    valid = true;
}

int Edge::rotatedBinarySearch(double angle) const{
    int s = 0, e = static_cast<int>(size()) - 1, m = static_cast<int>((s + e) / 2);
    double start_angle = front().z(), mid_angle = at(m).z();
    while (s < e) {
        if (mid_angle > start_angle) {
            s = m + 1;
            start_angle = mid_angle;
        } else if (mid_angle < start_angle){
            e = m;
        } else {
            break;
        }
        m = static_cast<int>((s + e) / 2);
        mid_angle = at(m).z();
    }
    m = static_cast<int>((s + e) / 2);
    if (m != static_cast<int>(size()) - 1) {
        if (angle >= front().z()) {
            int id = binarySearch(std::make_pair(0, m), angle);
            if (id == -2) return m;
            return id;
        } 
        int id = binarySearch(std::make_pair(m, static_cast<int>(size()) - 1), angle);
        if (id == -1) return m;
        return id;
    }
    else {
        return binarySearch(std::make_pair(0, m), angle);
    }
}
int Edge::binarySearch(std::pair<int, int> range, double angle) const{
    int s = range.first, e = range.second, m = static_cast<int>((s + e) / 2);
    if (angle < at(s).z()) return -1;
    else if (angle > at(e).z()) return -2;
    double mid_angle = at(m).z();
    while (s < e) {
        if (mid_angle > angle) {
            e = m;
        } else if (mid_angle < angle) {
            s = m + 1;
        } else {
            return std::max(range.first, m);
        }
        m = static_cast<int>((s + e) / 2);
        mid_angle = at(m).z();
    } 
    return std::max(range.first, s);
}
