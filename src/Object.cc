#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include "Object.hpp"

const double PI = 3.141592653589793238;
 
// 内部投影需要确定每条valid边在内部投影之后是否可以进行外部投影（ids有非负的吗）
// 内部投影先做，做完之后可以确定每个object有谁投影
void Object::internalProjection(const Eigen::Vector2d& obs) {
    // 每一次被选中投影之后，自身队列没有必要存在了
    /// @todo: heap没能成功关联 
    printf("Internal proj, edges size: %lu\n", edges.size());
    while (heap.empty() == false) {
        size_t top = heap.top();
        Edge& this_edge = edges[top];
        heap.pop();
        if (this_edge.valid == false) continue;
        this_edge.projected = true;
        size_t i = 0;
        for (Edge& eg: edges) {
            if (eg.valid == false || eg.projected == true) {
                i++;
                continue;
            }
            LOG_INFO("Before proj (%lu), proj ids is: %d, %d", i, this_edge.proj_ids.first, this_edge.proj_ids.second);
            projectEdge2Edge(this_edge, obs, eg);
            LOG_SHELL("After proj (%lu), proj ids is: %d, %d", i, this_edge.proj_ids.first, this_edge.proj_ids.second);
            i++;
        }
    }
}

void Object::externalOcclusion(Object& obj, Eigen::Vector2d obs) {
    for (size_t i = 0; i < edges.size(); i++) {
        Edge& eg = edges[i];
        if (eg.valid == false) continue;
        if (eg.proj_ids.first < 0 && eg.proj_ids.second < 0) continue;
        LOG_CHECK("Other object is being occ by edge %lu, ", i);
        // obj 是被本object选中的eg遮挡
        obj.externalProjector(eg, obs);
    }
}

// 以一个edge进行投影，修改余下的edge
// 内部三投影方式不修改堆
void Object::externalProjector(Edge& src, const Eigen::Vector2d& obs) {
    // 别的object产生的投影，也是需要使用到这样一个heap的，而pop操作会破怪heap
    while (heap.empty() == false) {
        size_t top = heap.top();
        Edge& this_edge = edges[top];
        heap.pop();
        if (this_edge.valid == false) continue;
        int fx = this_edge.front().x(), fy = this_edge.front().y(), bx = this_edge.back().x(), by = this_edge.back().y();
        LOG_INFO("Edge from (%d, %d) to (%d, %d) in the object (%lu) is being shadowed.", fx, fy, bx, by, id);
        projectEdge2Edge<false>(src, obs, this_edge);
        LOG_MARK("After shadowing, this edge: %d, all others:", int(this_edge.valid));
        for (size_t i = 0; i < edges.size(); i++) {
            printf("%d, ", int(edges[i].valid));
        }
        printf("\n");
    }
    // 需要重新建堆
    bool valid_flag = false;
    for (size_t i = 0; i < edges.size(); i++) {
        if (edges[i].valid == true) {
            valid_flag = true;
            heap.emplace(i);
        }
    }
    if (valid_flag == false)                    // 假如全部被遮挡，那么这个object就没有投影的必要了
        valid = false;
}

// 根据观测位置，进行背面剔除，建立对应的object
void Object::intialize(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& obs) {
    edges.clear();
    LOG_SHELL("Before init, functor size: %lu, address: %x", functor.egs.size(), &functor.egs);
    while (heap.empty() == false) heap.pop();
    Edge to_add;
    bool zero_pushed = false;
    for (size_t i = 1; i < pts.size(); i++) {
        Eigen::Vector2d vec = pts[i] - pts[i - 1];
        Eigen::Vector2d ctr_vec = (pts[i] + pts[i - 1]) / 2.0 - obs;
        Eigen::Vector2d norm = Eigen::Vector2d(-vec(1), vec(0));
        if (ctr_vec.dot(norm) < 0.0) {
            if (i == 1)
                zero_pushed = true;
            to_add.min_dist = std::min(to_add.min_dist, (pts[i - 1] - obs).norm());
            to_add.push_back(pts[i - 1]);
        } else {
            if (to_add.empty() == false) {
                to_add.push_back(pts[i - 1]);
                edges.push_back(to_add);
                to_add.reset();
            }
        }
    }
    Eigen::Vector2d vec = pts.front() - pts.back();
    Eigen::Vector2d ctr_vec = (pts.back() + pts.front()) / 2.0 - obs;
    Eigen::Vector2d norm = Eigen::Vector2d(-vec(1), vec(0));
    if (to_add.empty() == false) {
        to_add.emplace_back(pts.back());
        to_add.min_dist = std::min(to_add.min_dist, (pts.back() - obs).norm());
    }
    if (zero_pushed && ctr_vec.dot(norm) < 0.0) {
        Edge& front = edges.front();
        for (Edge::const_reverse_iterator rit = to_add.crbegin(); rit != to_add.crend(); rit++)
            front.push_front(*rit);
        front.min_dist = std::min(front.min_dist, to_add.min_dist);
    } else if (to_add.empty() == false) {
        edges.push_back(to_add);
    }
    for (size_t i = 0; i < edges.size(); i++) {
        Edge& eg = edges[i];
        eg.proj_ids.first = 0;
        eg.proj_ids.second = static_cast<int>(eg.size()) - 1;
        Eigen::Vector2d sbeam = eg.front() - obs;
        eg.angles.first = atan2(sbeam.y(), sbeam.x());
        Eigen::Vector2d ebeam = eg.back() - obs;
        eg.angles.second = atan2(ebeam.y(), ebeam.x());
        eg.valid = true;
        projected = false;
        if (eg.min_dist < min_dist)
            min_dist = eg.min_dist;
        heap.emplace(i);
    }
    LOG_SHELL("After init (1), functor size: %lu, address: %x", functor.egs.size(), &functor.egs);
    LOG_ERROR("Heap size: %lu", heap.size());

    valid = true;
    projected = false;
}

// make polygons for render(ing)，输出供opencv使用的多边形
void Object::makePolygons4Render(Eigen::Vector2d obs, std::vector<std::vector<cv::Point>>& polygons) const{
    for (const Edge& eg: edges) {
        if (eg.valid == false) continue;
        polygons.emplace_back();
        std::vector<cv::Point> back = polygons.back();
        back.emplace_back(obs.x(), obs.y());
        for (const Eigen::Vector2d& p: eg) {
            back.emplace_back(p.x(), p.y());
        }
    }
}

// 内部投影也需要考虑加edge了
template<bool SET_SRC>
void Object::projectEdge2Edge(Edge& src, const Eigen::Vector2d& obs, Edge& dst) {
    LOG_CHECK("Edge(%x) is being processed. (%d, %d), (%d, %d)", &dst, dst.proj_ids.first, dst.proj_ids.second, src.proj_ids.first, src.proj_ids.second);
    int first_id = src.proj_ids.first, second_id = src.proj_ids.second, point_num = 0;
    std::array<bool, 2> can_proj = {false, false};
    if (first_id >= 0)  {
        can_proj[0] = true;
        point_num++;
    }    
    if (second_id >= 0) {
        can_proj[1] = true;
        point_num++;  
    }
    bool pop_back_flag = true;
    Eigen::Vector2d beam = Eigen::Vector2d::Zero();
    double angle = 0.0;
    // 需要判定，主投影edge的两个点有几个点在被遮挡edge的内部
    // 如果由两个，则有加边逻辑，如果只有一个，则无需加边
    if (point_num == 2) {
        Eigen::Vector2d fpt = src[first_id] - obs, ept = src[second_id] - obs;
        double f_ang = atan2(fpt.y(), fpt.x()), e_ang = atan2(ept.y(), ept.x());
        bool head_in_range = dst.angleInRange(f_ang), end_in_range = dst.angleInRange(e_ang);
        if (head_in_range && end_in_range) {        // 需要有加edge逻辑
            if (SET_SRC) {
                src.proj_ids.first = -1;
                src.proj_ids.second = -1;
            }
            LOG_ERROR_STREAM("Breaking edge!");
            breakEdge(fpt, ept, obs, dst);
            return;
        } else if (head_in_range) {
            beam = fpt;
            angle = f_ang;
            if (SET_SRC) src.proj_ids.first = -1;
        } else if (end_in_range) {
            beam = ept;
            angle = e_ang;
            if (SET_SRC) src.proj_ids.second = -1;
            pop_back_flag = false;          // pop_front
        } else return;
    } else {
        // 大小角逻辑
        const Eigen::Vector2d o2s = dst.back() - obs, o2f = dst.front() - obs;
        if (can_proj.front() == true) {         // 小角度（逆时针）
            const Eigen::Vector2d src_o2f = src.front() - obs;
            if (src_o2f.dot(o2f) <= 0.0)        // 反向光线
                return;
            const Eigen::Vector2d f2f = src.front() - dst.front(), f2s = src.back() - dst.front(), s2f = src.front() - dst.back();
            const Eigen::Vector2d nf2f(-f2f(1), f2f(0)), ns2f(-s2f(1), s2f(0)), nf2s(-f2s(1), f2s(0));
            if (nf2f.dot(o2f) >= 0) {           // src.first 超过 dst.first 可全覆盖
                if (nf2s.dot(o2f) < 0) {        // src.second 不超过 dst.first 真全覆盖
                    dst.valid = false;
                }
                return;
            } else if (ns2f.dot(o2s) < 0) {     // src.first 超过 dst.second
                return;
            }
            beam = src[first_id] - obs;
            angle = atan2(beam.y(), beam.x());
            if (SET_SRC) src.proj_ids.first = -1;
        } else if (can_proj.back() == true){
            const Eigen::Vector2d src_o2s = src.back() - obs;
            if (src_o2s.dot(o2s) <= 0.0)        // 反向光线
                return;
            const Eigen::Vector2d s2s = src.back() - dst.back(), s2f = src.front() - dst.back(), f2s = src.back() - dst.front();
            const Eigen::Vector2d ns2s(-s2s(1), s2s(0)), ns2f(-s2f(1), s2f(0)), nf2s(-f2s(1), f2s(0));        // 求法向量
            if (ns2s.dot(o2s) < 0) {            // src.second 超过 dst.second 可能全覆盖
                if (ns2f.dot(o2s) >= 0) {        // src.first 不超过 dst.second 说明真全覆盖
                    dst.valid = false;
                } 
                return;
            } else if (nf2s.dot(o2f) >= 0) {    // src.second 不超过 dst.first 完全无关
                return;
            }
            // 超界问题
            beam = src[second_id] - obs;
            angle = atan2(beam.y(), beam.x());
            if (SET_SRC) src.proj_ids.second = -1;
            pop_back_flag = false;          // pop_front
        } else return;              // 完全没有关联的两个edges     
    }
    size_t id = 1;
    const Eigen::Vector2d& fpt = dst.front() - obs;
    bool break_flag = false;                        /// TODO: 删除 debug使用
    double last_angle = atan2(fpt.y(), fpt.x()), this_angle = 0.0;
    for (id = 1; id < dst.size(); id++) {
        Eigen::Vector2d vec = dst[id] - obs;
        this_angle = atan2(vec.y(), vec.x());
        if (this_angle < 0.0 && last_angle > 0.0) {        // 奇异角度
            if (angle <= this_angle || angle >= last_angle ) {
                break_flag = true;
                break;
            }
        } else {
            if (angle <= this_angle && angle >= last_angle) {
                break_flag = true;
                break;
            }
        }
        printf("%lu, ang: %.6lf, last: %.6lf, now: %.6lf\n", id, angle, last_angle, this_angle);
        last_angle = this_angle;
    }
    assert(break_flag == true);
    /// 交点计算的逻辑检查
    Eigen::Vector2d intersect = getIntersection(beam, dst[id - 1], dst[id], obs);
    if (pop_back_flag == true) {            // 删除大角度
        size_t delete_cnt = dst.size() - id;
        // LOG_GAY("Deleting backs. delete cnt is %lu, size is %lu", delete_cnt, dst.size());
        for (size_t i = 0; i < delete_cnt; i++)
            dst.pop_back();
        dst.emplace_back(intersect);
        dst.proj_ids.second = -1;           // 大角度删除
        dst.angles.second = angle;
    } else {
        int j = 0;
        (j == 1);
        for (size_t i = 0; i < id; i++) {
            dst.pop_front();
        }
        dst.emplace_front(intersect);
        dst.proj_ids.first = -1;            // 小角度删除
        if (dst.proj_ids.second >= 0)       // 尾部id发生变化
            dst.proj_ids.second = static_cast<int>(dst.size()) - 1;
        dst.angles.first = angle;
    }
}

void Object::breakEdge(Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d obs, Edge& dst) {
    std::array<Eigen::Vector2d, 2> task = {b1, b2};
    std::vector<Eigen::Vector2d> crs;
    std::array<size_t, 2> ids = {0, 0};
    for (size_t i = 0; i < 2; i++) {
        const Eigen::Vector2d& beam = task[i];
        size_t id = 1;
        const Eigen::Vector2d& fpt = dst.front();
        bool break_flag = false;                        /// TODO: 删除 debug使用
        double last_angle = atan2(fpt.y(), fpt.x()), this_angle = 0.0, angle = atan2(beam.y(), beam.x());
        for (id = 1; id < dst.size(); id++) {
            Eigen::Vector2d vec = dst[id] - obs;
            this_angle = atan2(vec.y(), vec.x());
            if (this_angle < 0.0 && last_angle > 0.0) {        // 奇异角度
                if (angle <= this_angle || angle >= last_angle ) {
                    break_flag = true;
                    break;
                }
            } else {
                if (angle <= this_angle && angle >= last_angle) {
                    break_flag = true;
                    break;
                }
            }
            LOG_CHECK("Last angle: %lf, this angle: %lf", last_angle, this_angle);
            last_angle = this_angle;
        }
        assert(break_flag == true);
        Eigen::Vector2d intersect = getIntersection(beam, dst[id - 1], dst[id], obs);
        crs.emplace_back(intersect);
        ids[i] = dst.size() - id;
    }
    Edge new_edge;
    size_t add_cnt = 0;
    for (Edge::const_reverse_iterator rit = dst.crbegin(); rit != dst.crend() && add_cnt < ids[1]; rit++, add_cnt++)
        new_edge.push_front(*rit);
    new_edge.push_front(crs[1]);
    new_edge.initWithObs(obs);
    for (size_t i = 0; i < ids[0]; i++)
        dst.pop_back();
    dst.push_back(crs[0]);
    dst.angles.second = atan2(task[0].y(), task[0].x());
    dst.proj_ids.second =  -1;
    edges.push_back(new_edge);
    heap.emplace(edges.size() - 1);              // 可能不太安全
}

void Object::visualizeEdges(cv::Mat& src, cv::Point obs) const{
    printf("Object: ");
    if (valid == false)
        return;
    for (const Edge& eg: edges) {
        if (eg.valid == false) continue;
        printf("(%d, %d), ", int(eg.front().x()), int(eg.front().y()));
        for (size_t i = 1; i < eg.size(); i++) {
            cv::line(src, cv::Point(eg[i - 1].x(), eg[i - 1].y()), cv::Point(eg[i].x(), eg[i].y()), cv::Scalar(0, 255, 255), 3);
            printf("(%d, %d), ", int(eg[i].x()), int(eg[i].y()));
        }
        printf("\n");
        cv::circle(src, cv::Point(eg.front().x(), eg.front().y()), 3, cv::Scalar(255, 255, 0), -1);
        cv::circle(src, cv::Point(eg.back().x(), eg.back().y()), 3, cv::Scalar(255, 0, 255), -1);
    }
    cv::circle(src, obs, 4, cv::Scalar(0, 255, 0), -1);
}

// 根据光源位置，计算光线与线段的交点
Eigen::Vector2d Object::getIntersection(
    const Eigen::Vector2d& vec,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2, 
    const Eigen::Vector2d& obs
) {
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d vec_line = p2 - p1;
    A << -vec(1), vec(0), -vec_line(1), vec_line(0);
    double b1 = Eigen::RowVector2d(-vec(1), vec(0)) * obs;
    double b2 = Eigen::RowVector2d(-vec_line(1), vec_line(0)) * p1;
    Eigen::Vector2d b(b1, b2);
    double det = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    if (std::abs(det) < 1e-5)
        return p1;
    return A.inverse() * b;                   // 解交点
}
