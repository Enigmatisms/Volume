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
    HeapType heap(edges);
    for (size_t i = 0; i < edges.size(); i++)
        heap.emplace(i);
    while (heap.empty() == false) {
        size_t top = heap.top();
        Edge& this_edge = edges[top];
        heap.pop();
        if (this_edge.valid == false) continue;
        for (size_t i = 0; i < edges.size(); i++) {
            Edge& eg = edges[i];
            if (eg.valid == false)
                continue;
            LOG_INFO("Before proj (%lu), proj ids is: %d, %d", i, this_edge.proj_ids.first, this_edge.proj_ids.second);
            projectEdge2Edge(this_edge, obs, eg, heap);
            LOG_SHELL("After proj (%lu), proj ids is: %d, %d", i, this_edge.proj_ids.first, this_edge.proj_ids.second);
        }
        LOG_MARK_STREAM("After internal proj once, info:");
    }
}

void Object::externalOcclusion(Object& obj, Eigen::Vector2d obs) {
    for (size_t i = 0; i < edges.size(); i++) {
        Edge& eg = edges[i];
        LOG_CHECK("Other object is being occ by edge %lu, f: %d, s:%d", i, eg.proj_ids.first, eg.proj_ids.second);
        if (eg.valid == false) continue;
        if (eg.proj_ids.first < 0 && eg.proj_ids.second < 0) continue;
        // obj 是被本object选中的eg遮挡
        obj.externalProjector(eg, obs);
    }
}

// 以一个edge进行投影，修改余下的edge
// 内部三投影方式不修改堆
void Object::externalProjector(Edge& src, const Eigen::Vector2d& obs) {
    // 别的object产生的投影，也是需要使用到这样一个heap的，而pop操作会破怪heap
    printf("External projection started.\n");
    HeapType heap(edges);
    for (size_t i = 0; i < edges.size(); i++)
        heap.emplace(i);
    while (heap.empty() == false) {
        size_t top = heap.top();
        Edge& this_edge = edges[top];
        heap.pop();
        if (this_edge.valid == false) continue;
        int fx = this_edge.front().x(), fy = this_edge.front().y(), bx = this_edge.back().x(), by = this_edge.back().y();
        LOG_INFO("Edge from (%d, %d) to (%d, %d) in the object (%lu) is being shadowed.", fx, fy, bx, by, id);
        projectEdge2Edge(src, obs, this_edge, heap);
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
            break;
        }
    }
    if (valid_flag == false)                    // 假如全部被遮挡，那么这个object就没有投影的必要了
        valid = false;
}

// 根据观测位置，进行背面剔除，建立对应的object
void Object::intialize(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& obs) {
    edges.clear();
    Edge to_add;
    bool zero_pushed = false;
    for (size_t i = 1; i < pts.size(); i++) {
        Eigen::Vector2d vec = pts[i] - pts[i - 1];
        Eigen::Vector2d ctr_vec = (pts[i] + pts[i - 1]) / 2.0 - obs;
        Eigen::Vector2d norm = Eigen::Vector2d(-vec(1), vec(0));
        if (ctr_vec.dot(norm) < 0.0) {
            if (i == 1)
                zero_pushed = true;
            Eigen::Vector2d o2p = pts[i - 1] - obs;
            to_add.emplace_back();
            to_add.back() << pts[i - 1], atan2(o2p(1), o2p(0));
        } else {
            if (to_add.empty() == false) {
                Eigen::Vector2d o2p = pts[i - 1] - obs;
                to_add.emplace_back();
                to_add.back() << pts[i - 1], atan2(o2p(1), o2p(0));
                edges.push_back(to_add);
                to_add.reset();
            }
        }
    }
    Eigen::Vector2d vec = pts.front() - pts.back();
    Eigen::Vector2d ctr_vec = (pts.back() + pts.front()) / 2.0 - obs;
    Eigen::Vector2d norm = Eigen::Vector2d(-vec(1), vec(0));
    if (to_add.empty() == false) {
        Eigen::Vector2d o2p = pts.back() - obs;
        to_add.emplace_back();
        to_add.back() << pts.back(), atan2(o2p(1), o2p(0));
    }
    if (ctr_vec.dot(norm) < 0.0) {
        if (zero_pushed == true) {
            Edge& front = edges.front();
            if (to_add.empty() == true) {
                front.emplace_front();
                Eigen::Vector2d o2p = pts.back() - obs;
                front.front() << pts.back(), atan2(o2p(1), o2p(0));
            } else {
                for (Edge::const_reverse_iterator rit = to_add.crbegin(); rit != to_add.crend(); rit++)
                    front.push_front(*rit);
            }
        } else {
            Eigen::Vector2d o2p = pts.front() - obs;
            to_add.emplace_back();
            to_add.back() << pts.front(), atan2(o2p(1), o2p(0));
            edges.push_back(to_add);
        }
    } else if (to_add.empty() == false) {
        edges.push_back(to_add);
    }
    for (Edge& eg: edges) {
        for (const Eigen::Vector3d& pt: eg) {
            double dist = (pt.block<2, 1>(0, 0) - obs).norm();
            if (dist < eg.min_dist) eg.min_dist = dist;
        }
    }
    for (size_t i = 0; i < edges.size(); i++) {
        Edge& eg = edges[i];
        eg.proj_ids.first = 0;
        eg.proj_ids.second = static_cast<int>(eg.size()) - 1;
        eg.valid = true;
        if (eg.min_dist < min_dist)
            min_dist = eg.min_dist;
    }
    valid = true;
}

// make polygons for render(ing)，输出供opencv使用的多边形
void Object::makePolygons4Render(Eigen::Vector2d obs, std::vector<std::vector<cv::Point>>& polygons) const{
    for (const Edge& eg: edges) {
        if (eg.valid == false) continue;
        polygons.emplace_back();
        std::vector<cv::Point> back = polygons.back();
        back.emplace_back(obs.x(), obs.y());
        for (const Eigen::Vector3d& p: eg) {
            back.emplace_back(p.x(), p.y());
        }
    }
}

// 内部投影也需要考虑加edge了
void Object::projectEdge2Edge(const Edge& src, const Eigen::Vector2d& obs, Edge& dst, HeapType& heap) {
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
    if (point_num == 2) {
        Eigen::Vector2d fpt = src[first_id].block<2, 1>(0, 0) - obs, ept = src[second_id].block<2, 1>(0, 0) - obs;
        double f_ang = src[first_id].z(), e_ang = src[second_id].z();
        bool head_in_range = dst.angleInRange(f_ang), end_in_range = dst.angleInRange(e_ang);
        if (head_in_range && end_in_range) {        // 需要有加edge逻辑
            LOG_ERROR_STREAM("Breaking edge!");
            breakEdge(fpt, ept, obs, dst, heap);
            return;
        } else if (head_in_range) {
            beam = fpt;
            angle = f_ang;
        } else if (end_in_range) {
            beam = ept;
            angle = e_ang;
            pop_back_flag = false;          // pop_front
        } else {
            if (src.angleInRange(dst.front().z()) || src.angleInRange(dst.back().z()))
                dst.valid = false;
            return;
        }
    } else {
        // 大小角逻辑
        const Eigen::Vector2d o2s = dst.back().block<2, 1>(0, 0) - obs, o2f = dst.front().block<2, 1>(0, 0) - obs;
        if (can_proj.front() == true) {         // 小角度（逆时针）
            const Eigen::Vector2d src_o2f = src.front().block<2, 1>(0, 0) - obs;
            if (src_o2f.dot(o2f) <= 0.0)        // 反向光线
                return;
            const Eigen::Vector2d f2f = src.front().block<2, 1>(0, 0) - dst.front().block<2, 1>(0, 0),
                    f2s = src.back().block<2, 1>(0, 0) - dst.front().block<2, 1>(0, 0),
                    s2f = src.front().block<2, 1>(0, 0) - dst.back().block<2, 1>(0, 0);
            const Eigen::Vector2d nf2f(-f2f(1), f2f(0)), ns2f(-s2f(1), s2f(0)), nf2s(-f2s(1), f2s(0));
            if (nf2f.dot(o2f) >= 0) {           // src.first 超过 dst.first 可全覆盖
                if (nf2s.dot(o2f) < 0) {        // src.second 不超过 dst.first 真全覆盖
                    dst.valid = false;
                }
                return;
            } else if (ns2f.dot(o2s) < 0) {     // src.first 超过 dst.second
                return;
            }
            beam = src[first_id].block<2, 1>(0, 0) - obs;
            angle = src[first_id].z();
        } else if (can_proj.back() == true){
            const Eigen::Vector2d src_o2s = src.back().block<2, 1>(0, 0) - obs;
            if (src_o2s.dot(o2s) <= 0.0)        // 反向光线
                return;
            const Eigen::Vector2d s2s = src.back().block<2, 1>(0, 0) - dst.back().block<2, 1>(0, 0),
                    s2f = src.front().block<2, 1>(0, 0) - dst.back().block<2, 1>(0, 0),
                    f2s = src.back().block<2, 1>(0, 0) - dst.front().block<2, 1>(0, 0);
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
            beam = src[second_id].block<2, 1>(0, 0) - obs;
            angle = src[second_id].z();
            pop_back_flag = false;          // pop_front
        } else return;              // 完全没有关联的两个edges     
    }
    int id = dst.rotatedBinarySearch(angle);
    assert(id > 0);
    if (rangeSuitable(dst[id - 1], dst[id], beam, obs) == false) return;
    Eigen::Vector3d intersect = getIntersection(beam, dst[id - 1], dst[id], obs);
    if (pop_back_flag == true) {            // 删除大角度
        int delete_cnt = static_cast<int>(dst.size()) - id;
        // LOG_GAY("Deleting backs. delete cnt is %lu, size is %lu", delete_cnt, dst.size());
        for (int i = 0; i < delete_cnt; i++)
            dst.pop_back();
        dst.emplace_back(intersect);
        dst.proj_ids.second = -1;           // 大角度删除
    } else {
        for (int i = 0; i < id; i++) {
            dst.pop_front();
        }
        dst.emplace_front(intersect);
        dst.proj_ids.first = -1;            // 小角度删除
        if (dst.proj_ids.second >= 0)       // 尾部id发生变化
            dst.proj_ids.second = static_cast<int>(dst.size()) - 1;
    }
    for (const Eigen::Vector3d& pt: dst) {
        double dist = (pt.block<2, 1>(0, 0) - obs).norm();
        if (dist < dst.min_dist) dst.min_dist = dist;
    }
}

void Object::breakEdge(Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d obs, Edge& dst, HeapType& heap) {
    std::array<Eigen::Vector2d, 2> task = {b1, b2};
    std::vector<Eigen::Vector3d> crs;
    std::array<size_t, 2> ids = {0, 0};
    for (size_t i = 0; i < 2; i++) {
        const Eigen::Vector2d& beam = task[i];
        int id = dst.rotatedBinarySearch(dst.front().z());
        if (rangeSuitable(dst[id - 1], dst[id], beam, obs) == false) return;
        Eigen::Vector3d intersect = getIntersection(beam, dst[id - 1], dst[id], obs);
        crs.emplace_back(intersect);
        ids[i] = dst.size() - static_cast<size_t>(id);
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
    dst.proj_ids.second =  -1;
    for (const Eigen::Vector3d& pt: dst) {
        double dist = (pt.block<2, 1>(0, 0) - obs).norm();
        if (dist < dst.min_dist) dst.min_dist = dist;
    }
    edges.push_back(new_edge);
    heap.emplace(edges.size() - 1);              // 可能不太安全
}

void Object::visualizeEdges(cv::Mat& src, cv::Point obs) const{
    printf("Object: ");
    if (valid == false)
        return;
    int i = -1;
    for (const Edge& eg: edges) {
        i++;
        if (eg.valid == false) continue;
        printf("(%d, %d), ", int(eg.front().x()), int(eg.front().y()));
        for (size_t i = 1; i < eg.size(); i++) {
            cv::line(src, cv::Point(eg[i - 1].x(), eg[i - 1].y()), cv::Point(eg[i].x(), eg[i].y()), cv::Scalar(0, 255, 255), 3);
            printf("(%d, %d), ", int(eg[i].x()), int(eg[i].y()));
        }
        printf("\n");
        char str[4];
        snprintf(str, 4, "%d", i);
		cv::putText(src, str, cv::Point(eg.front().x(), eg.front().y()) + cv::Point(10, 10),
					cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
        cv::circle(src, cv::Point(eg.front().x(), eg.front().y()), 3, cv::Scalar(255, 255, 0), -1);
        cv::circle(src, cv::Point(eg.back().x(), eg.back().y()), 3, cv::Scalar(255, 0, 255), -1);
    }
    cv::circle(src, obs, 4, cv::Scalar(0, 255, 0), -1);
}

// 根据光源位置，计算光线与线段的交点
Eigen::Vector3d Object::getIntersection(
    const Eigen::Vector2d& vec,
    const Eigen::Vector3d& _p1,
    const Eigen::Vector3d& _p2, 
    const Eigen::Vector2d& obs
) {
    const Eigen::Vector2d p1 = _p1.block<2, 1>(0, 0), p2 = _p2.block<2, 1>(0, 0);
    const Eigen::Vector2d vec_line = p2 - p1;
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    A << -vec(1), vec(0), -vec_line(1), vec_line(0);
    double b1 = Eigen::RowVector2d(-vec(1), vec(0)) * obs;
    double b2 = Eigen::RowVector2d(-vec_line(1), vec_line(0)) * p1;
    const Eigen::Vector2d b(b1, b2);
    double det = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    if (std::abs(det) < 1e-5)
        return _p1;
    const Eigen::Vector2d pt = A.inverse() * b, new_vec = pt - obs;
    double angle = atan2(new_vec(1), new_vec(0));
    Eigen::Vector3d result;
    result << pt, angle;
    return result;                   // 解交点
}

bool Object::rangeSuitable(
    const Eigen::Vector3d& p1, 
    const Eigen::Vector3d& p2, 
    const Eigen::Vector2d& beam, 
    const Eigen::Vector2d& obs
) const {
    Eigen::Vector2d v1 = p1.block<2, 1>(0, 0) - obs, v2 = p2.block<2, 1>(0, 0) - obs;
    double mean_range = (v1.norm() + v2.norm()) / 2.0;
    return beam.norm() < mean_range;
}
