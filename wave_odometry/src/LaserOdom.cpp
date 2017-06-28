#include <type_traits>
#include "LaserOdom.hpp"

namespace wave {

float LaserOdom::l2sqrd(const PointType &p1, const PointType &p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

float LaserOdom::l2sqrd(const PointType &pt) {
    return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
}

PointType LaserOdom::scale(const PointType &pt, const float scale) {
    PointType p;
    p.x = pt.x * scale;
    p.y = pt.y * scale;
    p.z = pt.z * scale;
    p.intensity = pt.intensity;
    return p;
}

LaserOdom::LaserOdom(const LaserOdomParams params) : param(params) {
    this->cur_scan.reserve(param.n_ring);
    this->cur_curve.reserve(this->param.n_ring);
    this->filter.reserve(this->param.n_ring);
}

void LaserOdom::addPoints(std::vector<PointXYZIR> pts,
                          const int tick,
                          const TimeType stamp) {
    if (tick == 0) {  // Start of a new scan
        if (this->initialized) {
            this->undistort();
            this->rollover(stamp);
        } else {
            this->initialized = true;
        }
    }
    for (PointXYZIR pt : pts) {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        this->cur_scan.at(pt.ring).push_back(this->applyIMU(p, tick));
    }
}

PointType LaserOdom::applyIMU(const PointType &p, int tick) {
    // for now don't transform
    PointType pt;
    pt = p;
    return pt;
}

void LaserOdom::rollover(const TimeType stamp) {
    this->prv_time = this->cur_time;
    this->prv_scan.swap(this->cur_scan);
    this->cur_time = stamp;
    this->resetIMU(stamp);
    for (int i = 0; i < this->param.n_ring; i++) {
        this->cur_curve.at(i).clear();
        this->cur_scan.at(i).clear();
    }
}

void LaserOdom::undistort() {
    this->generateFeatures();
    this->match();
}

void LaserOdom::generateFeatures() {
    this->computeCurvature();
    this->prefilter();
    for (int i = 0; i < this->param.n_ring; i++) {
        std::sort(this->filter.at(i).begin(),
                  this->filter.at(i).end(),
                  [](const std::pair<unlong, float> lhs,
                     const std::pair<unlong, float> rhs) {
                      return lhs.second < rhs.first;
                  });
    }
}

void LaserOdom::computeCurvature() {
    for (int i = 0; i < this->param.n_ring; i++) {
        auto n_pts = this->cur_scan.at(i).size();
        this->cur_curve.at(i).resize(n_pts);
        for (unlong j = this->param.knn; j < n_pts - this->param.knn; j++) {
            float diffX = 0, diffY = 0, diffZ = 0;
            for (unlong k = 1; k <= this->param.knn; k++) {
                diffX += this->cur_scan.at(i).at(j + k).x +
                         this->cur_scan.at(i).at(j - k).x;
                diffY += this->cur_scan.at(i).at(j + k).y +
                         this->cur_scan.at(i).at(j - k).y;
                diffZ += this->cur_scan.at(i).at(j + k).z +
                         this->cur_scan.at(i).at(j - k).z;
            }
            diffX -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).x;
            diffY -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).y;
            diffZ -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).z;
            this->cur_curve.at(i).at(j) = std::make_pair(
              true, diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }
}

// This part filters out points that will not provide salient features. See
// paper for details
void LaserOdom::prefilter() {
    for (int i = 0; i < this->param.n_ring; i++) {
        auto n_pts = this->cur_curve.at(i).size();
        auto max_pts = n_pts - this->param.knn;
        for (unlong j = this->param.knn; j < max_pts; j++) {
            auto delforward = this->l2sqrd(this->cur_scan.at(i).at(j),
                                           this->cur_scan.at(i).at(j + 1));
            auto delback = this->l2sqrd(this->cur_scan.at(i).at(j),
                                        this->cur_scan.at(i).at(j - 1));
            // First section excludes any points who's score is likely caused
            // by occlusion
            if (delforward > this->param.occlusion_tol) {
                float d1 = std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j)));
                float d2 =
                  std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j + 1)));
                auto unit1 = this->scale(this->cur_scan.at(i).at(j), 1 / d1);
                auto unit2 =
                  this->scale(this->cur_scan.at(i).at(j + 1), 1 / d2);
                auto diff = std::sqrt(this->l2sqrd(unit1, unit2));
                if (diff < this->param.occlusion_tol) {
                    if (d1 > d2) {
                        // todo(ben) See if this <= is actually correct
                        for (unlong k = 0; k <= this->param.knn; k++) {
                            this->cur_curve.at(i).at(j - k).first = false;
                        }
                    } else {
                        // todo(ben) See if this <= is actually correct
                        for (unlong k = 1; k <= this->param.knn + 1; k++) {
                            this->cur_curve.at(i).at(j + k).first = false;
                        }
                    }
                }
            }
            // This section excludes any points whose nearby surface is
            // near to parallel to the laser
            auto dis = this->l2sqrd(this->cur_scan.at(i).at(j),
                                    this->cur_scan.at(i).at(j - 1));
            if ((delforward > (this->param.parallel_tol) * dis)
                && (delback > (this->param.parallel_tol * dis))) {
                    this->cur_curve.at(i).at(j).first = false;
                }
        }
        // now store each selected point for sorting
        this->filter.at(i).resize(max_pts);
        unlong cnt = 0;
        for (unlong j = 0; j < n_pts; j++) {
            if (this->cur_curve.at(i).at(j).first) {
                this->filter.at(i).at(cnt).first = j;
                this->filter.at(i).at(cnt).second =
                  this->cur_curve.at(i).at(j).second;
                cnt++;
            }
        }
        this->filter.at(i).resize(cnt);
    }
}

}  // namespace wave