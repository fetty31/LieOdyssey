#pragma once

#include "lio_ros/types.hpp"
#include "lio_ros/state.hpp"
#include "lio_ros/octree.hpp"

namespace lio_ros {

struct Plane {
    Eigen::Vector4f coeffs; // (a, b, c, d), normalized
    bool is_plane;

    Plane() : coeffs(Eigen::Vector4f::Zero()), is_plane(false) {}

    explicit Plane(const MapPoints& pts, const float& thres) {
        estimate_plane(pts);
        is_plane = plane_eval(pts, thres);
    }

    float distance(const Eigen::Vector3f& p) const {
        return coeffs.head<3>().dot(p) + coeffs(3);
    }

    bool plane_eval(const MapPoints& pts, const float& thres){
        for (std::size_t i = 0; i < pts.size(); i++) {
            Eigen::Vector3f p(pts[i].x, pts[i].y, pts[i].z);
            if (std::fabs(distance(p)) > thres) return false;
        }
        return true;
    }

    void estimate_plane(const MapPoints& pts){
        int NUM_MATCH_POINTS = pts.size();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(NUM_MATCH_POINTS, 3);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b(NUM_MATCH_POINTS, 1);
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            A(j,0) = pts[j].x;
            A(j,1) = pts[j].y;
            A(j,2) = pts[j].z;
        }

        Eigen::Matrix<float, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

        float n = normvec.norm();
        coeffs(0) = normvec(0) / n;
        coeffs(1) = normvec(1) / n;
        coeffs(2) = normvec(2) / n;
        coeffs(3) = 1.0 / n;
    }
};

struct Match {

    Eigen::Vector3f point, local_point;
    Plane plane;

    Match(const Eigen::Vector3f& p, const Eigen::Vector3f& bl_p, const Plane& H) 
        : point(p), local_point(bl_p), plane(H) {}
    Match() = default;

    bool lisanAlGaib(){ // whether is the chosen one :)
        return plane.is_plane;
    }

};

class Map {
public:
    Map(const int n_threads,
        const int n_plane_points,
        const float threshold,
        const std::size_t max_num_matches,
        const int octree_bucket_size,
        const float octree_extent) 
        : num_threads_(n_threads),
        min_plane_points_(n_plane_points),
        thres_(threshold),
        max_num_pc2match_(max_num_matches) 
        {
            octree_.setBucketSize(octree_bucket_size);
            octree_.setMinExtent(octree_extent);
            octree_.setDownsample(false);
        }

    bool exists(){
        return octree_.num_points_ > 0;
    }

    int size(){
        return octree_.num_points_;
    }

    void update(pcl::PointCloud<LioPointType>::Ptr& pc){
        if(pc->points.size() < 1) return;

        // If map doesn't exists, build one
        if(!this->exists()) this->octree_.initialize(pc);       
        else this->octree_.update(pc);
    }

    std::vector<Match> match(State s, pcl::PointCloud<LioPointType>::Ptr& pc){
        std::vector<Match> matches;
        if(not this->exists()) return matches;

        int N0 = (pc->points.size() > max_num_pc2match_) ? pc->points.size() - max_num_pc2match_ : 0;

        std::vector<Match> init_matches;
        init_matches.resize(pc->points.size()-N0);

        #pragma omp parallel for num_threads(this->num_threads_)
        for(std::size_t i = 0; i < pc->points.size()-N0; i++){
            
            Eigen::Vector3f bl_point(pc->points[i].x, pc->points[i].y, pc->points[i].z); // base link point
            Eigen::Vector3f global_point = s.get_transform() * bl_point;                        // global point
            Match match = this->match_plane(global_point, bl_point);                     // point-to-plane match

            init_matches[i] = match; 
        }

        for(std::size_t j = 0; j < init_matches.size(); j++){
            if(init_matches[j].lisanAlGaib())
                matches.push_back(init_matches[j]); // if match is chosen, push it
        }

        return matches;
    }

private:
    octree::Octree octree_;
    int num_threads_;
    int min_plane_points_;
    float thres_;
    std::size_t max_num_pc2match_;

    Match match_plane(Eigen::Vector3f& p, Eigen::Vector3f& bl_p) {
        // Find k nearest points
        std::vector<float> pointSearchSqDis;
        MapPoints neighbors;

        this->octree_.knn(MapPoint(p(0), p(1), p(2)),
                min_plane_points_,
                neighbors,
                pointSearchSqDis);
        
        // Construct a plane fitting between them
        return Match( p, bl_p, Plane(neighbors, thres_) );
    }
};


} // namespace lio_ros