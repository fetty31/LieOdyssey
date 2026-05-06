#pragma once

#include "gilda_lio/types.hpp"
#include "gilda_lio/state.hpp"
#include "gilda_lio/gauss_ivox.hpp"

namespace gilda_lio {

struct Match {

    using Scalar        = gauss_mapping::Scalar;
    using GaussPtr      = gauss_mapping::GaussPtr;
    using Point         = gauss_mapping::Point;
    using PrimitiveType = gauss_mapping::PrimitiveType;

    Eigen::Vector3f point, local_point;
    Eigen::Matrix<Scalar, 4, 1> coeffs = Eigen::Matrix<Scalar, 4, 1>::Zero(); // (a, b, c, d), normalized
    Scalar dist = std::numeric_limits<Scalar>::max(); // point-to-plane distance
    bool good_fit = false;

    Match(const Eigen::Vector3f& p, 
            const Eigen::Vector3f& bl_p, 
            const GaussPtr& gauss,
            const int min_points,
            const Scalar max_distance) 
        : point(p), local_point(bl_p)
    {
        if(gauss == nullptr) return; // safety check

        // Check closest gaussian is valid
        if( enough_points(gauss, min_points) && isPlanePrimitive(gauss) )
        {
            setPlaneEq(gauss);

            dist = getDist();

            good_fit = close_enough(dist, max_distance); // early match rejection
        }
    }

    Match() = default;

    bool lisanAlGaib(){ // whether is the chosen one :)
        return good_fit;
    }

    bool enough_points(const GaussPtr& g, const int& min_n) {
        return g->count >= min_n;
    }

    bool isPlanePrimitive(const GaussPtr& g){
        return g->type == PrimitiveType::PLANE;
    }

    Scalar getDist() {
        Point p = point.cast<Scalar>();
        return coeffs.head<3>().dot(p) + coeffs(3);
    }

    void setPlaneEq(const GaussPtr& g){
        auto n = g->buildNormalVector();
        Scalar norm = n.norm();
        coeffs(0) = n(0) / norm;
        coeffs(1) = n(1) / norm;
        coeffs(2) = n(2) / norm;
        coeffs(3) = g->param(2) / norm; 
    }

    bool close_enough(const Scalar& d, const Scalar& thres){
        return std::abs(d) < thres;
    }

};

class Map {
public:
    using MapType  = gauss_mapping::GaussianIVox;

    using Scalar   = gauss_mapping::Scalar;
    using Mat3     = gauss_mapping::Mat3;
    using GaussPtr = gauss_mapping::GaussPtr;
    using Point    = gauss_mapping::Point;
    using PointCov = gauss_mapping::pointWithCov;

    struct Config {
        int min_plane_points = 5;
        Scalar max_dist_thresh = 2.0;
        std::size_t max_num_pc2match = 10000;

        Scalar resolution = 0.5; 
        std::size_t update_thresh = 1; 
        std::size_t max_buffer_size = 50;
        Scalar planarity_thresh = 0.1;
        Scalar chi_square_thresh = 7.81;
        Scalar noise_floor = 0.001; 
        Scalar lidar_angle_noise = 0.001;
        Scalar lidar_range_noise = 0.00001;
    };

    Map(const int n_threads,
        const Map::Config& cfg ) 
        : num_threads_(n_threads), config_(cfg)
        { }

    bool exists(){
        return (this->map_ != nullptr);
    }

    int size(){
        if(exists()) return this->map_->size();
        else return 0;
    }

    const MapType* getMap() const {
        return map_.get();
    }

    void update(pcl::PointCloud<LioPointType>::Ptr& pc, State& s, const std::vector<double>& state_cov){
        if(pc->points.size() < 1) return;

        std::vector<PointCov> points;

        processPoints(pc, s, state_cov, points);

        if (points.empty()) return;

        // If map doesn't exists, build one
        if(not this->exists()){
            this->map_ = std::make_unique<MapType>(config_.resolution, 
                                                    config_.update_thresh,
                                                    config_.max_buffer_size,
                                                    config_.planarity_thresh,
                                                    config_.chi_square_thresh,
                                                    config_.noise_floor
                                                ); 
        }
        this->map_->update(points);   
    }

    std::vector<Match> match(State s, pcl::PointCloud<LioPointType>::Ptr& pc){
        std::vector<Match> matches;
        if(not this->exists()) return matches;

        int N0 = (pc->points.size() > config_.max_num_pc2match) ? pc->points.size() - config_.max_num_pc2match : 0;

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
    std::unique_ptr<MapType> map_;

    int num_threads_;
    Config config_;

    Match match_plane(Eigen::Vector3f& p, Eigen::Vector3f& bl_p) {

        // Find k nearest gaussians
        GaussPtr gauss = this->map_->getPrimitiveAtPoint(Point(p(0), p(1), p(2)));

        // Construct a gaussian match 
        return Match(p, bl_p, gauss, config_.min_plane_points, config_.max_dist_thresh);
    }


    /**
     * @brief Processes the input points to compute per-point covariance
     * @param pts Input point cloud (base_link/body frame w.r.t last propagated state (Xt2))
     * @param s State
     * @param state_cov Pose covariance assuming [position, orientation] order
     * @param out Output vector of valid points
     */
    void processPoints(const pcl::PointCloud<LioPointType>::Ptr& pts, 
                        State& s,
                        const std::vector<double>& state_cov,
                        std::vector<PointCov>& out) {

        if (state_cov.size() < 36) {
            throw std::runtime_error("gilda_lio::Mapper::state_cov too small for 6x6 matrix");
        }
        
        // Map covariance vector -> Eigen matrix
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> P_pose(state_cov.data());

        // Extract blocks
        Eigen::Matrix3d P_pos = P_pose.block<3,3>(0,0).cast<Scalar>();
        Eigen::Matrix3d P_rot = P_pose.block<3,3>(3,3).cast<Scalar>();

        out.clear();
        out.reserve(pts->size());
        for (const auto& pt : *pts) { 

            // --- Measurement covariance in body frame (sensor error modeling) ---
            Mat3 cov;
            gauss_mapping::calcBodyCov(
                Point(pt.x, pt.y, pt.z),
                config_.lidar_range_noise,
                config_.lidar_angle_noise,
                cov
            );

            // --- Transform point to world ---
            Eigen::Vector3f pt3 = pt.getVector3fMap();
            pt3 = s.get_transform() * pt3;

            Point p_world(pt3[0], pt3[1], pt3[2]);

            // --- Rotate measurement covariance ---
            Mat3 R = s.get_R().cast<Scalar>();
            Mat3 cov_world = R * cov * R.transpose();

            // --- Add rotation uncertainty (observer uncertainty) ---
            Mat3 skew;
            skew <<     0, -p_world.z(),  p_world.y(),
                    p_world.z(),     0, -p_world.x(),
                    -p_world.y(), p_world.x(),     0;

            cov_world += (-skew) * P_rot * (-skew).transpose();

            // --- Add translation uncertainty (observer uncertainty) ---
            cov_world += P_pos;

            // --- Store result ---
            out.push_back({p_world, cov_world});
        }
    }
};


} // namespace gilda_lio