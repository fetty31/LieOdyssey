#include "lidar_odometry_ros/modules/mapper.hpp"

// class lidar_odometry_ros::Mapper
    // public

        Mapper::Mapper() : last_map_time(-1.), num_threads_(1){

            // Init cfg values
            this->config.NUM_MATCH_POINTS = 5;
            this->config.MAX_NUM_PC2MATCH = 1.e+4;
            this->config.MAX_DIST_PLANE   = 2.0;
            this->config.PLANE_THRESHOLD  = 5.e-2;
        }

        void Mapper::set_num_threads(int n){
            if(n < 1) return;
            this->num_threads_ = n;
        }

        void Mapper::set_config(const Config::Mapping& cfg){
            this->config = cfg;
        }
                
        bool Mapper::exists(){
            return (this->map_ != nullptr);
        }

        int Mapper::size(){
            if(exists()) return this->map_->size();
            else return 0;
        }

        void Mapper::clear_matches(){
            this->matches.clear();
        }

        double Mapper::last_time(){
            return this->last_map_time;
        }

        Matches Mapper::match(State s, pcl::PointCloud<PointType>::Ptr& pc){

            if(not this->exists()) return matches;

            // if(this->matches.size() > 0) {
            //     return this->matches; // if matches from last iter exist, return them (avoid matching every iter, which is costly)
            // }

            std::size_t N0 = (pc->points.size() > static_cast<std::size_t>(config.MAX_NUM_PC2MATCH)) ? 
                                static_cast<std::size_t>(pc->points.size()) - config.MAX_NUM_PC2MATCH : 0;

            Matches init_matches;
            init_matches.resize(pc->points.size()-N0);

            #pragma omp parallel for num_threads(this->num_threads_)
            for(std::size_t i = 0; i < pc->points.size()-N0; i++){
                
                Eigen::Vector4f bl4_point(pc->points[i].x, pc->points[i].y, pc->points[i].z, 1.); // base link 4d point
                Eigen::Vector4f global_point = s.get_RT() * bl4_point;                            // global 4d point == [x', y', z', 1.0]
                Match match = this->match_plane(global_point, bl4_point);                         // point-to-plane match

                init_matches[i] = match; 
            }

            Matches chosen_matches;
            for(std::size_t j = 0; j < init_matches.size(); j++){
                if(init_matches[j].lisanAlGaib())
                    chosen_matches.push_back(init_matches[j]); // if match is chosen, push it
            }

            this->matches = chosen_matches; // save matches for next iter
            return chosen_matches;
        }
        
        void Mapper::add(pcl::PointCloud<PointType>::Ptr& pc, double time){
            if(pc->points.size() < 1) return;

            // If map doesn't exists, build one
            if(not this->exists()){
                this->map_ = std::make_unique<octree::Octree>(
                                static_cast<size_t>(this->config.octree.bucket_size), 
                                false,
                                static_cast<float>(this->config.octree.min_extent)
                            ); 
                this->map_->initialize(pc);
            }        
            else this->map_->update(pc);

            this->last_map_time = time;
        }

    // private

        Match Mapper::match_plane(Eigen::Vector4f& p, Eigen::Vector4f& p_local) {

            // Find k nearest points
            std::vector<float> pointSearchSqDis;
            std::vector<pcl::PointXYZ> neighbors;

            this->map_->knn(pcl::PointXYZ(p(0), p(1), p(2)),
                    this->config.NUM_MATCH_POINTS,
                    neighbors,
                    pointSearchSqDis);
            
            MapPoints near_points(neighbors.begin(), neighbors.end());
            // Construct a plane fitting between them
            return Match( p.head(3), p_local.head(3), Plane (near_points, pointSearchSqDis, &config) );
        }