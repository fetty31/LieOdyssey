#pragma once

#include <random>

#include "gilda_lio/odometry_core.hpp"

#include "rclcpp/rclcpp.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ros_visuals {

Eigen::Vector3f getTurboColor(float t) {
    const float kR[] = {0.153f, 1.822f, -9.726f, 21.099f, -19.537f, 6.221f};
    const float kG[] = {0.022f, 1.415f, -2.439f, -3.130f, 6.936f, -2.771f};
    const float kB[] = {0.483f, 1.257f, 3.415f, -19.247f, 24.321f, -9.320f};
    auto eval = [&](const float* k, float t) {
        return k[0] + t*(k[1] + t*(k[2] + t*(k[3] + t*(k[4] + t*k[5]))));
    };
    return Eigen::Vector3f(eval(kR, t), eval(kG, t), eval(kB, t)).cwiseMax(0.0f).cwiseMin(1.0f);
}

Eigen::Vector3f getHighContrastColor(float t) {
    // A simplified "Magma/Inferno" style: Purple -> Red -> Orange -> Yellow
    Eigen::Vector3f color;
    if (t < 0.25f) {
        float f = t / 0.25f;
        color << 0.1f, 0.0f, 0.3f + 0.5f * f; // Dark Purple to Blue-ish
    } else if (t < 0.5f) {
        float f = (t - 0.25f) / 0.25f;
        color << 0.1f + 0.7f * f, 0.0f, 0.8f - 0.3f * f; // Purple to Red
    } else if (t < 0.75f) {
        float f = (t - 0.5f) / 0.25f;
        color << 0.8f + 0.2f * f, 0.4f * f, 0.5f * (1.0f - f); // Red to Orange
    } else {
        float f = (t - 0.75f) / 0.25f;
        color << 1.0f, 0.4f + 0.6f * f, 0.0f; // Orange to Yellow/White
    }
    return color;
}

Eigen::Matrix3d transformMinimalToLocal(
    const Eigen::Matrix3d& global_cov, 
    const gauss_mapping::Point& center,
    int axis_idx) // 0: (ny,nz,d), 1: (nx,nz,d), 2: (nx,ny,d)
{
    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
    
    // Select which coordinates of the center affect the offset 'd'
    // based on which normal components are in the covariance.
    if (axis_idx == 2) {      // ny, nz
        J(2, 0) = center.y(); 
        J(2, 1) = center.z();
    } else if (axis_idx == 1) { // nx, nz
        J(2, 0) = center.x();
        J(2, 1) = center.z();
    } else {                    // nx, ny
        J(2, 0) = center.x();
        J(2, 1) = center.y();
    }

    return J * global_cov * J.transpose();
}

visualization_msgs::msg::Marker makePlaneMarker(
    const gauss_mapping::GaussianPrimitive& g,
    const gauss_mapping::Point& voxel_center,
    const double resolution,
    int id,
    const rclcpp::Time& stamp,
    const std::string frame_id,
    std::tuple<float,float,float> color = {0.f,0.f,0.f}) 
{

    using Point = gauss_mapping::Point;

    visualization_msgs::msg::Marker m;

    const double plane_size = resolution;
    const double thickness = 0.002;

    // Compute plane center and normal
    Point normal = g.buildNormalVector();
    double norm2 = normal.squaredNorm();
    Point p0 = voxel_center; 

    Point p_plane = p0 - normal * (normal.dot(p0) + g.param[2]) / norm2;

    normal.normalize();
    if (normal.norm() < 1e-6) normal = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d z_axis = normal;

    // Pick a WORLD axis that is least aligned with the normal
    Eigen::Vector3d ref_axis;
    if (std::abs(z_axis.x()) <= std::abs(z_axis.y()) &&
        std::abs(z_axis.x()) <= std::abs(z_axis.z())) {
        ref_axis = Eigen::Vector3d::UnitX();
    }
    else if (std::abs(z_axis.y()) <= std::abs(z_axis.z())) {
        ref_axis = Eigen::Vector3d::UnitY();
    }
    else {
        ref_axis = Eigen::Vector3d::UnitZ();
    }

    // Build orthonormal basis
    Eigen::Vector3d x_axis = ref_axis.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = normal;

    Eigen::Quaterniond q(R);

    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "gilda_planes";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = p_plane.x();
    m.pose.position.y = p_plane.y();
    m.pose.position.z = p_plane.z();

    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.scale.x = plane_size; 
    m.scale.y = plane_size;
    m.scale.z = thickness;
    // m.scale.x = plane_size * sqrt(g.cov(0,0)); // spread along x-axis
    // m.scale.y = plane_size * sqrt(g.cov(1,1)); // spread along y-axis
    // m.scale.z = sqrt(g.cov(2,2)); // spread along z-axis

    m.color.r = std::get<0>(color);
    m.color.g = std::get<1>(color);
    m.color.b = std::get<2>(color);
    m.color.a = 0.8f;

    m.lifetime = rclcpp::Duration(0, 0);

    return m;
}

visualization_msgs::msg::Marker makePlaneUncertaintyMarker(
    const gauss_mapping::GaussianPrimitive& g,
    const gauss_mapping::Point& voxel_center,
    const double resolution,
    int id,
    const rclcpp::Time& stamp,
    const std::string frame_id) 
{

    auto m = makePlaneMarker(g, voxel_center, resolution, id, stamp, frame_id);
    m.ns = "gilda_uncertain_plane";

    // Get the normal and global offset
    gauss_mapping::Point normal = g.buildNormalVector(); // Make sure this is normalized
    auto d_global = g.param(2);

    // Project voxel center onto the plane to get the anchor point
    // Formula: x_p = x_v - (n·x_v + d) * n
    auto dist = normal.dot(voxel_center) + d_global;
    gauss_mapping::Point x_plane = voxel_center - dist * normal;

    auto local_cov = transformMinimalToLocal(g.plane_cov, x_plane, g.main_dir);

    Eigen::SelfAdjointEigenSolver<gauss_mapping::Mat3> es(local_cov);
    double uncertainty = sqrt(es.eigenvalues().maxCoeff());

    // Convert to Log-Space
    double u_log = std::log10(uncertainty + 1e-12);

    static const double display_min = -1.5; // log10(0.01)
    static const double display_max = 1.0;  // log10(10)

    double t = (u_log - display_min) / (display_max - display_min);
    t = std::clamp(t, 0.0, 1.0);

    // Apply a Power Curve to "stretch" the low values
    // This makes the difference between 0.1 and 0.5 much more visible
    float t_warped = std::pow(static_cast<float>(t), 0.7f); 

    Eigen::Vector3f rgb = getHighContrastColor(t_warped);

    m.color.r = rgb.x();
    m.color.g = rgb.y();
    m.color.b = rgb.z();
    m.color.a = 1.0;

    return m;
}

visualization_msgs::msg::Marker makeVolumeMarker(
    const gauss_mapping::GaussianPrimitive& g,
    int id,
    const rclcpp::Time& stamp,
    const std::string frame_id,
    std::tuple<float,float,float>& color
    ) 
{
    visualization_msgs::msg::Marker m;

    const double k_sigma = 1.0;

    Eigen::SelfAdjointEigenSolver<gauss_mapping::Mat3> es(g.cov);
    if (es.info() != Eigen::Success) return m;

    Eigen::Vector3d evals = es.eigenvalues();
    Eigen::Matrix3d evecs = es.eigenvectors();

    constexpr double EPS = 1e-12;
    evals = evals.cwiseMax(EPS);

    if (evecs.determinant() < 0.0)
        evecs.col(0) *= -1.0;

    Eigen::Quaterniond q(evecs);

    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "gilda_volumes";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = g.mean.x();
    m.pose.position.y = g.mean.y();
    m.pose.position.z = g.mean.z();

    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.scale.x = 2.0 * k_sigma * std::sqrt(evals(0));
    m.scale.y = 2.0 * k_sigma * std::sqrt(evals(1));
    m.scale.z = 2.0 * k_sigma * std::sqrt(evals(2));

    m.color.r = std::get<0>(color);
    m.color.g = std::get<1>(color);
    m.color.b = std::get<2>(color);
    m.color.a = 0.8f;

    m.lifetime = rclcpp::Duration(0, 0);

    return m;
}

visualization_msgs::msg::MarkerArray makeGaussianMarkers(
    const gauss_mapping::GaussianIVox& ivox,
    const rclcpp::Time& stamp,
    const std::string frame_id)
{
    visualization_msgs::msg::MarkerArray arr;

    std::mt19937 gen(12345); // fixed seed for consistent cluster colors
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    std::unordered_map<gauss_mapping::UnionFindNode*, std::tuple<float,float,float>> parent_colors;
    std::unordered_map<gauss_mapping::UnionFindNode*, std::vector<Bonxai::CoordT>> clusters;

    // Build clusters based on Union-Find parents and assign colors using Bonxai cell loop
    const auto& map = ivox.getRawGrid();
    map.forEachCell([&clusters, &parent_colors, &gen, &dis](const std::unique_ptr<gauss_mapping::UnionFindNode>& cell, const Bonxai::CoordT& coord) {
        if (cell) {
            gauss_mapping::UnionFindNode* root = cell->find();
            clusters[root].push_back(coord);
            if (parent_colors.find(root) == parent_colors.end()) {
                parent_colors[root] = {dis(gen), dis(gen), dis(gen)};
            }
        }
    });

    auto resolution = ivox.getVoxelResolution();

    // Visualize each voxel Gaussian primitive
    int id = 0;
    for (auto& [root, voxels] : clusters) {

        auto& gaus = root->gauss_ptr;
        if (!gaus || gaus->count <= 0) continue; // safety check and skip empty primitives

        for (const auto& coord : voxels) {

        gauss_mapping::Point voxel_center;
        // You can also use Bonxai's native position conversion:
        // auto pos = map.coordToPos(coord);
        // voxel_center << pos.x, pos.y, pos.z;
        voxel_center[0] = (coord.x + 0.5) * resolution;
        voxel_center[1] = (coord.y + 0.5) * resolution;
        voxel_center[2] = (coord.z + 0.5) * resolution;

        visualization_msgs::msg::Marker m;

        switch (gaus->type) {
            case gauss_mapping::PrimitiveType::PLANE:
                m = makePlaneMarker(*gaus, voxel_center, resolution, id++, stamp, frame_id, parent_colors[root]);
                arr.markers.push_back(m);
                break;

            case gauss_mapping::PrimitiveType::VOLUME:
                m = makeVolumeMarker(*gaus, id++, stamp, frame_id, parent_colors[root]);
                arr.markers.push_back(m);
                break;

            default:
                break;
        }
    }
    }
    
    return arr;
}

// visualization_msgs::msg::MarkerArray makeGaussianMarkers(
//     const gauss_mapping::GaussianIVox& ivox,
//     const rclcpp::Time& stamp,
//     const std::string frame_id)
// {
//     visualization_msgs::msg::MarkerArray arr;

//     std::mt19937 gen(12345); // fixed seed for consistent cluster colors
//     std::uniform_real_distribution<float> dis(0.0f, 1.0f);
//     std::unordered_map<gauss_mapping::UnionFindNode*, std::tuple<float,float,float>> parent_colors;
//     std::unordered_map<gauss_mapping::UnionFindNode*, std::vector<Eigen::Vector3i>> clusters;

//     // Build clusters based on Union-Find parents and assign colors
//     for (const auto& [key, node] : ivox.map_) {
//         auto root = node->find();
//         clusters[root].push_back(key);
//         if (parent_colors.find(root) == parent_colors.end()) {
//             parent_colors[root] = {dis(gen), dis(gen), dis(gen)};
//         }
//     }

//     auto resolution = ivox.getVoxelResolution();

//     // Visualize each voxel Gaussian primitive
//     int id = 0;
//     for (auto& [root, voxels] : clusters) {

//         auto& gaus = root->gauss_ptr;
//         if (!gaus) continue; // safety check, should always have a Gaussian

//         for (const auto& key : voxels) {

//             gauss_mapping::Point voxel_center;
//             voxel_center[0] = (key.x() + 0.5) * resolution;
//             voxel_center[1] = (key.y() + 0.5) * resolution;
//             voxel_center[2] = (key.z() + 0.5) * resolution;

//             visualization_msgs::msg::Marker m;

//             switch (gaus->type) {
//                 case gauss_mapping::PrimitiveType::PLANE:
//                     m = makePlaneMarker(*gaus, voxel_center, resolution, id++, stamp, frame_id, parent_colors[root]);
//                     arr.markers.push_back(m);
//                     break;

//                 case gauss_mapping::PrimitiveType::VOLUME:
//                     m = makeVolumeMarker(*gaus, id++, stamp, frame_id, parent_colors[root]);
//                     arr.markers.push_back(m);
//                     break;

//                 default:
//                     break;
//             }

//         }

//     }
    
//     return arr;
// }

visualization_msgs::msg::MarkerArray makeUncertaintyMarkers(
    const gauss_mapping::GaussianIVox& ivox,
    const rclcpp::Time& stamp,
    const std::string frame_id)
{
    visualization_msgs::msg::MarkerArray arr;

    std::unordered_map<gauss_mapping::UnionFindNode*, std::vector<Bonxai::CoordT>> clusters;

    // Build clusters based on Union-Find parents using Bonxai cell loop
    const auto& map = ivox.getRawGrid();
    map.forEachCell([&clusters](const std::unique_ptr<gauss_mapping::UnionFindNode>& cell, const Bonxai::CoordT& coord) {
        if (cell) {
            gauss_mapping::UnionFindNode* root = cell->find();
            clusters[root].push_back(coord);
        }
    });

    auto resolution = ivox.getVoxelResolution();

    // Visualize each voxel Gaussian primitive
    int id = 0;
    for (auto& [root, voxels] : clusters) {

        auto& gaus = root->gauss_ptr;
        if (!gaus || gaus->count <= 0) continue; 

        for (const auto& coord : voxels) {

        gauss_mapping::Point voxel_center;
        voxel_center[0] = (coord.x + 0.5) * resolution;
        voxel_center[1] = (coord.y + 0.5) * resolution;
        voxel_center[2] = (coord.z + 0.5) * resolution;

        visualization_msgs::msg::Marker m;

        switch (gaus->type) {
            case gauss_mapping::PrimitiveType::PLANE:
                m = makePlaneUncertaintyMarker(*gaus, voxel_center, resolution, id++, stamp, frame_id);
                arr.markers.push_back(m);
                break;

            default:
                break;
        }
    }
    }
    
    return arr;
}

// visualization_msgs::msg::MarkerArray makeUncertaintyMarkers(
//     const gauss_mapping::GaussianIVox& ivox,
//     const rclcpp::Time& stamp,
//     const std::string frame_id)
// {
//     visualization_msgs::msg::MarkerArray arr;

//     std::unordered_map<gauss_mapping::UnionFindNode*, std::vector<Eigen::Vector3i>> clusters;

//     // Build clusters based on Union-Find parents and assign colors
//     for (const auto& [key, node] : ivox.map_) {
//         auto root = node->find();
//         clusters[root].push_back(key);
//     }

//     auto resolution = ivox.getVoxelResolution();

//     // Visualize each voxel Gaussian primitive
//     int id = 0;
//     for (auto& [root, voxels] : clusters) {

//         auto& gaus = root->gauss_ptr;
//         if (!gaus) continue; // safety check, should always have a Gaussian

//         for (const auto& key : voxels) {

//             gauss_mapping::Point voxel_center;
//             voxel_center[0] = (key.x() + 0.5) * resolution;
//             voxel_center[1] = (key.y() + 0.5) * resolution;
//             voxel_center[2] = (key.z() + 0.5) * resolution;

//             visualization_msgs::msg::Marker m;

//             switch (gaus->type) {
//                 case gauss_mapping::PrimitiveType::PLANE:
//                     m = makePlaneUncertaintyMarker(*gaus, voxel_center, resolution, id++, stamp, frame_id);
//                     arr.markers.push_back(m);
//                     break;

//                 default:
//                     break;
//             }
//         }

//     }
    
//     return arr;
// }

visualization_msgs::msg::MarkerArray makeVoxelMarkers(
    const gauss_mapping::GaussianIVox& ivox,
    const rclcpp::Time& stamp,
    const std::string frame_id)
{
    visualization_msgs::msg::MarkerArray arr;

    std::mt19937 gen(12345); // fixed seed for consistent cluster colors
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    std::unordered_map<gauss_mapping::UnionFindNode*, std::tuple<float,float,float>> parent_colors;

    auto resolution = ivox.getVoxelResolution();

    int id = 0;
    
    // Pulling grid const-reference safely to avoid the deleted copy constructor issue
    const auto& map = ivox.getRawGrid();

    map.forEachCell([&](const std::unique_ptr<gauss_mapping::UnionFindNode>& cell, const Bonxai::CoordT& coord) {
        if (!cell) return;

        gauss_mapping::UnionFindNode* root = cell->find();
        if (parent_colors.find(root) == parent_colors.end()) {
            parent_colors[root] = {dis(gen), dis(gen), dis(gen)};
        }
        auto [r, g, b] = parent_colors[root];

        gauss_mapping::Point voxel_center_;
        voxel_center_[0] = (coord.x + 0.5) * resolution;
        voxel_center_[1] = (coord.y + 0.5) * resolution;
        voxel_center_[2] = (coord.z + 0.5) * resolution;

        // --- Voxel cube ---
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "gilda_voxels";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = voxel_center_[0];
        m.pose.position.y = voxel_center_[1];
        m.pose.position.z = voxel_center_[2];
        m.pose.orientation.w = 1.0;
        m.scale.x = resolution;
        m.scale.y = resolution;
        m.scale.z = resolution;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.2f;
        
        arr.markers.push_back(m);
    });
    
    return arr;
}

// visualization_msgs::msg::MarkerArray makeVoxelMarkers(
//     const gauss_mapping::GaussianIVox& ivox,
//     const rclcpp::Time& stamp,
//     const std::string frame_id)
// {
//     visualization_msgs::msg::MarkerArray arr;

//     std::mt19937 gen(12345); // fixed seed for consistent cluster colors
//     std::uniform_real_distribution<float> dis(0.0f, 1.0f);
//     std::unordered_map<gauss_mapping::UnionFindNode*, std::tuple<float,float,float>> parent_colors;

//     auto resolution = ivox.getVoxelResolution();

//     int id = 0;
//     for (const auto& [key, node] : ivox.map_) {
//         auto* root = node->find();
//         if (parent_colors.find(root) == parent_colors.end()) {
//             parent_colors[root] = {dis(gen), dis(gen), dis(gen)};
//         }
//         auto [r,g,b] = parent_colors[root];

//         gauss_mapping::Point voxel_center_;
//         voxel_center_[0] = (key.x() + 0.5) * resolution;
//         voxel_center_[1] = (key.y() + 0.5) * resolution;
//         voxel_center_[2] = (key.z() + 0.5) * resolution;

//         // --- Voxel cube ---
//         visualization_msgs::msg::Marker m;
//         m.header.frame_id = frame_id;
//         m.header.stamp = stamp;
//         m.ns = "gilda_voxels";
//         m.id = id++;
//         m.type = visualization_msgs::msg::Marker::CUBE;
//         m.action = visualization_msgs::msg::Marker::ADD;
//         m.pose.position.x = voxel_center_[0];
//         m.pose.position.y = voxel_center_[1];
//         m.pose.position.z = voxel_center_[2];
//         m.pose.orientation.w = 1.0;
//         m.scale.x = resolution;
//         m.scale.y = resolution;
//         m.scale.z = resolution;
//         m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.2f;
//         arr.markers.push_back(m);

//         // --- Optional: Add text marker with point count ---
//     }
    
//     return arr;
// }


} // namespace ros_visuals