#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <shared_mutex>
#include <memory>
#include <mutex>
#include <atomic>
#include <cmath>

namespace gauss_mapping {

using Scalar = double;
using Point  = Eigen::Matrix<Scalar, 3, 1>;
using Mat3   = Eigen::Matrix<Scalar, 3, 3>;

/**
 * @brief Combines two hash values using FNV-like mixing.
 * @param seed Initial hash seed
 * @param v Value to combine
 */
inline size_t hash_combine(size_t seed, size_t v) {
    return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

struct HashVec3i {
    /**
     * @brief Hash function for 3D integer vectors
     * @param v 3D vector to hash
     */
    size_t operator()(const Eigen::Vector3i& v) const {
        size_t seed = 0;
        seed = hash_combine(seed, std::hash<int>()(v[0]));
        seed = hash_combine(seed, std::hash<int>()(v[1]));
        seed = hash_combine(seed, std::hash<int>()(v[2]));
        return seed;
    }
};

/**
 * @brief Computes the adjugate (adjoint) matrix of a 3x3 matrix
 * @param A Input 3x3 matrix
 * @param adj Output adjugate matrix
 */
inline void adjugateM3D(const Mat3& A, Mat3& adj) {
    adj(0,0) =  A(1,1)*A(2,2) - A(1,2)*A(2,1);
    adj(0,1) = -A(0,1)*A(2,2) + A(0,2)*A(2,1);
    adj(0,2) =  A(0,1)*A(1,2) - A(0,2)*A(1,1);

    adj(1,0) = -A(1,0)*A(2,2) + A(1,2)*A(2,0);
    adj(1,1) =  A(0,0)*A(2,2) - A(0,2)*A(2,0);
    adj(1,2) = -A(0,0)*A(1,2) + A(0,2)*A(1,0);

    adj(2,0) =  A(1,0)*A(2,1) - A(1,1)*A(2,0);
    adj(2,1) = -A(0,0)*A(2,1) + A(0,1)*A(2,0);
    adj(2,2) =  A(0,0)*A(1,1) - A(0,1)*A(1,0);
}

/**
 * @brief Auxiliar function to compute covariance of a point measurement in the sensor frame, given range and angular uncertainties
 * @param point_body Point measurement in the sensor frame
 * @param range_std Standard deviation of the range measurement
 * @param angle_rad_std Standard deviation of the angular measurement (in radians)
 * @param covariance Output covariance matrix in the sensor frame
 */
inline void calcBodyCov(Point point_body,
                        const Scalar range_std,
                        const Scalar angle_rad_std,
                        Mat3 &covariance) {
    // --- Compute range and its variance ---
    Scalar range = point_body.norm();
    Scalar range_variance = range_std * range_std;

    // --- Angular uncertainty (in radians) ---
    Scalar angle_variance = std::pow(std::sin(angle_rad_std), 2);

    Eigen::Matrix<Scalar, 2, 2> angular_cov = Eigen::Matrix<Scalar, 2, 2>::Zero();
    angular_cov(0, 0) = angle_variance;
    angular_cov(1, 1) = angle_variance;

    // --- Normalize direction vector ---
    Point direction = point_body;
    if (std::abs(direction.z()) < 1e-6) {
        direction.z() = 1e-6;  // avoid numerical instability
    }
    direction.normalize();

    // --- Skew-symmetric matrix of direction (cross-product matrix) ---
    Mat3 direction_skew;
    direction_skew <<     0, -direction.z(),  direction.y(),
                   direction.z(),            0, -direction.x(),
                  -direction.y(),  direction.x(),            0;

    // --- Build orthonormal basis perpendicular to direction ---
    Point basis1(1.0, 1.0, -(direction.x() + direction.y()) / direction.z());
    basis1.normalize();

    Point basis2 = basis1.cross(direction);
    basis2.normalize();

    // Matrix whose columns span the tangent plane
    Eigen::Matrix<Scalar, 3, 2> tangent_basis;
    tangent_basis.col(0) = basis1;
    tangent_basis.col(1) = basis2;

    // --- Map angular noise to Cartesian space ---
    Eigen::Matrix<Scalar, 3, 2> angular_jacobian =
        range * direction_skew * tangent_basis;

    // --- Final covariance ---
    covariance =
        // Radial uncertainty (along beam direction)
        direction * range_variance * direction.transpose()
        +
        // Angular uncertainty (perpendicular to beam)
        angular_jacobian * angular_cov * angular_jacobian.transpose();
}

struct pointWithCov {
    Point p;
    Mat3 cov;
};

enum class PrimitiveType {
    UNKNOWN,
    PLANE,
    VOLUME
};

struct GaussianPrimitive
{
    // --- Core Gaussian ---
    Point mean = Point::Zero(); // world coordinates mean
    Mat3  cov  = Mat3::Zero();
    PrimitiveType type = PrimitiveType::UNKNOWN;

    // --- Plane parametric form ax + by + cz + d = 0 ---
    int main_dir; // 0=z dominant, 1=y dominant, 2=x dominant
    Point param;  // [a, b, d] plane parametrization in dominant axis form
    Mat3 plane_cov = Mat3::Zero();

    // --- Incremental stats (Scatter Matrix) ---
    Scalar xx = 0, yy = 0, zz = 0;
    Scalar xy = 0, xz = 0, yz = 0;
    Scalar x = 0, y = 0, z = 0;
    int count = 0;

    /**
     * @brief Updates incremental sums with a new point
     * @param p Point to add to the Gaussian primitive
     */
    void addPoint(const Point& p) {
        count++;
        x += p.x(); y += p.y(); z += p.z();
        xx += p.x() * p.x(); yy += p.y() * p.y(); zz += p.z() * p.z();
        xy += p.x() * p.y(); xz += p.x() * p.z(); yz += p.y() * p.z();
    }

    /**
     * @brief Merges incremental sums from another primitive
     * @param other Another Gaussian primitive to merge
     */
    void mergeSums(const GaussianPrimitive& other) {
        // Direct accumulation (same reference frame)
        xx += other.xx;
        yy += other.yy;
        zz += other.zz;

        xy += other.xy;
        xz += other.xz;
        yz += other.yz;

        x += other.x;
        y += other.y;
        z += other.z;

        count += other.count;
    }

    /**
     * @brief Constructs the plane normal vector based on dominant axis
     * @return Normal vector in form [x, y, z] == [a, b, c]
     */
    Point buildNormalVector() const {
        Point w;
        switch (main_dir) {
            case 0: w << param(0), param(1), 1.0; break;
            case 1: w << param(0), 1.0, param(1); break;
            case 2: w << 1.0, param(0), param(1); break;
        }
        return w;
    }
};

using GaussPtr = std::shared_ptr<GaussianPrimitive>;

class UnionFindNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GaussPtr gauss_ptr;
    UnionFindNode *rootNode;
    Point voxel_center; // Center of the voxel for relative point storage
    std::atomic<size_t> points_since_update = 0;
    std::vector<pointWithCov> points_buffer; // Buffer to store points for covariance computation
    bool buffer_full = false; // Flag to indicate if buffer has reached capacity
    
    /**
     * @brief Constructs a union-find node for a voxel
     * @param center Center coordinates of the voxel
     */
    UnionFindNode(const Point& center) : voxel_center(center) {
        gauss_ptr = std::make_shared<GaussianPrimitive>();
        rootNode = this;
    }

    /**
     * @brief Finds root node with path compression
     */
    UnionFindNode* find() {
        if (rootNode == this) return this;
        return rootNode = rootNode->find();
    }
};

class GaussianIVox {
public:

    /**
     * @brief Constructs a GaussianIVox map with specified parameters
     * @param v_sz Voxel size (resolution)
     * @param upd_thresh Update threshold (points to accumulate before geometry recomputation)
     * @param planarity_thresh Threshold for plane vs volume classification
     * @param chi_square_thresh Mahalanobis distance threshold for merging
     * @param sensor_noise Sensor noise floor for covariance regularization
     */
    GaussianIVox(Scalar v_sz, 
                std::size_t upd_thresh, 
                std::size_t max_buffer_size = 50,
                Scalar planarity_thresh = 0.1, 
                Scalar chi_square_thresh = 7.815,
                Scalar sensor_noise = 0.01) 
        : v_size_(v_sz), inv_v_size_(1.0 / v_sz), update_threshold_(upd_thresh), max_buffer_size_(max_buffer_size),
          planarity_threshold_(planarity_thresh), chi_square_threshold_(chi_square_thresh), 
          noise_(sensor_noise) { }

    /**
     * @brief Destructor: cleans up all union-find nodes
     */
    ~GaussianIVox() {
        for (auto& pair : map_) {
            delete pair.second;
        }
        map_.clear();
    }

    /**
     * @brief Updates the map with a new point, triggers geometry computation and neighbor merging
     * @param point Point to insert into the map (world-frame coordinates)
     */
    void update(const pointWithCov& point_cov) {
        auto &point = point_cov.p;
        Eigen::Vector3i key = (point * inv_v_size_).array().floor().cast<int>();

        Point voxel_center;
        voxel_center[0] = (key.x() + 0.5) * v_size_;
        voxel_center[1] = (key.y() + 0.5) * v_size_;
        voxel_center[2] = (key.z() + 0.5) * v_size_;
        
        UnionFindNode* node = nullptr;
        {
            std::unique_lock<std::shared_mutex> lock(map_mtx_);
            auto it = map_.find(key);
            if (it == map_.end()) {
                node = new UnionFindNode(voxel_center);
                map_[key] = node;
            } else {
                node = it->second;
            }
        }

        UnionFindNode* root = node->find();

        if(root->buffer_full) {
            checkNeighbors(root, key);
            return;
        }

        root->gauss_ptr->addPoint(point);
        root->points_buffer.push_back(point_cov);
        root->points_since_update++;
        total_points_++;

        // Trigger geometry update incrementally
        if (root->points_since_update >= update_threshold_) {
            computeGeometry(root);
            // checkNeighbors(root, key);
        }
    }

    /**
     * @brief Updates the map with new points
     * @param points Points to insert into the map (world-frame coordinates)
     */
    void update(const std::vector<pointWithCov>& points) {
        for (const auto& point_cov : points) {
            update(point_cov);
        }
    }

    /**
     * @brief Retrieves the Gaussian primitive at a given point location
     * @param p Query point in world coordinates
     * @return Shared pointer to the root Gaussian primitive, or nullptr if not found
     */
    GaussPtr getPrimitiveAtPoint(const Point& p) const
    {
        Eigen::Vector3i key = (p * inv_v_size_).array().floor().cast<int>();

        std::shared_lock<std::shared_mutex> lock(map_mtx_);

        auto it = map_.find(key);
        if (it == map_.end())
            return nullptr;

        UnionFindNode* node = it->second;
        UnionFindNode* root = node->find();

        return root->gauss_ptr;
    }

    /**
     * @brief Retrieves neighboring Gaussian primitives around a query point
     * @param p Query point in world coordinates
     * @param radius Neighborhood radius in voxel units
     * @return Vector of neighboring Gaussian primitives
    */
    std::vector<GaussPtr> getNeighborPrimitives(const Point& p,
                                                int radius = 1) const
    {
        std::vector<GaussPtr> neighbors;

        Eigen::Vector3i center_key =
            (p * inv_v_size_).array().floor().cast<int>();

        std::shared_lock<std::shared_mutex> lock(map_mtx_);

        // Reserve approximate neighborhood size
        int side = 2 * radius + 1;
        neighbors.reserve(side * side * side);

        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    Eigen::Vector3i neighbor_key =
                        center_key + Eigen::Vector3i(dx, dy, dz);

                    auto it = map_.find(neighbor_key);

                    if (it == map_.end())
                        continue;

                    UnionFindNode* node = it->second;

                    if (!node)
                        continue;

                    UnionFindNode* root = node->find();

                    if (!root)
                        continue;

                    GaussPtr g = root->gauss_ptr;

                    if (!g)
                        continue;

                    // Skip empty primitives
                    if (g->count <= 0)
                        continue;

                    neighbors.push_back(g);
                }
            }
        }

        return neighbors;
    }

    /**
     * @brief Retrieves K-Nearest Neighbor primitives around a query point
     * @param p Query point in world coordinates
     * @param k Number of nearest neighbors to retrieve
     * @param max_radius Maximum search radius in voxel units (upper search limit to avoid excessive search in sparse maps)
     * @return Vector of neighboring Gaussian primitives
    */
    std::vector<GaussPtr> getKNearestPrimitives(const Point& p,
                                                size_t k,
                                                int max_radius = 2) const
    {
        std::vector<std::pair<Scalar, GaussPtr>> candidates;

        Eigen::Vector3i center_key =
            (p * inv_v_size_).array().floor().cast<int>();

        std::shared_lock<std::shared_mutex> lock(map_mtx_);

        std::unordered_set<UnionFindNode*> visited_roots;

        // Progressive radius expansion
        for (int radius = 1; radius <= max_radius; ++radius)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                for (int dy = -radius; dy <= radius; ++dy)
                {
                    for (int dz = -radius; dz <= radius; ++dz)
                    {
                        Eigen::Vector3i neighbor_key =
                            center_key + Eigen::Vector3i(dx, dy, dz);

                        auto it = map_.find(neighbor_key);

                        if (it == map_.end())
                            continue;

                        UnionFindNode* node = it->second;

                        if (!node)
                            continue;

                        UnionFindNode* root = node->find();

                        if (!root)
                            continue;

                        // Avoid duplicates from DSU
                        if (visited_roots.count(root))
                            continue;

                        visited_roots.insert(root);

                        GaussPtr g = root->gauss_ptr;

                        if (!g)
                            continue;

                        if (g->count <= 0)
                            continue;

                        // Distance to Gaussian mean
                        Scalar dist =
                            (g->mean - p).squaredNorm();

                        candidates.emplace_back(dist, g);
                    }
                }
            }

            // Stop once enough candidates found
            if (candidates.size() >= k)
                break;
        }

        // Sort by distance
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const auto& a, const auto& b)
            {
                return a.first < b.first;
            });

        // Keep only k nearest
        std::vector<GaussPtr> result;

        size_t n =
            std::min(k, candidates.size());

        result.reserve(n);

        for (size_t i = 0; i < n; ++i)
            result.push_back(candidates[i].second);

        return result;
    }

    /**
     * @brief Thread-safe retrieval of all unique root Gaussian primitives
     * @return Vector of shared pointers to all root Gaussians in the map
     */
    std::vector<GaussPtr> getGaussians() const {
        std::shared_lock<std::shared_mutex> lock(map_mtx_);
        std::vector<GaussPtr> result;
        for (auto const& [_, node] : map_) {
            if (node->rootNode != node) continue; // only roots
            result.push_back(node->gauss_ptr);
        }
        return result;
    }

    /**
     * @brief Thread-safe retrieval of Gaussians filtered by primitive type
     * @param type Primitive type to filter (PLANE, VOLUME, etc.)
     * @return Vector of shared pointers to Gaussians of the specified type
     */
    std::vector<GaussPtr> getByType(PrimitiveType type) const {
        std::shared_lock<std::shared_mutex> lock(map_mtx_);
        std::vector<GaussPtr> result;
        for (const auto& [_, node] : map_) {
            if (node->rootNode != node) continue; // only roots

            const auto& g = node->gauss_ptr;
            if (g && g->type == type) {
                result.push_back(g);
            }
        }

        return result;
    }

    /**
     * @brief Clears the map and resets all data
     */
    void clear() {
        std::unique_lock<std::shared_mutex> lock(map_mtx_);
        for (auto& pair : map_) {
            delete pair.second;
        }
        map_.clear();
        total_points_ = 0;
    }

    /**
     * @brief Returns the total number of points inserted into the map
     * @return Total number of points
     */
    size_t getTotalPoints() const {
        return total_points_.load();
    }

    /**
     * @brief Returns the total number of voxels
     * @return Total number of voxels
     */
    size_t size() const {
        std::shared_lock<std::shared_mutex> lock(map_mtx_);
        return map_.size();
    }

    /**
     * @brief Returns the voxel resolution
     * @return Voxel resolution
     */
    Scalar getVoxelResolution() const {
        return v_size_;
    }

    /**
     * @brief Apply a function to each voxel in a thread-safe manner
     * @param f Function to apply, taking (key, node) as arguments
     */
    template<typename Func>
    void forEachVoxel(Func&& f) const {
        std::shared_lock<std::shared_mutex> lock(map_mtx_);
        for (const auto& [key, node] : map_) {
            f(key, node);
        }
    }

    /**
     * @brief Computes Gaussian geometry from scatter matrix, classifies primitive type via PCA
     * @param node Union-find node containing the point statistics
     */
    void computeGeometry(UnionFindNode* node) {

        auto& g = node->gauss_ptr;
        if (g->count < 5) return;

        Scalar n = static_cast<Scalar>(g->count);

        // World Mean
        g->mean << (g->x / n),
                    (g->y / n),
                    (g->z / n);

        // Unbiased covariance estimation using the scatter matrix and mean
        Scalar inv = 1.0 / (n - 1); // Sample covariance normalization

        Scalar mx = g->x / n;
        Scalar my = g->y / n;
        Scalar mz = g->z / n;

        g->cov <<
            (g->xx - n*mx*mx) * inv, (g->xy - n*mx*my) * inv, (g->xz - n*mx*mz) * inv,
            (g->xy - n*mx*my) * inv, (g->yy - n*my*my) * inv, (g->yz - n*my*mz) * inv,
            (g->xz - n*mx*mz) * inv, (g->yz - n*my*mz) * inv, (g->zz - n*mz*mz) * inv;

        Eigen::SelfAdjointEigenSolver<Mat3> es(g->cov);
        Point evals = es.eigenvalues();

        // PCA classification
        Scalar l0 = std::max(evals(0), 0.0);

        // Primitive classification
        if (l0 < planarity_threshold_) {
            g->type = PrimitiveType::PLANE;

            if(!solvePlaneAdjugate(node, es.eigenvectors().col(0)))
                g->type = PrimitiveType::UNKNOWN; // Degenerate case, keep as UNKNOWN
        }
        else {
            g->type = PrimitiveType::VOLUME;

            // For volumes, nothing extra needed — mean and cov already store 3D Gaussian
        }

        node->points_since_update = 0;
        if(node->points_buffer.size() >= max_buffer_size_) {
            node->buffer_full = true;
            std::vector<pointWithCov>().swap(node->points_buffer); // Clear buffer to save memory
        }
    }

    /**
     * @brief Checks 6-neighbor voxels and attempts merging with same-type primitives
     * @param root Root union-find node of current voxel
     * @param key Voxel grid key of current location
     */
    void checkNeighbors(UnionFindNode* root, const Eigen::Vector3i& key) {
        
        // 6-Neighbor Search for Merging (DSU)
        std::shared_lock<std::shared_mutex> lock(map_mtx_);
        int offsets[6][3] = {{-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,1}, {0,0,-1}};

        for (auto& off : offsets) {
            Eigen::Vector3i neighbor_key(key.x() + off[0], key.y() + off[1], key.z() + off[2]);
            auto it = map_.find(neighbor_key);
            if (it != map_.end()) {
                UnionFindNode* neighbor_root = it->second->find();
                if (neighbor_root != root) tryMerge(root, neighbor_root);
            }
        }
    }

    /**
     * @brief Attempts to merge two primitives using Mahalanobis distance test
     * @param a Root node of first primitive
     * @param b Root node of second primitive
     */
    void tryMerge(UnionFindNode* a, UnionFindNode* b) {

        GaussPtr ga = a->gauss_ptr;
        GaussPtr gb = b->gauss_ptr;

        // Only merge fully build planes or volumes
        if(!(a->buffer_full) || !(b->buffer_full)) return;

        // Only merge same type primitives
        if (ga->type != gb->type || ga->type == PrimitiveType::UNKNOWN) return;

        bool merge = false;
        switch (ga->type) {
            case PrimitiveType::PLANE: {
                // --- Mahalanobis distance for Plane similarity ---
                if(ga->main_dir != gb->main_dir) break;

                Point na = ga->buildNormalVector();
                Point nb = gb->buildNormalVector();

                if (na.dot(nb) < 0) {
                    gb->param *= -1.0;
                }

                // (Bayesian style)
                Eigen::Vector3d delta_pi = ga->param - gb->param; 

                // Add a regularization term to the sum of covariances
                // This represents the uncertainty of the "link" between two voxels.
                Mat3 cov_sum = ga->plane_cov + gb->plane_cov;
                cov_sum.diagonal().array() += noise_*noise_; // regularization floor

                Scalar m_dist_sq = delta_pi.transpose() * cov_sum.inverse() * delta_pi;

                if (m_dist_sq < chi_square_threshold_)
                    merge = true;

                break;
            }

            case PrimitiveType::VOLUME: {
                // --- Mahalanobis distance on means for Gaussian similarity ---
                Point diff = gb->mean - ga->mean;
                Mat3 cov_sum = ga->cov + gb->cov;
                cov_sum.diagonal().array() += noise_*noise_; // regularization floor
                if (cov_sum.determinant() < 1e-12) break;

                Scalar m_dist_sq = diff.transpose() * cov_sum.inverse() * diff;

                if (m_dist_sq < chi_square_threshold_) 
                    merge = true;

                break;
            }

            default:
                break;
        }

        if(!merge) return;

        b->rootNode = a; // DSU Union: root of b points to root of a

        ga->mergeSums(*(gb)); // Merge incremental sums for accurate geometry update

        switch (ga->type) {
            case PrimitiveType::PLANE: {
                // VoxelMap++ merging logic: weighted average of plane parameters based on covariance "confidence"
                Scalar trA = ga->plane_cov.trace();
                Scalar trB = gb->plane_cov.trace();

                Scalar wA = trB / (trA + trB);
                Scalar wB = trA / (trA + trB);
                
                ga->param = wA * ga->param + wB * gb->param;
                ga->plane_cov = wA * wA * ga->plane_cov + wB * wB * gb->plane_cov;

                break;
            }
            
            case PrimitiveType::VOLUME: {
                computeGeometry(a); // Recompute mean and covariance for merged volume
                break;
            }

            default:
                break;
        }

        b->gauss_ptr.reset(); // Clear merged primitive to save memory, only root holds valid Gaussian
    }

    /**
     * @brief Solves plane parameters using adjugate method, selects stable axis form
     * @param node Union-find node containing point statistics
     * @param evecMin Smallest eigenvector for axis selection
     * @return true if plane solved successfully, false if degenerate
     */
    bool solvePlaneAdjugate(
        UnionFindNode* node,
        const Point& evecMin
    ) {
        Mat3 A, A_star;
        Point E;
        Scalar detA = 0.0;

        auto g = node->gauss_ptr;

        Scalar n = static_cast<Scalar>(g->count);
        Scalar xx = g->xx, yy = g->yy, zz = g->zz;
        Scalar xy = g->xy, xz = g->xz, yz = g->yz;
        Scalar x  = g->x,  y  = g->y,  z  = g->z;

        // --- Select most stable parameterization ---
        g->main_dir = 0;

        if (std::fabs(evecMin[0]) >= std::fabs(evecMin[1]) &&
            std::fabs(evecMin[0]) >= std::fabs(evecMin[2])) {
            g->main_dir = 2; // x dominant → x + ay + bz + d = 0
            A << yy, yz, y,
                yz, zz, z,
                y,  z,  n;

            E << -xy, -xz, -x;
        }
        else if (std::fabs(evecMin[1]) >= std::fabs(evecMin[0]) &&
                std::fabs(evecMin[1]) >= std::fabs(evecMin[2])) {
            g->main_dir = 1; // y dominant → ax + y + bz + d = 0

            A << xx, xz, x,
                xz, zz, z,
                x,  z,  n;

            E << -xy, -yz, -y;
        }
        else {
            g->main_dir = 0; // z dominant → ax + by + z + d = 0

            A << xx, xy, x,
                xy, yy, y,
                x,  y,  n;

            E << -xz, -yz, -z;
        }

        detA = A.determinant();
        if (std::fabs(detA) < 1e-9) return false;

        adjugateM3D(A, A_star);

        Point param = (A_star * E) / detA;

        // Store dominant axis parameters [a, b, d] for merging logic
        g->param = param; 

        // --- Compute analytical plane covariance ---
        Point ddetA_dpw;
        Mat3 dAstarE_dpw, J_pw;

        for (auto pv : node->points_buffer) {
            Scalar xi = pv.p[0];
            Scalar yi = pv.p[1];
            Scalar zi = pv.p[2];
            switch(g->main_dir) {
                case 0: // z dominant
                    ddetA_dpw
                            << 2 * n * xi * yy + 2 * y * (xy + yi * x) - 2 * yy * x - 2 * xi * y * y - 2 * n * yi * xy,
                            2 * n * yi * xx + 2 * x * (xy + xi * y) - 2 * xx * y - 2 * yi * x * x - 2 * n * xi * xy,
                            0.0;
                    dAstarE_dpw << zi * y * y - n * zi * yy + n * yi * yz - yz * y + yy * z - yi * y * z,
                            2 * xz * y - 2 * n * yi * xz + n * (xi * yz + zi * xy) - x * (yz + zi * y) +
                            2 * yi * x * z - z * (xy + xi * y),
                            xi * y * y - n * xi * yy + n * yi * xy - yi * x * y + yy * x - xy * y,
                            n * (yi * xz + zi * xy) - y * (xz + zi * x) + 2 * yz * x - 2 * n * xi * yz +
                            2 * xi * y * z - z * (yi * x + xy),
                            n * xi * xz - x * xz + zi * x * x - n * zi * xx + xx * z - xi * x * z,
                            n * xi * xy - xi * x * y + yi * x * x - n * yi * xx + xx * y - xy * x,
                            yy * (xz + zi * x) - y * (zi * xy + yi * xz) + 2 * xi * yz * y - yz * (xy + yi * x) +
                            2 * yi * z * xy - 2 * xi * z * yy,
                            2 * yi * xz * x - xz * (xi * y + xy) + xx * (yz + zi * y) - x * (zi * xy + xi * yz) +
                            2 * xi * z * xy - 2 * yi * z * xx,
                            xi * yy * x - xi * xy * y + yi * xx * y - yi * xy * x + xy * xy - xx * yy;
                    break;
                case 1: // y dominant
                    ddetA_dpw
                            << 2 * n * xi * zz + 2 * z * (xz + zi * x) - 2 * zz * x - 2 * xi * z * z - 2 * n * zi * xz,
                            0.0,
                            2 * n * zi * xx + 2 * x * (xz + xi * z) - 2 * zi * x * x - 2 * xx * z - 2 * n * xi * xz;
                    dAstarE_dpw << yi * z * z - n * yi * zz + n * zi * yz - yz * z + y * zz - zi * y * z,
                            xi * z * z - n * xi * zz + n * zi * xz - zi * x * z + zz * x - xz * z,
                            2 * xy * z - 2 * n * zi * xy + n * (yi * xz + xi * yz) - x * (yz + yi * z) +
                            2 * zi * y * x - y * (xz + xi * z),
                            n * (yi * xz + zi * xy) - z * (xy + yi * x) + 2 * yz * x - 2 * n * xi * yz +
                            2 * xi * y * z - y * (xz + zi * x),
                            n * xi * xz - xi * x * z + zi * x * x - n * zi * xx + xx * z - xz * x,
                            n * xi * xy - xy * x + yi * x * x - n * yi * xx + y * xx - xi * y * x,
                            zz * (yi * x + xy) - z * (yi * xz + zi * xy) + 2 * xi * yz * z - yz * (xz + zi * x) +
                            2 * zi * y * xz - 2 * xi * y * zz,
                            xi * zz * x - xi * xz * z + zi * xx * z - zi * xz * x + xz * xz - xx * zz,
                            2 * zi * xy * x - xy * (xi * z + xz) + xx * (yz + yi * z) - x * (yi * xz + xi * yz) +
                            2 * xi * y * xz - 2 * zi * y * xx;
                    break;
                case 2: // x dominant
                    ddetA_dpw << 0.0,
                            2 * n * yi * zz + 2 * z * (yz + zi * y) - 2 * zz * y - 2 * yi * z * z - 2 * n * zi * yz,
                            2 * n * zi * yy + 2 * y * (yz + yi * z) - 2 * zi * y * y - 2 * yy * z - 2 * n * yi * yz;
                    dAstarE_dpw << yi * z * z - n * yi * zz + n * zi * yz - zi * y * z + zz * y - yz * z,
                            xi * z * z - n * xi * zz + n * zi * xz - xz * z + x * zz - zi * x * z,
                            2 * xy * z - 2 * n * zi * xy + n * (xi * yz + yi * xz) - y * (xz + xi * z) +
                            2 * zi * x * y - x * (yz + yi * z),
                            n * yi * yz - yi * y * z + zi * y * y - n * zi * yy + yy * z - yz * y,
                            n * (xi * yz + zi * xy) - z * (xy + xi * y) + 2 * xz * y - 2 * n * yi * xz +
                            2 * yi * x * z - x * (yz + zi * y),
                            n * yi * xy - xy * y + xi * y * y - n * xi * yy + x * yy - yi * x * y,
                            yi * zz * y - yi * yz * z + zi * yy * z - zi * yz * y + yz * yz - yy * zz,
                            zz * (xy + xi * y) - z * (xi * yz + zi * xy) + 2 * yi * xz * z - xz * (yz + zi * y) +
                            2 * zi * x * yz - 2 * yi * x * zz,
                            2 * zi * xy * y - xy * (yz + yi * z) + yy * (xz + xi * z) - y * (xi * yz + yi * xz) +
                            2 * yi * x * yz - 2 * zi * x * yy;
                    break;
                default:
                    return false; // should never happen
            }
            
            J_pw = A_star * E * (-1.0 * ddetA_dpw / detA / detA).transpose() + dAstarE_dpw / detA;
            g->plane_cov += J_pw * pv.cov * J_pw.transpose();
        }

        return true;
    }

    mutable std::shared_mutex map_mtx_;
    std::unordered_map<Eigen::Vector3i, UnionFindNode*, HashVec3i> map_;
    Scalar v_size_, inv_v_size_;
    std::size_t update_threshold_;
    std::size_t max_buffer_size_;
    std::atomic<size_t> total_points_{0};
    Scalar planarity_threshold_;  // Tunable threshold for plane classification
    Scalar chi_square_threshold_; // Tunable threshold for plane merging
    Scalar noise_;                // Sensor noise floor for covariance estimation
};

} // namespace gauss_mapping