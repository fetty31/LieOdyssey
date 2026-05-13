# LieOdyssey  

> An odyssey through inertial navigation on Lie groups.  

`LieOdyssey` is a lightweight, research-friendly toolkit for IMU preintegration and state estimation in Inertial Navigation Systems (INS), built on the mathematical foundation of Lie Theory. 

It provides efficient preintegration routines and filters on the most common Lie Groups, enabling robust navigation and state estimation in robotics, drones and autonomous systems.  

---

## ✨ Features  
- Preintegration on any Group available in [Lie++](https://github.com/aau-cns/Lie-plusplus) or [ManIf](https://github.com/artivis/manif).
- Lie algebra tools — exponential / logarithm maps, Jacobians, Adjoints, etc.  
- Ready-to-deploy filters for SLAM/INS estimation.
- Lightweight & modular — easy to plug into factor graphs (e.g., GTSAM, Ceres).  

---

## 🚀 Getting Started  

### Installation  
Clone the repo:
```sh
git clone https://github.com/fetty31/LieOdyssey
```
Build `LieOdyssey` _C++_ standalone library:
```sh
cd LieOdyssey/cpp
mkdir build && cd build
cmake -DENABLE_TEST=ON .. # build test (optional)
make install
```
> By default both _Lie++_ and _Manif_ will be the retrieved and installed when building _LieOdyssey_. If you would want to install only one of them, the explicit build flags `-DUSE_{MANIF/LIEPLUSPLUS}=ON/OFF` are available.

The project includes multiple __ROS packages__ that use the `lie_odyssey_cpp` library as a core backend for LiDAR–Inertial SLAM and odometry:
    
- [__`lio_ros`__](ros/lio_ros/README.md): A minimalist LiDAR-Inertial Odometry (LIO) implementation for ROS 2 
- [__`gilda_lio`__](ros/gilda_lio/README.md): A LiDAR-Inertial Odometry (LIO) system with an Adaptive Gaussian Voxelmap (same idea as [Voxelmap++](https://github.com/uestc-icsp/VoxelMapPlus_Public))

---

## 📚 Background  

### IMU Preintegration on Lie Groups

IMU preintegration is a cornerstone of modern **inertial navigation systems (INS)** and **visual/lidar-inertial SLAM** pipelines.  
Instead of re-integrating raw IMU data between every pair of keyframes during optimization, **preintegration accumulates relative motion increments (“deltas”) directly on the appropriate Lie group** — such as **SO(3)**, **SE₂(3)**, or **SGal(3)**.

This allows high-rate inertial measurements to be summarized into a compact representation that can be reused efficiently during optimization.

---

#### Why Preintegration?

##### ✅ Geometric Consistency
By operating on Lie groups, preintegration respects the **non-Euclidean geometry** of rotations and poses, avoiding errors caused by naive linear approximations in Euclidean space.

##### ⚡ Faster Optimization
High-rate IMU data are summarized into compact preintegrated measurements, significantly **reducing computation** and the number of variables optimized.

##### 🎯 Accurate Uncertainty Propagation
Covariances and Jacobians are propagated alongside the motion increments ("deltas"), ensuring **statistically correct weighting** of IMU constraints in the estimation process.

---

### Lie-Theoretic Foundations

Classical filters like the Extended Kalman Filter (EKF) assume **additive updates in Euclidean space**:
```math
x_{k+1} = f(x_k, u_k) + w_k
```

However, many robotic states (e.g., rotations in SO(3) or poses in SE(3)) lie on **Lie groups**, which are nonlinear manifolds. In this setting, addition is not globally well-defined.

Instead of additive noise, uncertainty is modeled in the **Lie algebra** (the tangent space), and mapped to the group via the exponential map:
```math
X_{k+1} = f(X_k, u_k)\,\exp(w_k)
```

The state is expressed as a (here right) perturbation around an estimate:
```math
X = \hat{X}\,\exp(\delta)
```

This formulation naturally handles **nonlinear manifold structure**, improving numerical stability and global consistency.

By combining preintegration with Lie-theoretic estimation, we achieve **robust, efficient and geometrically consistent** state estimation for systems with high-rate inertial sensors.

---

## 🛡 Roadmap  
- [X] ESEKF Filter.  
- [X] Invariant Filter.  
- [ ] Pose-Graph on general manifolds.  
- [ ] Example demos (VIO, drone INS).  

---

## ⚖️ License  
MIT License. See LICENSE for details.  

---

## 🌌 Inspiration  

The name LieOdyssey merges the epic Greek journey (Odyssey) with the mathematical elegance of Lie groups, symbolizing the voyage from raw inertial measurements to precise navigation.  

