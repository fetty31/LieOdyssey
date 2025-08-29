# LieOdyssey  

> An odyssey through inertial navigation on Lie groups.  

`LieOdyssey` is a lightweight, research-friendly toolkit for IMU preintegration in Inertial Navigation Systems (INS), built on the mathematical foundation of Lie Algebra.  

It provides efficient preintegration routines on the most common Lie Groups, enabling robust navigation and state estimation in robotics, drones and autonomous systems.  

---

## ✨ Features  
- Preintegration on any Group available in [Sophus](https://github.com/strasdat/Sophus), [Lie++](https://github.com/aau-cns/Lie-plusplus) or [ManIf](https://github.com/artivis/manif).
- Lie algebra tools — exponential / logarithm maps, Jacobians, and retractions.  
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
> By default the library Lie++ will be the one retrieved and built. If you would want to use ManIf or Sophus you should make sure they're installed in your system and modify [this flag](cpp/include/lie_odyssey/config.hpp).

If you want to use `LieOdyssey` within ROS, ensure you have cloned the repo inside a ROS workspace and build via default:
```sh
colcon build --symlink-install
```

---

## 📚 Background  

IMU preintegration is a cornerstone of modern inertial navigation systems (INS) and visual/lidar-inertial SLAM pipelines.
Rather than repeatedly integrating raw IMU measurements between every pair of keyframes during optimization, preintegration __accumulates relative motion increments (“deltas”) directly on the appropriate Lie group__ (e.g., SO(3), SE₂(3), SGal(3)).

This approach offers several key benefits:

- __Geometric consistency__: Working on Lie groups respects the non-Euclidean geometry of rotations and poses, avoiding errors from naive linear approximations.

- __Faster optimization__: Preintegrated measurements summarize high-rate IMU data, reducing the number of variables the optimizer must handle.

- __Accurate uncertainty propagation__: Covariances and Jacobians are computed alongside the deltas, enabling correct weighting of measurements in estimation.

By combining these advantages, preintegration allows robust and efficient state estimation in systems with high-rate inertial sensors.

---

## 🛡 Roadmap  
- [ ] Example demos (VIO, drone INS).  

---

## ⚖️ License  
MIT License. See LICENSE for details.  

---

## 🌌 Inspiration  

The name LieOdyssey merges the epic Greek journey (Odyssey) with the mathematical elegance of Lie groups, symbolizing the voyage from raw inertial measurements to precise navigation.  

