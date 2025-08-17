# LieOdyssey  

> An odyssey through inertial navigation on Lie groups.  

LieOdyssey is a lightweight, research-friendly toolkit for IMU preintegration in Inertial Navigation Systems (INS), built on the mathematical foundations of Lie groups and Lie algebras.  

It provides efficient preintegration routines on Gal(3), enabling robust navigation and state estimation in robotics, drones and autonomous systems.  

---

## ✨ Features  
- Preintegration on Gal(3) — precise handling of rotations and poses. 
- Preintegration on Gal(3) x gal(3) — precise handling of IMU biases.
- Lie algebra tools — exponential / logarithm maps, Jacobians, and retractions.  
- Lightweight & modular — easy to plug into factor graphs (e.g., GTSAM, Ceres).  

---

## 🚀 Getting Started  

### Installation  


---

## 📚 Background  

IMU preintegration is essential for modern INS navigation pipelines.  
Instead of re-integrating raw IMU data between every keyframe, preintegration accumulates “deltas” on the Lie group SE(3). This leads to:  
- Faster optimization in SLAM / VIO.  
- Better handling of uncertainty.  
- Consistency with the geometry of rotations and poses.  

---

## 🛡 Roadmap  
- [ ] Example demos (VIO, drone INS).  

---

## ⚖️ License  
MIT License. See LICENSE for details.  

---

## 🌌 Inspiration  

The name LieOdyssey merges the epic Greek journey (Odyssey) with the mathematical elegance of Lie groups, symbolizing the voyage from raw inertial measurements to precise navigation.  

