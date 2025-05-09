# KFUPM AutoWheels 
# [Youtube link for our Competition Submission – Stage 1](https://youtu.be/H7bgU0j2-UE)

![Simulation Snapshot](https://github.com/Moussa-Rd/KFUPM_AutoWheels-Stage-1/blob/b0e6da5982b8cf6432e9b7472f07b0c60544912b/KFUPM-AutoWheels.png)


---

# 📘 ReadMe

## 👋 Overview

Welcome to our Stage 1 submission for the **KFUPM Self-Driving Challenge**!

As students from **King Fahd University of Petroleum and Minerals (KFUPM)**, we’ve built a complete autonomous vehicle simulation using the **QCar platform** in a realistic urban environment. Our focus was to combine strong engineering principles with real-time control, computer vision, and system integration.This solution meets mostly all the evaluation criteria.

This repository combines **intelligent object detection**, **dynamic vehicle control**, and a **custom competition environment** in Quanser's QLabs.

--- 

## 🧪 Testing Environment

| Component | Details |
|----------|---------|
| **Simulation** | Quanser Interactive Labs v2.22 (Oct 2024) |
| **Hardware** | Dell i7 9th Gen, 16GB RAM, GTX 1650|
|**OS** | Windows 11 pro |
| **Detection Framework** | YOLOv8s |
| **Programming Language** | Python 3.10 |
| **Control** | Stanley Steering + PID Throttle |
| **Map** | Acc Setup_Real_Senario map |

---

## 🏁 Results

- ✅ **Accurate QCar path-following** with smooth ramped control  
- ✅ **YOLOv8 successfully detected** stop signs and traffic lights in varying conditions  
- ✅ **Real-time traffic light interaction** (red → stop, green → go)  
- ✅ **Stop signs triggered correct pauses**, then resumed autonomously  
- ✅ **System remained stable** throughout full trajectory duration


---

### 🔍 Perception Layer
- **YOLOv8 segmentation model** trained to detect:
  - Stop signs
  - Traffic lights
  - Yield signs
- **Stop signs** are filtered by bounding box width and center position

---

## 🧠 Development Process

### 1. Object Detection
- Integrated YOLOv8 (via Ultralytics) to detect key road signs

### 2. Control System
- Built EKF to estimate QCar state using GPS and IMU data
- Developed PID-based speed control and Stanley-based steering
- Tested dynamic pause logic triggered by stop signs or red lights

---


## 📂 Repository Structure

```
KFUPMAutoDrive-Simulation/
├── vehicle_control_yolo_vf.py/
│   ├── vehicle_control process       # EKF + Stanley + PID controller
│   ├── vision_detection process      # YOLOv8s-based sign & light detection
├── yolov8s-seg.pt           # Pretrained YOLOv8s model (FineTuned)
├── requirements.txt             # Python dependencies
└── README.md
```

---

## 🛠️ How to Run

1. **Python dependencies**

        pip install -r requirements.txt
   
3. **Start QLabs**
     Make sure QLabs is open and all previous real-time models are terminated.

3.**Launch experiment**
  - First run Setup_Real_Senario.py
  - Then run vehicle_control_yolo_vf.py (alongside all the necessary libraries attached to this repository)

---


## ✨ Reflections

Working on this simulation felt like building a real autonomous car, honestly it kind of is! 
From tuning controllers to tracking RGB frames in real-time, this challenge pushed our limits in the best way.
It's amazing how teamwork, code, and robotics can come together to make something that moves.

---

## 🗂️ File Descriptions
---

| File                         | Description                                                                                                                                                                                                                                                                                                        |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Setup_Real_Scenario.py`     | Responsible for **setting up the full QLabs simulation environment**, including walls, roads, signs (stop, yield, roundabout), crosswalks, traffic lights, and camera views. Also manages the traffic light state machine (red-yellow-green cycle) and spawns the QCar with an associated real-time model.         |
| `vehicle_control_yolo_vf.py` | **Main experiment file** that launches both control and vision processes using Python's multiprocessing module. It integrates an EKF-based estimator for QCar state, Stanley and PID controllers for path tracking and speed control, and a YOLOv8 segmentation model for detecting traffic lights and stop signs. |



---
**PS**: Screen recording affects the performance simulation  



