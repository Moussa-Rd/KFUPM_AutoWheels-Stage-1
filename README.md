# KFUPM AutoWheels 🚗
### [Competition Submission – Stage 1](https://youtu.to_be_done)

![Simulation Snapshot](https://github.com/Moussa-Rd/KFUPM_AutoWheels-Stage-1/KFUPM-AutoWheels.png)

---

# 📘 Competition Submission – ReadMe

## 👋 Overview

Welcome to our Stage 1 submission for the **KFUPM Self-Driving Challenge**!

As students from **King Fahd University of Petroleum and Minerals (KFUPM)**, we’ve built a complete autonomous vehicle simulation using the **QCar platform** in a realistic urban environment. Our focus was to combine strong engineering principles with real-time control, computer vision, and system integration.

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

> Real-time object detection is handled via RGB image processing and HSV color analysis.  
> EKF fuses QCar GPS and gyroscope for localization.  
> Controller logic adapts to visual cues from stop signs and traffic lights.


## 🏁 Results

- ✅ **Accurate QCar path-following** with smooth ramped control  
- ✅ **YOLOv8 successfully detected** stop signs and traffic lights in varying conditions  
- ✅ **Real-time traffic light interaction** (red → stop, green → go)  
- ✅ **Stop signs triggered correct pauses**, then resumed autonomously  
- ✅ **System remained stable** throughout full trajectory duration



### 🔍 Perception Layer
- **YOLOv8 segmentation model** trained to detect:
  - Stop signs
  - Traffic lights
  - Yield signs
- Traffic light status is determined using **HSV brightness analysis** on detected boxes
- **Stop signs** are filtered by bounding box width and center position

## 🧠 Development Process

### 1. Object Detection
- Integrated YOLOv8 (via Ultralytics) to detect key road signs
- Classified traffic light color status using HSV brightness masks

### 2. Control System
- Built EKF to estimate QCar state using GPS and IMU data
- Developed PID-based speed control and Stanley-based steering
- Tested dynamic pause logic triggered by stop signs or red lights


## 📂 Repository Structure

```
KFUPMAutoDrive-Simulation/
├── control_vision/
│   ├── vehicle_control.py       # EKF + Stanley + PID controller
│   ├── vision_detection.py      # YOLOv8s-based sign & light detection
├── models/
│   └── yolov8s-seg.pt           # Pretrained YOLOv8 model (FineTuned)
├── vehicle_control_yolo_vf.py   # Entry point for launching both control & vision
├── requirements.txt             # Python dependencies
└── README.md
```


## 🛠️ How to Run

1. **Install Python dependencies**
   ---bash
   
        pip install -r requirements.txt
   
2. **Start QLabs**
     Make sure QLabs is open and all previous real-time models are terminated.

3.**Launch experiment**
  - First run Setup_Real_Senario.py
  - Then run vehicle_control_yolo_vf.py (alongside all the necessary libraries attached to this repository)
   
## ✨ Reflections

Working on this simulation felt like building a real autonomous car, honestly it kind of is! 
From tuning controllers to tracking RGB frames in real-time, this challenge pushed our limits in the best way.
It's amazing how teamwork, code, and robotics can come together to make something that moves.


## 🗂️ File Descriptions
---

| File                         | Description                                                                                                                                                                                                                                                                                                        |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Setup_Real_Scenario.py`     | Responsible for **setting up the full QLabs simulation environment**, including walls, roads, signs (stop, yield, roundabout), crosswalks, traffic lights, and camera views. Also manages the traffic light state machine (red-yellow-green cycle) and spawns the QCar with an associated real-time model.         |
| `vehicle_control_yolo_vf.py` | **Main experiment file** that launches both control and vision processes using Python's multiprocessing module. It integrates an EKF-based estimator for QCar state, Stanley and PID controllers for path tracking and speed control, and a YOLOv8 segmentation model for detecting traffic lights and stop signs. |



---
**PS**: Screen recording affects the performance simulation  


Messing around with Control Systems has been a blast. It's like being in a video game, but way cooler 'cause it's real life! This challenge has taught us a ton about making split-second decisions, which we're sure will come in handy down the road. Plus, teaming up with folks from all majors has been a blast. Who knew self-driving cars could bring so many cool people together?


