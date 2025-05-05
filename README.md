# KFUPM AutoWheels
# [Competition Submission Stage 1](https://youtu.to be done 
![image](https://github.com/SulimanSalih/KFUPMAUTOWHEELS--Stage1/assets/108358496/d6eec892-a7b1-4839-b83c-e7ed3c3dcf50)

# Competition Submission Read Me

## Overview
Hey there!
We’re excited to share our submission for the Self-Driving Challenge. As students from King Fahd University of Petroleum and Minerals (KFUPM), we’re passionate about diving into the world of autonomous vehicles and demonstrating what we’ve learned.

Our project combines the thrill of hands on learning with the technical depth of Control Systems, especially the fast paced decision making required to keep a self driving car both quick and stable.

What's Included:
This repository contains everything you need to explore our submission:

💻 The complete source code for our solution

🎥 A recorded video of our system in action

📄 Notes and documentation on how to run and understand our approach

We're proud of what we’ve built, and we hope you enjoy checking it out!

## Testing Environment  

- Quanser Interactive Labs Software v2.16  
- Testing was conducted on the QCar Lab PC  
- **YOLOv8** was used to detect stop signs, traffic lights, roundabouts, and yield signs  
- The roadmap-based planning was performed using custom graph utilities (`RoadMap`, `WHMap`)  


## Results  

- Achieved smooth and accurate QCar motion using optimized trajectories  
- Segmented road signs and objects with high reliability using YOLOv8  
- Depth images were used to compute the distance to segmented objects  
- Traffic light and stop sign behavior was correctly handled

## Variations  

- Visual detection may vary depending on hardware (camera quality, lighting)  
- Path behavior may shift with different map scaling or node configurations

  ## Development Process  

1. **Perception Layer**  
   - Integrated YOLOv8 for object segmentation on RGB images  
   - Mapped segmentation masks to depth data for real-world distance estimation  

2. **Planning Layer**  
   - Constructed roadmap-based path planner with A* and Hermite spline trajectory generation  
   - Visualized paths and map overlays using `matplotlib` and `OpenCV`  

3. **Integration**  
   - Combined segmentation with map-based navigation  
   - Developed modular utilities for testing both independently and as a system


  Extra : *******


  
Messing around with Control Systems has been a blast. It's like being in a video game, but way cooler 'cause it's real life!
This challenge has taught us a ton about making split-second decisions, which we're sure will come in handy down the road.
Plus, teaming up with folks from all majors has been a blast. Who knew self-driving cars could bring so many cool people together?

  
