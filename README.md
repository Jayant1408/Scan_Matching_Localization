# Project: Scan Matching Localization
Udacity Self-Driving Car Engineer Nanodegree – Localization Module

## Introduction
In this project `Scan Matching Localization`, we implementd two 3D scan matching algorithms in C++ in order to localize a simulated vehicle in the `CARLA Simulator`. The scan matching algorithm selected for this project is the `Iterative Closest Point` (ICP).

During the course, we explored the theory behind ICP and implemented it from scratch using `Singular Value Decomposition` (SVD) to estimate the rigid-body transformation parameters. In this project, however, we use the `pcl::IterativeClosestPoint` implementation from the `Point Cloud Library` (PCL) to focus on applying ICP for vehicle pose estimation in realistic driving scenarios.

The underlying assumption is that a `rigid-body transformation` exists between consecutive 3D point clouds. By aligning the `source` cloud to a known `target` map, we can extract the vehicle’s change in position and orientation, effectively estimating its 3D pose over time. 


## Core Goals
* Immplement `Iterative Closest Point` (ICP) algorithm to perform 3D scan matching in C++.
* Evaluate ICP in the `CARLA Simulator` using synthetic LiDAR data and a simulated ego-vehicle.
* Analyze the effectiveness and limitations of ICP when applied to real-world-inspired driving scenarios.

### Course Contents
The following topics are covered in course exercises:
* Markov Localisation in C++;
* Bayes' theorem with numerical examples;
* Point Cloud Library (PCL) in C++;
* Scan matching algorithms in C++;
* Localisation and mapping in CARLA Simulator. 


Other topics covered in course lectures and reading material:
* Markov assumption for motion model;
* Deriving the recursive Bayes filter;
* Gaussian normal distributions;
* Kinematic Bicycle Motion model;
* Coordinate systems and reference frames;
* Transformation matrices;
* Odometry, motion sensor data, wheel encoder edge cases;
* Scan matching, alignment and mapping.


### Learning Outcomes
#### Lesson 1: Introduction to Localization
* Understand how self-driving vehicles leverage GPS or object detections to localise itself in an environment;
* Predict vehicle motion using the `Kinematic Bicycle Motion` model.

#### Lesson 2: Markov Localization
* Apply the law of total probability to robotic motion;
* Derive the general Bayes' Filter with Markov assumption;
* Implement a 1D localisation programme in C++.

#### Lesson 2.5: Introduction to Point Cloud Library
* Familiarise yourself with the `PCL` library;
* Learn about the basics of PCL, e.g., the PCD file type and the PCL Viewer;
* Create and use LiDAR objects in PCL;
* Learn the relative PCL object `templates` and pointers in C++;
* Modify LiDAR object properties and parameters, e.g., number of layers, add random noise.

#### Lesson 3: Creating Scan Matching Algorithms
* Learn how the `Iterative Closest Point` (ICP) algorithm is used for localisation;
* Learn how the `Normal Distributions Transform` (NDT) algorithm is used for localisation;
* Implement the ICP and NDT for 2D localisation in C++.

#### Lesson 4: Utilizing Scan Matching in 3D
* Align 3D point cloud maps using the ICP algorithm;
* Align 3D point cloud maps using the NDT algorithm;
* Create and use point cloud maps in the CARLA simulator.
