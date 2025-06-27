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


## Programming Task

### Iterative Closest Point (ICP) for 3D Scan Matching

#### Background

`Iterative Closest Point` algorithm estimates a rigid-body transformation between two 3D point clouds by iteratively minimizing the mean square error between corresponding points. It operates in six degrees of freedom—3D position and rotation—and is widely used in localization and mapping tasks.

In this project, we use the `pcl::IterativeClosestPoint` implementation from the `Point Cloud Library` (PCL) to align two successive LiDAR point clouds — a `source` scan and a transformed `target` scan — that represent the ego-vehicle’s movement over a time step in the CARLA simulator. 

While the ICP is guaranteed to converge monotonically to a local minimum from any given rotation and translation of a data point set, it may or may not converge on the desired global minimum. To properly initialise our starting estimate, we assume a fixed starting pose at the origin. Given this we set our "initial guess" estimate to be the starting location of the ego-vehicle with position $\left(0, 0, 0\right)$ and form an initial estimate for the $4x4$ transformation matrix. 

`K-d tree` structure built from the point cloud centroids is used to improve search efficiency during the nearest-neighbor correspondence phase. A radius search with a threshold of 5 meters is performed using `pcl::KdTreeFLANN< PointT, Dist >::radiusSearch()`. This restricts correspondence search to nearby points, reducing computational complexity from $\mathcal{O}(N)$ to approximately $\mathcal{O}(\log N)$, where $N$ is the number of points in the `target` cloud.

In regards to the transformation estimate, we use the SVD algorithm to solve an `eigenvalue / eigenvector problem`. In other words, the transformation of the distance vectors formed by each point-correspondence is estimated in three-dimensions


The rigid-body transformation between point sets is computed using the `Singular Value Decomposition` (SVD) method, which solves the least-squares minimization between corresponding points. SVD yields both rotation and translation components by minimizing residual errors between matched points. In our implementation, we use `Eigen::JacobiSVD<MaxtrixXd, ComputeThinU | ComputeThinV` option for an efficient decomposition suited to rectangular matrices. In other words, this is equivalent to the least-squares solution with $\mathrm{U}$ given as a $n\times m$ and $\mathrm{V}$ as a $p\times m$ for rectangular $n \times p$ transformation matrix.

With this, we have introduced the ICP algorithm and covered most of the implementation overview from what was written in C++ code in exercises. Iterative Closest Point is therefore a fairly easy-to-understand, semi-computationally efficient local point-set matching algorithm that does not require pre-processing of the 3D point clouds (e.g., smoothing).

ICP is intuitive and relatively efficient, requiring minimal preprocessing (e.g., no smoothing). However, the method has notable drawbacks. Its performance heavily depends on a good initial pose and it can be disrupted by outliers or poor-quality data. Moreover, the rigid assumption and least-squares nature of the algorithm make it susceptible to inaccuracies from noisy range measurements, especially in adverse driving conditions (e.g., poor lighting, weather, occlusions, or uneven terrain).

While ICP is relatively intuittive and efficient requiring minimal preprocessing, it has limitations:

* Sensitive to the initial guess and outliers

* Prone to local minima

* Performance degrades with noisy or incomplete scans

In real-world self-driving scenarios, these limitations can lead to misalignments, undermining downstream tasks like object detection and tracking. Consequently, while ICP is a useful baseline for scan matching, its robustness may not be sufficient in all conditions. For more reliable performance, especially in dynamic or cluttered environments, feature-based methods or RANSAC-enhanced matching may offer better alternatives.


### Compiling and Executing the project

To perform scan matching using the ICP algorithm, several parameters can be configured in `c3-main.cpp` to control the LiDAR behavior and point cloud resolution:

```cpp
// Minimum distance threshold of the LiDAR scan points to preserve (in meters).
const static double kMinimumDistanceLidarDetections = 8.0;
// Maximum number of map scan points to store in memory.
// const static int kMaximumScanPointsMap = 4500;
const static int kMaximumScanPointsMap = 5000;
/*** Setting voxel grid hyperparameters ***/
// Resolution of each 3D voxel ('box') used to downsample the point cloud
const static double kLeafSizeVoxelGrid = 0.5;
```
Additionally, LiDAR-specific attributes (e.g., field of view, number of channels, range) are set using the `carla::client::Sensor blueprint sensor.lidar.ray_cast`. These can be configured manually via the `SetLidarAttributes` function or programmatically using the `GetRecommendedValues()` API (not shown here). We compiled and built the project using CMake and run it inside the CARLA environment to visualize ICP-based localization in real-time.

##### Loading the input scans

To run the ICP-based localization, you must have at least one map point cloud file, such as `map-loop.pcd`, provided in the Udacity workspace or this repository. 

By default, point cloud `.pcd` files should be placed in the root directory of the project. If stored elsewhere, update the path in `c3-main.cpp` accordingly. This path should be relative to the current working directory (`build/` folder containing the executable):

```cpp
// Set the base path relative to CWD where '.pcd' files are stored
const static std::string kBasePath = "../";
```

##### Setting the hyperparameters

Several configurable parameters inside `c3-main.cpp` control how ICP performs scan matching:
```cpp
/*** ICP Configuration Hyperparameters ***/
// Maximum correspondence distance between `source` and `target` (in meters)
// const static double kMaxCorrespondenceDistanceICP = 7;  		// meters (m)
const static double kMaxCorrespondenceDistanceICP = 5;  		// meters (m)
// Maximum number of ICP iterations to perform before termination.
// const static int kMaximumIterationsICP = 140;
const static int kMaximumIterationsICP = 120;
// Maximum epsilon threshold between previous transformation and current estimated transformation.
// const static double kTransformationEpsilonICP = 1e-8;
const static double kTransformationEpsilonICP = 1e-4;
// Maximum sum of Euclidean squared errors between two consecutive steps for ICP Convergence.
// const static double kEuclideanFitnessEpsilonICP = 5;
const static double kEuclideanFitnessEpsilonICP = 2;
// RANSAC threshold for filtering outliers in point correspondences
// const static double kRANSACOutlierRejectionThresholdICP = 0.3;  // meters (m)
const static double kRANSACOutlierRejectionThresholdICP = 0.2;  // meters (m)
```

##### Configuring CMAKE

To build the project, edit the `CMakeLists.txt` file to specify the source files:

```cpp
set(sources {FILENAME OF MAIN} {FILENAME OF HELPERS})
```

where `{FILENAME OF MAIN}` should be `c3-main.cpp` and `{FILENAME OF HELPERS}` should be `helpers.cpp`.


##### Creating the executable

To build the programme with the configured `CMakeLists.txt` file, first from the project root (e.g., `project/`), create a build directory:

```console
# mkdir build
```

Then, navigate to inside the `build` folder and execute the `cmake` build script. You can do this with the following command:


```console
# cd build && cmake ..
```

##### Executing the programme

Once the programme has been compiled successfully, the executable can be run the following command:

```console
# cd build && ./cloud_loc
```

Note that if using the Udacity VM to run this programme, you will need to perform two extra steps before the executable can be run. 

First, navigate to the project root directory and run the following:

```console
#  ./run_carla.sh
```

This should set the CARLA Simulator to headless mode (i.e., disable graphics output) and prevent the programme from incurring any `Segmentation fault (core dumped)` errors.

##### Using the programme

In order to navigate the ego-vehicle inside the simulator, keyboard input is required. To move the vehicle forward, press the up-arrow key three times (once set the vehicle motion forward plus twice to set the vehicle throttle). To stop the vehicle, press the down-arrow key. Once the vehicle has made a complete stop, press the down-arrow key again to set the car to reverse. From there, use the up-arrow key 2x to increase the vehicle throttle and move the car backwards. To control the steering angle of the vehicle, use the left- and right-arrow keys.


#### Results

The following output was produced during a test run of the localisation programme using ICP scan matching, where the ego drove for 170 m and pose error was always under 1.2m and ego was able to continously localize when it is moving at medium speed (3 taps on the up arrow):

<div align="center">

  <img src="../results/Jayant_Kumar_ICP.gif" width="100%" height="100%">

  <p><strong>Figure 1:</strong> Testing vehicle localization in CARLA using the ICP scan matching algorithm.</p>

</div>

## 3. Closing Remarks

##### Alternatives

* Evaluate the performance of a more-robust scan matching algorithm on real-world data in difficult driving environments (e.g., uneven terrian, harsh weather conditions, occlusions, etc.).
* Use the quaternion algorithm to estimate the rotation parameters with the Iterative Closest Point (ICP) in $n$ dimensions;


##### Extensions of task

* Experiment with NDT hyperparameters to improve accuracy and reduce time to converge. 

## 4. Future Work

* (Completed) Use extraction to move code outside `main` (i.e., create `SetActorsAndBlueprint`, `ProcessScan`, `UpdatePoseAndControl`, `UpdatePoseError` functions).
* Fine-tune the NDT algorithm hyperparameters.
* Replace `auto` keyword declaration with explicit typing (note: `CARLA C++ API` uses templates and their own smart pointer implementation).
* Re-organise `helpers.cpp` and `helpers.h` such that the implementation and header files follow proper conventions.


