#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

/*** Defining Programme Parameters ***/
// Path to directory where point cloud data (.pcd) is stored
const static std::string kBasePathRel = "../";
// Minimum distance threshold of the LiDAR scan points to preserve (in meters).
const static double kMinimumDistanceLidarDetections = 8.0;
// Maximum number of map scan points to store in memory.
// const static int kMaximumScanPointsMap = 4500;
const static int kMaximumScanPointsMap = 5000;

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

/*** Voxel Grid Hyperparameters ***/
// Resolution of each 3D voxel ('box') used to downsample the point cloud
const static double kLeafSizeVoxelGrid = 0.5;


PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;

// Keyboard event handler for controlling the vehicle via arrow keys
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

/* Handles the vehicle state, i.e., applies acceleration, brake, and steering to the CARLA vehicle based on input ControlState */

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

/* Renders a vehicle bounding box at a given pose for visualization */

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}



/* Configures the simulated LiDAR sensor attributes used in the CARLA Simulator. */

void SetLidarAttributes(
		auto& lidarSensor
) {

    lidarSensor.SetAttribute(
        "upper_fov", "15"
    );
    lidarSensor.SetAttribute(
        "lower_fov", "-25"
    );
    lidarSensor.SetAttribute(
        "channels", "32"
    );
    lidarSensor.SetAttribute(
        "range", "30"
    );
    lidarSensor.SetAttribute(
        "rotation_frequency", "30"
    );
    lidarSensor.SetAttribute(
        "points_per_second", "500000"
    );
}


/* Filters out LiDAR points within ego-vehicle bounds*/

bool InEgoVehicleRange(
        auto& detection
) {
    return (((detection.point.x * detection.point.x)
             + (detection.point.y * detection.point.y)
             + (detection.point.z * detection.point.z)
            ) <= kMinimumDistanceLidarDetections
    );
}


/* Updates vehicle pose, renders ground truth and steering vector, and applies control */
void UpdatePoseAndControl(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    std::shared_ptr<carla::client::Vehicle>& vehicle,
    Pose& truePose,
    Pose& poseRef,
    carla::client::Vehicle::Control& control,
    std::vector<ControlState>& controlStateHistory
) {
    // Clear previous vehicle box renderings
    viewer->removeShape("box0");
    viewer->removeShape("boxFill0");

    // Extract vehicle transform from CARLA
    const auto transform = vehicle->GetTransform();
    const auto location = transform.location;
    const auto rotation = transform.rotation;

    // Convert to Pose relative to reference
    truePose = Pose(
        Point(location.x, location.y, location.z),
        Rotate(rotation.yaw * pi / 180.0,
               rotation.pitch * pi / 180.0,
               rotation.roll * pi / 180.0)
    ) - poseRef;

    // Render ground-truth vehicle pose in red
    drawCar(truePose, 0, Color(1.0f, 0.0f, 0.0f), 0.7f, viewer);

    // Visualize steering direction in green
    double theta = truePose.rotation.yaw;
    double steer_angle = control.steer * pi / 4 + theta;

    viewer->removeShape("steer");
    renderRay(
        viewer,
        Point(truePose.position.x + 2 * cos(theta),
              truePose.position.y + 2 * sin(theta),
              truePose.position.z),
        Point(truePose.position.x + 4 * cos(steer_angle),
              truePose.position.y + 4 * sin(steer_angle),
              truePose.position.z),
        "steer",
        Color(0.0f, 1.0f, 0.0f)
    );

    // Apply last control state if available
    if (!controlStateHistory.empty()) {
        ControlState lastControl = controlStateHistory.back();
        controlStateHistory.clear();

        // Convert ControlState to CARLA control
        control.throttle = lastControl.t;
        control.steer = lastControl.s;
        control.brake = lastControl.b;

        vehicle->ApplyControl(control);
    }
}


/* Computes the error between the estimated and ground-truth vehicle pose. */
void UpdatePoseError(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const Pose& estimatedPose,
    const Pose& groundTruthPose,
    double& maxError
) {
    // Compute Euclidean error in the XY plane
    double dx = groundTruthPose.position.x - estimatedPose.position.x;
    double dy = groundTruthPose.position.y - estimatedPose.position.y;
    double poseError = std::sqrt(dx * dx + dy * dy);

    // Update maximum error if current error is larger
    if (poseError > maxError) {
        maxError = poseError;
    }

    // Compute distance driven from origin by ground-truth pose
    double distDriven = std::sqrt(
        groundTruthPose.position.x * groundTruthPose.position.x +
        groundTruthPose.position.y * groundTruthPose.position.y
    );

    // Remove old text overlays
    viewer->removeShape("maxE");
    viewer->removeShape("derror");
    viewer->removeShape("dist");
    viewer->removeShape("eval");

    // Display updated error statistics
    viewer->addText("Max Error: " + std::to_string(maxError) + " m", 
                    200, 100, 32, 1.0, 1.0, 1.0, "maxE", 0);

    viewer->addText("Pose Error: " + std::to_string(poseError) + " m", 
                    200, 150, 32, 1.0, 1.0, 1.0, "derror", 0);

    viewer->addText("Distance: " + std::to_string(distDriven) + " m", 
                    200, 200, 32, 1.0, 1.0, 1.0, "dist", 0);

    // Evaluate performance visually based on thresholds
    if (maxError > 1.2) {
        viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval", 0);
    } else if (distDriven >= 170.0) {
        viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval", 0);
    }
}

/* Implements Iterative Closest Point (ICP) Algorithm.*/
// Performs Iterative Closest Point alignment between `source` and `target` point clouds.
// Returns the transformation matrix aligning source to target.

Eigen::Matrix4d ICP(
    PointCloudT::Ptr target,
    PointCloudT::Ptr source,
    const Pose& startingPose,
    int iterations
) {
    // Initialize result as identity (no transformation)
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

    // Construct transformation matrix from initial pose
    Eigen::Matrix4d startingPoseTransform = transform3D(
        startingPose.rotation.yaw,
        startingPose.rotation.pitch,
        startingPose.rotation.roll,
        startingPose.position.x,
        startingPose.position.y,
        startingPose.position.z
    );

    // Apply the initial transformation to the source cloud
    PointCloudT::Ptr sourceTransformed(new PointCloudT);
    pcl::transformPointCloud(*source, *sourceTransformed, startingPoseTransform);

    // Configure and run ICP
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(sourceTransformed);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(kMaxCorrespondenceDistanceICP);
    icp.setMaximumIterations(iterations);
    icp.setTransformationEpsilon(kTransformationEpsilonICP);
    icp.setEuclideanFitnessEpsilon(kEuclideanFitnessEpsilonICP);
    icp.setRANSACOutlierRejectionThreshold(kRANSACOutlierRejectionThresholdICP);

    PointCloudT::Ptr aligned(new PointCloudT);
    pcl::console::TicToc timer;
    timer.tic();
    icp.align(*aligned);
    std::cout << "Finished ICP alignment in " << timer.toc() << " ms\n";
    std::cout << "ICP converged: " << std::boolalpha << icp.hasConverged()
              << ", Fitness score: " << icp.getFitnessScore() << "\n";

    // If ICP converged, update transformation matrix
    if (icp.hasConverged()) {
        transformationMatrix = icp.getFinalTransformation().cast<double>();
        transformationMatrix *= startingPoseTransform;
    } else {
        std::cout << "WARNING: ICP did not converge\n";
    }

    return transformationMatrix;
}

int main(){

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool init_scan = true;
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	// pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	// pcl::io::loadPCDFile("/home/ubuntu/nd0013_cd2693_Exercise_Starter_Code/Lesson_7_Project_Scan_Matching_Localization/c3-project/map.pcd", *mapCloud);
  	pcl::io::loadPCDFile("/home/ubuntu/Scan_Matching_Localization/project/map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
					pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		static int frame_id = 0;
		std::string filename = "frame_" + std::to_string(frame_id++) + ".png";
		viewer->saveScreenshot(filename);
		
		if(!new_scan){

			// Initialize pose with the ground-truth on first scan to set a reference frame
			if(init_scan == true)
			{
				pose.position = truePose.position;
				pose.rotation = truePose.rotation;
			}
			// Reset new scan state
			new_scan = true;

			// TODO: (Filter scan using voxel filter)
			pcl::VoxelGrid<PointT> voxelGrid;
			voxelGrid.setInputCloud(scanCloud);
			voxelGrid.setLeafSize(
				kLeafSizeVoxelGrid,
				kLeafSizeVoxelGrid,
				kLeafSizeVoxelGrid
			);

			cloudFiltered->clear();
			voxelGrid.filter(*cloudFiltered);
			
			//Apply ICP to align the current scan to the reference map
			Eigen::Matrix4d transformEstimate = ICP(mapCloud, cloudFiltered, pose, kMaximumIterationsICP);
			// TODO: Find pose transform by using ICP or NDT matching
			//pose = ....
			// Update estimated pose from the ICP result
			pose = getPose(transformEstimate);
			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr correctedScan(new PointCloudT);
			pcl::transformPointCloud(*cloudFiltered,*correctedScan,transformEstimate);

			// Visualize the aligned scan point cloud in red
			viewer->removePointCloud("scan");
			// TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, correctedScan, "scan", Color(1,0,0) );


			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}

			pclCloud.points.clear();
		}
  	}
	return 0;
}