/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// function prototypes
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    //renderRays(viewer, lidar->position, cloud); // render rays to each point from lidar postion
    //renderPointCloud(viewer, cloud, "highway"); // render the entire point cloud

    // render obstacle and ground clouds
    ProcessPointClouds<pcl::PointXYZ> ppc;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc.SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc.Clustering(segmentCloud.second, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        ppc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = ppc.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    delete lidar;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // get real pcl data on street
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud"); //render all pcl
    
    cityBlock(viewer, pointProcessorI, inputCloud);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{  
    // downsampling pcl
    Eigen::Vector4f cropbox_minpoint_vec(-15, -6, -2, 1); // cropbox vectors defines a rectangular space where points are kept
    Eigen::Vector4f cropbox_maxpoint_vec( 15, 6, 0.2, 1); // the heights are found to remove the ceiling points from nearby buildings while keep the floor plane points(later for segmentation)
                                                          // the width is set to not include the nearby buildings 
    float filter_resolution = 0.25;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = pointProcessorI->FilterCloud(inputCloud, filter_resolution, cropbox_minpoint_vec, cropbox_maxpoint_vec);
    //renderPointCloud(viewer,filtered_cloud,"filtered_cloud"); // render voxelgrid- and cropbox- filtered pcl

    // segmentation road and obstacles
    ProcessPointClouds<pcl::PointXYZI> ppc;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = ppc.SegmentPlane(filtered_cloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1,0,0)); // keep for rendering obstacles for reference

    // cluster and draw the clusters and their bounding boxes
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = ppc.Clustering(segmentCloud.second, 0.4, 10, 500); 
    // 0.4 because it should be a little larger than  filter resolution 
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)}; // R,G,B
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        ppc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % 3]); // add modulo to cycle through colors
        Box box = ppc.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

// implementation with my implementation of RANSAC, KD-tree, Euclidean Clustering
void cityBlockCustom(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{  
    // downsampling pcl
    Eigen::Vector4f cropbox_minpoint_vec(-15, -6, -2, 1); // cropbox vectors defines a rectangular space where points are kept
    Eigen::Vector4f cropbox_maxpoint_vec( 15, 6, 0.2, 1); // the heights are found to remove the ceiling points from nearby buildings while keep the floor plane points(later for segmentation)
                                                          // the width is set to not include the nearby buildings 
    float filter_resolution = 0.25;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = pointProcessorI->FilterCloud(inputCloud, filter_resolution, cropbox_minpoint_vec, cropbox_maxpoint_vec);
    //renderPointCloud(viewer,filtered_cloud,"filtered_cloud"); // render voxelgrid- and cropbox- filtered pcl

    // segmentation road and obstacles
    ProcessPointClouds<pcl::PointXYZI> ppc;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = ppc.SegmentPlaneMy(filtered_cloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1,0,0)); // keep for rendering obstacles for reference

    // cluster and draw the clusters and their bounding boxes
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = ppc.ClusteringMy(segmentCloud.second, 0.4, 10, 500); 
    // 0.4 because it should be a little larger than  filter resolution 
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)}; // R,G,B
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        ppc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % 3]); // add modulo to cycle through colors
        Box box = ppc.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

// for lesson before 4-7
void view_lidar_pcl_once() 
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer); // Lesson 1,2,3
    cityBlock(viewer); // Lesson 4

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }  
}

// for lesson 4-7
void view_lidar_pcl_pcd_stream() 
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        
        cityBlock(viewer, pointProcessorI, inputCloudI);
        cityBlockCustom(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    //view_lidar_pcl_once();
    view_lidar_pcl_pcd_stream();
}