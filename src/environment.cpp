/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointprocessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud){
    // Processes the PCD data for a frame


    // ProcessPointClouds<pcl::PointXYZI> pointprocessor;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointprocessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // Filter to perform downsampling and excluding the PCD points beyond required area 
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointprocessor.FilterCloud(inputCloud, 0.4, Eigen::Vector4f (-10, -5.5, -2, 1), Eigen::Vector4f (30, 5.5, 1, 1));
    
    // Performs Cloud Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = 
                                                            pointprocessor.RansacPlaneSegmentation(filterCloud, 100, 0.2);
                                                            // pointprocessor.SegmentPlane(filterCloud, 100, 0.2);
    
    // Performs CLoud Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClustered = 
                                                            pointprocessor.KDTreeClustering(segmentedCloud.first, 0.5, 3, 300);
                                                            // pointprocessor.Clustering(segmentedCloud.first, 0.5, 3, 300);
    
    renderPointCloud(viewer, segmentedCloud.second, "road plane", Color(1, 1, 1));
    // renderPointCloud(viewer, inputCloud, "road plane");

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClustered) {
        std::cout << "Cluster Size ";
        pointprocessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointprocessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // :: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    // :: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    
    ProcessPointClouds<pcl::PointXYZ> pointprocessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointprocessor.SegmentPlane(inputCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointprocessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "Cluster Size ";
        pointprocessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointprocessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}