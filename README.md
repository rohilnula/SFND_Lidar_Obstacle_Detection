# Lidar Obstacle Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

Sensor fusion, which is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. 

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

In this project, the code uses the point cloud data collected by a Lidar Sensor. The code extensively utilizes the [Point Cloud Library (http://pointclouds.org/) (PCL), but RANSAC, KD-Tree and Euclidean distance modules are coustom implemeted instead of using the PCL library.

- The RANSAC algorithm is responsible for the separation between the road plane and obstacles. It is implemented in [src/Ransac.hpp](src/Ransac.hpp) file.
- The obstacle identification is made using the Euclidean clustering algorithm, which internally uses a k-d tree to speed up the search of points in the point cloud. Implemeted in [src/EuclideanClusterExtraction.hpp](src/EuclideanClusterExtraction.hpp) file.
- The implementation of the Euclidean clustering algorithm
provided in the [src/EuclideanClusterExtraction.hpp](src/EuclideanClusterExtraction.hpp) file.

### Execute the program:
```bash
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)