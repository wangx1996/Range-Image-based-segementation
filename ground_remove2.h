/*MIT License

Copyright (c) 2020 WX96

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/



#ifndef GROUND_REMOVE2_H
#define GROUND_REMOVE2_H

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <thread>
#include <mutex>  
//PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
//Eigen
#include <Eigen/Dense>
/*//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_driver_msgs/GpswithHeading.h>*/

using namespace std;
//using namespace message_filters;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

struct Velodyne_32_HDLE{
PCL_ADD_POINT4D;
int ring;
int pcaketnum;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(Velodyne_32_HDLE,
(float,x,x)
(float,y,y)
(float,z,z)
(int,ring,ring)
(int,pcaketnum,pcaketnum)
)


class GroundRemove2 {

public:

	GroundRemove2(int num_iter, int num_lpr, double th_seeds, double th_dist);
	void extract_initial_seeds_2(const pcl::PointCloud<Velodyne_32_HDLE>& p_sorted,
			pcl::PointCloud<Velodyne_32_HDLE>& g_seeds_pc);
	void estimate_plane_2(const pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc);
	void RemoveGround_Thread2(pcl::PointCloud<Velodyne_32_HDLE>& cloudIn,
			pcl::PointCloud<Velodyne_32_HDLE>& cloudgc,
			pcl::PointCloud<Velodyne_32_HDLE>& cloudngc,
			pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc1,
			pcl::PointCloud<Velodyne_32_HDLE>& g_not_ground_pc1);
	void RemoveGround2(pcl::PointCloud<Velodyne_32_HDLE>& cloudIn,
			pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc,
			pcl::PointCloud<Velodyne_32_HDLE>& g_not_ground_pc);

private:

	mutex regionmutex;
	int num_iter_ = 3;
	int num_lpr_ = 20;
	double th_seeds_ = 1.0;
	double th_dist_ = 0.15;
	int num_seg_ = 3;

};

#endif
