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

#include "ground_remove2.h"

MatrixXf normal_2(3, 1);
vector<float> normal2(3);
float th_dist_d_2;

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

bool point_cmp(Velodyne_32_HDLE a, Velodyne_32_HDLE b) {
	return a.z < b.z;
}

GroundRemove2::GroundRemove2(int num_iter, int num_lpr, double th_seeds,
		double th_dist) {
	num_iter_ = num_iter;
	num_lpr_ = num_lpr;
	th_seeds_ = th_seeds;
	th_dist_ = th_dist;
}

void GroundRemove2::extract_initial_seeds_2(
		const pcl::PointCloud<Velodyne_32_HDLE>& p_sorted,
		pcl::PointCloud<Velodyne_32_HDLE>& g_seeds_pc) {
	// LPR is the mean of low point representative
	double sum = 0;
	int cnt = 0;
	// Calculate the mean height value.
	for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; ++i) {
		sum += p_sorted.points[i].z;
		cnt++;
	}
	double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
	g_seeds_pc.clear();
	// iterate pointcloud, filter those height is less than lpr.height+th_seeds_
	for (int i = 0; i < p_sorted.points.size(); ++i) {
		if (p_sorted.points[i].z < lpr_height + th_seeds_) {
			g_seeds_pc.points.push_back(p_sorted.points[i]);
		}
	}
	// return seeds points
}

void GroundRemove2::estimate_plane_2(
		const pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc) {
	// Create covarian matrix in single pass.
	// TODO: compare the efficiency.
	Eigen::Matrix3f cov;
	Eigen::Vector4f pc_mean;
	vector<float> conv, mean;

	pcl::computeMeanAndCovarianceMatrix(g_ground_pc, cov, pc_mean);

	//computeMeanAndCovarianceMatrix(g_ground_pc, conv, mean);

	//static float A[9] = { conv[0], conv[1], conv[2],  conv[3],
	//conv[4], conv[5],  conv[6], conv[7], conv[8],};
	//static Eigen::MatrixXf conv2 = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> >(A);
	// Singular Value Decomposition: SVD
	JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
	// use the least singular vector as normal
	normal_2 = (svd.matrixU().col(2));
	// mean ground seeds value
	//Eigen::Vector3f seeds_mean = pc_mean.head<3>();

	// according to normal.T*[x,y,z] = -d
	//float d_ = -(normal_.transpose()*seeds_mean)(0,0);

	float d_ = -(normal_2(0, 0) * pc_mean(0) + normal_2(1, 0) * pc_mean(1)
			+ normal_2(2, 0) * pc_mean(2));

	// set distance threhold to `th_dist - d`
	th_dist_d_2 = th_dist_ - d_;

	// return the equation parameters
}

void GroundRemove2::RemoveGround_Thread2(pcl::PointCloud<Velodyne_32_HDLE>& cloudIn,
		pcl::PointCloud<Velodyne_32_HDLE>& cloudgc,
		pcl::PointCloud<Velodyne_32_HDLE>& cloudngc,
		pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc1,
		pcl::PointCloud<Velodyne_32_HDLE>& g_not_ground_pc1) {

	std::lock_guard < std::mutex > lock(regionmutex);
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_seeds_pc(
			new pcl::PointCloud<Velodyne_32_HDLE>());

	sort(cloudIn.points.begin(), cloudIn.points.end(), point_cmp);

	extract_initial_seeds_2(cloudIn, *g_seeds_pc);

	cloudgc = *g_seeds_pc;

	for (int i = 0; i < num_iter_; ++i) {

		estimate_plane_2(cloudgc);

		cloudgc.clear();
		cloudngc.clear();

		float xd = normal_2(0, 0);
		float yd = normal_2(1, 0);
		float zd = normal_2(2, 0);
		for (auto p : cloudIn.points) {
			float distance = p.x * xd + p.y * yd + p.z * zd;
			if (distance < th_dist_d_2) {
				//g_all_pc->points[r].label = 1u;// means ground
				cloudgc.points.push_back(p);
			} else {
				//g_all_pc->points[r].label = 0u;// means not ground and non clusterred
				cloudngc.points.push_back(p);
			}

		}

	}

	for (int k = 0; k < cloudgc.points.size(); ++k) {

		g_ground_pc1.points.push_back(cloudgc.points[k]);
	}

	for (int k = 0; k < cloudngc.points.size(); ++k) {

		g_not_ground_pc1.points.push_back(cloudngc.points[k]);
	}

}

void GroundRemove2::RemoveGround2(pcl::PointCloud<Velodyne_32_HDLE>& cloudIn,
		pcl::PointCloud<Velodyne_32_HDLE>& g_ground_pc,
		pcl::PointCloud<Velodyne_32_HDLE>& g_not_ground_pc) {
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_seeds_region1(
			new pcl::PointCloud<Velodyne_32_HDLE>());
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_seeds_region2(
			new pcl::PointCloud<Velodyne_32_HDLE>());
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_seeds_region3(
			new pcl::PointCloud<Velodyne_32_HDLE>());
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_ground_pc1(
			new pcl::PointCloud<Velodyne_32_HDLE>());
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr g_not_ground_pc1(
			new pcl::PointCloud<Velodyne_32_HDLE>());

	float xmin = -35, xmax = 35, ymin = -30, ymax = 30, zmin = -1.0, zmax = 3.0;
	float regionsize = (ymax - ymin) / 3;
	for (int i = 0; i < cloudIn.points.size(); ++i) {
		if (cloudIn.points[i].z < 0.75) {

			if (cloudIn.points[i].y < ymax - 0 * regionsize
					&& cloudIn.points[i].y > ymax - 1 * regionsize) {
				g_seeds_region1->points.push_back(cloudIn.points[i]);
			}
			if (cloudIn.points[i].y < ymax - 1 * regionsize
					&& cloudIn.points[i].y > ymax - 2 * regionsize) {
				g_seeds_region2->points.push_back(cloudIn.points[i]);
			}
			if (cloudIn.points[i].y < ymax - 2 * regionsize
					&& cloudIn.points[i].y > ymax - 3 * regionsize) {
				g_seeds_region3->points.push_back(cloudIn.points[i]);
			}
		} else {
			g_not_ground_pc.points.push_back(cloudIn.points[i]);
		}

	}

	cloudIn.clear();

	vector<pcl::PointCloud<Velodyne_32_HDLE>::Ptr> pcregion(3);

	pcregion[0] = g_seeds_region1;
	pcregion[1] = g_seeds_region2;
	pcregion[2] = g_seeds_region3;

	std::vector<std::thread> thread_vec(num_seg_);

	for (int ri = 0; ri < num_seg_; ++ri) {

		thread_vec[ri] = std::thread(&GroundRemove2::RemoveGround_Thread2, this,
				std::ref(*pcregion[ri]), std::ref(*g_ground_pc1),
				std::ref(*g_not_ground_pc1), std::ref(g_ground_pc),
				std::ref(g_not_ground_pc));

	}

	for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
		it->join();
	}

}


/*template<typename PointInT, typename PointOutT>
void CloudFilter(const pcl::PointCloud<PointInT>& cloudIn,
		pcl::PointCloud<PointOutT>& cloudOut, float x_min, float x_max,
		float y_min, float y_max, float z_min, float z_max) {
	cloudOut.header = cloudIn.header;
	cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
	cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
	cloudOut.points.clear();
	//1) set parameters for removing cloud reflect on ego vehicle
	float x_limit_min = -1.8, x_limit_max = 1.8, y_limit_forward = 5.0,
			y_limit_backward = -4.5;
	//2 apply the filter
	for (int i = 0; i < cloudIn.size(); ++i) {
		float x = cloudIn.points[i].x;
		float y = cloudIn.points[i].y;
		float z = cloudIn.points[i].z;
		// whether on ego vehicle
		if ((x > x_limit_min && x < x_limit_max && y > y_limit_backward
				&& y < y_limit_forward))
			continue;
		if ((x > x_min && x < x_max && y > y_min && y < y_max && z > z_min
				&& z < z_max)) {

			cloudOut.points.push_back(cloudIn.points[i]);

		}

	}
}

void Cloudcolor(const pcl::PointCloud<Velodyne_32_HDLEI>& cloudIn,
		const pcl::PointCloud<Velodyne_32_HDLEI>& cloudground,
		pcl::PointCloud<Velodyne_32_HDLERGB>& cloudOut) {

	int groundsize = cloudground.points.size();
	int ngroundsize = cloudIn.points.size();
	int size = groundsize + ngroundsize;

	for (int i = 0; i < ngroundsize; ++i) {
		Velodyne_32_HDLERGB p;
		p.x = cloudIn.points[i].x;
		p.y = cloudIn.points[i].y;
		p.z = cloudIn.points[i].z;
		p.r = 225;
		p.g = 0;
		p.b = 0;
		cloudOut.points.push_back(p);
	}

	for (int i = 0; i < groundsize; ++i) {
		Velodyne_32_HDLERGB p;
		p.x = cloudground.points[i].x;
		p.y = cloudground.points[i].y;
		p.z = cloudground.points[i].z;
		p.r = 225;
		p.g = 255;
		p.b = 255;
		cloudOut.points.push_back(p);
	}
}

class SubscribeAndPublish {
 public:
 SubscribeAndPublish(ros::NodeHandle nh, std::string lidar_topic_name,
 std::string imu_topic_name);

 void callback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg,
 const sensor_driver_msgs::GpswithHeadingConstPtr& gps_msg) {
 pcl::PointCloud<Velodyne_32_HDLEI>::Ptr cloud(
 new pcl::PointCloud<Velodyne_32_HDLEI>);
 pcl::PointCloud<Velodyne_32_HDLEI>::Ptr cloud_t(
 new pcl::PointCloud<Velodyne_32_HDLEI>);
 pcl::PointCloud<Velodyne_32_HDLEI>::Ptr cloud_f(
 new pcl::PointCloud<Velodyne_32_HDLEI>);
 pcl::PointCloud<Velodyne_32_HDLERGB>::Ptr cloud_color(
 new pcl::PointCloud<Velodyne_32_HDLERGB>);

 pcl::PointCloud<Velodyne_32_HDLEI>::Ptr g_ground_pc(
 new pcl::PointCloud<Velodyne_32_HDLEI>());
 pcl::PointCloud<Velodyne_32_HDLEI>::Ptr g_not_ground_pc(
 new pcl::PointCloud<Velodyne_32_HDLEI>());
 pcl::fromROSMsg(*cloudmsg, *cloud);

 float xmin = -35, xmax = 35, ymin = -30, ymax = 30, zmin = -1.0, zmax =
 3.0;

 CloudFilter(*cloud, *cloud_t, xmin, xmax, ymin, ymax, zmin, zmax);
 TransformKittiCloud( *cloud_t,*cloud_f);

 cloud->clear();
 int64_t tm0 = gtm();
 GroundRemove grobject(3,20,1.0,0.15);
 grobject.RemoveGround(*cloud_f,*g_ground_pc,*g_not_ground_pc);
 int64_t tm1 = gtm();
 printf("[INFO]region build cast time:%ld us\n", tm1 - tm0);

 Cloudcolor(*g_ground_pc, *g_not_ground_pc, *cloud_color);

 cloud_color->height = 1;
 cloud_color->width = cloud_color->points.size();
 cloud_color->is_dense = false;    //最终优化结果

 sensor_msgs::PointCloud2 ros_cloud;
 pcl::toROSMsg(*cloud_color, ros_cloud);
 ros_cloud.header.frame_id = "global_init_frame";
 pub_.publish(ros_cloud);
 //pcl::io::savePCDFileASCII<Velodyne_32_HDLERGB> ("test_simple.pcd", *cloud_color);
 //pcl::visualization::CloudViewer viewer("PCD2");
 //viewer.showCloud(cloud_color);

 }
 private:
 ros::NodeHandle n_;
 ros::Publisher pub_;
 ros::Subscriber sub_;
 message_filters::Subscriber<sensor_msgs::PointCloud2> Sub_Lidar;
 //message_filters::Subscriber<sensor_driver_msgs::GpswithHeading> Sub_IMU;
 message_filters::Subscriber<sensor_driver_msgs::GpswithHeading> Sub_IMU;
 typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
 sensor_driver_msgs::GpswithHeading> MySyncPolicy;
 // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
 Synchronizer<MySyncPolicy> sync;

 };

 SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle nh,
 std::string lidar_topic_name, std::string imu_topic_name) :
 n_(nh), Sub_Lidar(nh, lidar_topic_name, 10), Sub_IMU(nh, imu_topic_name,
 20), sync(MySyncPolicy(10), Sub_Lidar, Sub_IMU) {
 //Topic you want to publish
 pub_ = nh.advertise < sensor_msgs::PointCloud2 > ("/groundremove", 1);

 //Topic you want to subscribe
 //sub_ = n_.subscribe("lidar_cloud_calibrated", 1, &SubscribeAndPublish::callback, this);
 // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
 sync.registerCallback(
 boost::bind(&SubscribeAndPublish::callback, this, _1, _2));
 }


 int main(int argc, char** argv) {

 ros::init(argc, argv, "ground_node");
 SubscribeAndPublish SAPObject(ros::NodeHandle(), "lidar_cloud_calibrated",
 "gpsdata");
 ROS_INFO("waiting for data!");
 ros::spin();

 return 0;
 }*/

