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


#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include "ground_remove2.h"


#define PCL_NO_PRECOMPILE
using namespace std;
const float PI = 3.1415926;




float Polar_angle_cal(float x, float y) {
	float temp_tangle = 0;
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * PI;
	}
	return temp_tangle;
}
//HSV转rgb
vector<float> hsv2rgb(vector<float>& hsv) {
	vector<float> rgb(3);
	float R, G, B, H, S, V;
	H = hsv[0];
	S = hsv[1];
	V = hsv[2];
	if (S == 0) {
		rgb[0] = rgb[1] = rgb[2] = V;
	} else {

		int i = int(H * 6);
		float f = (H * 6) - i;
		float a = V * (1 - S);
		float b = V * (1 - S * f);
		float c = V * (1 - S * (1 - f));
		i = i % 6;
		switch (i) {
		case 0: {
			rgb[0] = V;
			rgb[1] = c;
			rgb[2] = a;
			break;
		}
		case 1: {
			rgb[0] = b;
			rgb[1] = V;
			rgb[2] = a;
			break;
		}
		case 2: {
			rgb[0] = a;
			rgb[1] = V;
			rgb[2] = c;
			break;
		}
		case 3: {
			rgb[0] = a;
			rgb[1] = b;
			rgb[2] = V;
			break;
		}
		case 4: {
			rgb[0] = c;
			rgb[1] = a;
			rgb[2] = V;
			break;
		}
		case 5: {
			rgb[0] = V;
			rgb[1] = a;
			rgb[2] = b;
			break;
		}
		}
	}

	return rgb;
}

//可视化

template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}



template<typename PointInT>
float CalculateRangeXY(const PointInT pointIn) {

	return sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y);
}

template<typename PointInT>
float CalculateRangeZXY(const PointInT pointIn) {

	return sqrt(
			pointIn.x * pointIn.x + pointIn.y * pointIn.y
					+ (pointIn.z-2.1) * (pointIn.z-2.1));
}



struct Range {

	float range_xy;
	float range_zxy;
	int ring_i;
	int frame_j;
	int count_num;

};




void CloudFilter(const pcl::PointCloud<Velodyne_32_HDLE>& cloudIn,
		pcl::PointCloud<Velodyne_32_HDLE>& cloudOut, float x_min, float x_max,
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



int total_frame = 0;

void Transform2RangeImage(const pcl::PointCloud<pcl::PointXYZI>& cloudIn,
		pcl::PointCloud<Velodyne_32_HDLE>& ng_cloudOut,
		pcl::PointCloud<Velodyne_32_HDLE>& g_cloudOut, unordered_map<int, Range> &unordered_map_out) {
     
	total_frame = int(cloudIn.points.size() / 32) - 1;
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr cloud2range(
				new pcl::PointCloud<Velodyne_32_HDLE>);

	pcl::PointCloud<Velodyne_32_HDLE>::Ptr cloud_filtered(
				new pcl::PointCloud<Velodyne_32_HDLE>);
        cloud2range->points.resize(cloudIn.points.size());

	for (int j = 0; j < total_frame; ++j) {
		int num_odd = 16;                //基数从16位开始排
		int num_even = 0;                //偶数从头

		for (int i = 0; i < 32; ++i) {
			if (float(i % 2) == 0.0) {
				cloud2range->points[j * 32 + i].x = cloudIn.points[j * 32 + i].x ;
				cloud2range->points[j * 32 + i].y = cloudIn.points[j * 32 + i].y ;
				cloud2range->points[j * 32 + i].z = cloudIn.points[j * 32 + i].z ;
				cloud2range->points[j * 32 + i].ring =num_even;
				cloud2range->points[j * 32 + i].pcaketnum =j;
				num_even++;
			} else {
				cloud2range->points[j * 32 + i].x  = cloudIn.points[j * 32 + i].x;
				cloud2range->points[j * 32 + i].y = cloudIn.points[j * 32 + i].y ;
				cloud2range->points[j * 32 + i].z = cloudIn.points[j * 32 + i].z ;
				cloud2range->points[j * 32 + i].ring =num_odd;
				cloud2range->points[j * 32 + i].pcaketnum =j;
				num_odd++;
			}
		} //按索引顺序排列

        }

	cloud2range->height = 1;
	cloud2range->width = cloud2range->points.size();


	float xmin = -35, xmax = 35, ymin = -30, ymax = 30, zmin = -1.0, zmax = 3.0;
        CloudFilter(*cloud2range, *cloud_filtered, xmin, xmax, ymin, ymax, zmin, zmax);

        GroundRemove2 ground_remove(3, 20, 1.0, 0.15);
         
        ground_remove.RemoveGround2(*cloud_filtered, g_cloudOut, ng_cloudOut);


	float x_limit_min = -1, x_limit_max = 1, y_limit_forward = 4.0,
			y_limit_backward = -3;

        for(int i=0; i<ng_cloudOut.points.size(); ++i){

            float x = ng_cloudOut.points[i].x;
	    float y = ng_cloudOut.points[i].y;
	    float z = ng_cloudOut.points[i].z;
       	    float distance = CalculateRangeXY( ng_cloudOut.points[i]);
            /*if (distance > 30)
				continue;
			if ((x > x_limit_min && x < x_limit_max && y > y_limit_backward
					&& y < y_limit_forward))
				continue;

			if ((z < -1 || z > 3))
				continue;*/


                        int ringnum = ng_cloudOut.points[i].ring;

			int image_index = ng_cloudOut.points[i].pcaketnum * 32 + (31 - ringnum);
			Range r;
			r.ring_i = 31 -ringnum;
			r.frame_j = ng_cloudOut.points[i].pcaketnum;
			r.count_num = i;
			r.range_xy = ng_cloudOut.points[i].z;
			r.range_zxy = CalculateRangeZXY(ng_cloudOut.points[i]);
			unordered_map_out.insert(make_pair(image_index, r));

        }


			


}



void find_neighbors(int Ix, int Iy, vector<int>& neighborudindex,
		vector<int>& neighborlrindex) {
	//cout<<"silinyu " << Ix<< " "<<Iy<<endl;
	for (int x = Ix - 2; x <= Ix + 2; ++x) {
		//cout<<"DDDDDDw "<<x<<endl;
		if (x == Ix)
			continue;
		int px = x;
		if (x < 0) {
			px = total_frame-1;
                        //cout<<"LL"<<endl;
		}
		if (x > total_frame-1) {
			px = 0;
                       // cout<<"RR"<<endl;
		}
		//cout<<px << " "<<Iy<<endl;
		neighborlrindex.push_back(px * 32 + Iy); 
	}

	for (int y = Iy - 1; y <= Iy + 1; ++y) {
		if (y == Iy)
			continue;
		if (y < 0 || y > 31)
			continue;
		//cout<<Ix << " "<<y<<endl;
		neighborudindex.push_back((Ix * 32 + y)); 
	}
}

void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}


bool compare_index(pair<int, Range> a, pair<int, Range> b) {
	return a.first < b.first;
} //升序

vector<int> range_cluster(unordered_map<int, Range> &unordered_map_in, double ringnum) {

	int current_cluster = 0;
	vector<int> cluster_indices = vector<int>(total_frame * 32, -1);
	float horizon_angle_resolution = 0.16 * PI / 180;
	float vertical_angle_resolution = 1.33 * PI / 180;
        vector<pair<int, Range>> tr(unordered_map_in.begin(), unordered_map_in.end());
        sort(tr.begin(), tr.end(), compare_index);
        for(int i =0 ; i< tr.size(); ++i){
        }

	int for_num = 0;
	float theta_thresh = 8 * PI / 180;
	float theta_thresh2 =30 * PI / 180;

	for (int i = 0; i < tr.size(); ++i) {
			unordered_map<int, Range>::iterator it_find;
			it_find = unordered_map_in.find(tr[i].first);


			if (it_find != unordered_map_in.end() && cluster_indices[tr[i].first] == -1) {
				queue<vector<int>> q;
				vector<int> indexxy(2);
				indexxy[0] = it_find->second.frame_j;
				indexxy[1] = it_find->second.ring_i;
				q.push(indexxy);
				while (q.size()>0) {

					if (cluster_indices[q.front()[0] * 32 + q.front()[1]]
							!= -1) {
					        q.pop();
						continue;
					}
 
					cluster_indices[q.front()[0] * 32 + q.front()[1]] =
							current_cluster;
					vector<int> neighborudid;
					vector<int> neighborlfid;
					unordered_map<int, Range>::iterator it_findo;
					it_findo = unordered_map_in.find(q.front()[0] * 32 + q.front()[1]);
					find_neighbors(q.front()[0], q.front()[1], neighborudid,
							neighborlfid);

					if (neighborudid.size() > 0) {
						for (int in = 0; in < neighborudid.size(); ++in) {
							unordered_map<int, Range>::iterator it_findn;
							it_findn = unordered_map_in.find(neighborudid[in]);
							if (it_findn != unordered_map_in.end()) {
								float d1 = max(it_findo->second.range_zxy,
										it_findn->second.range_zxy);
								float d2 = min(it_findo->second.range_zxy,
										it_findn->second.range_zxy);


								float angle = fabs((float) atan2(d2* sin(vertical_angle_resolution),d1- d2* cos(vertical_angle_resolution)));
                                                                float dmax = (it_findo->second.range_zxy) * sin(1.33*PI/180)/sin(50*PI/180 -1.33*PI/180) + 3*0.2;
							        if (it_findo->second.range_xy>1.2 && fabs(d2-d1) < dmax) {
									vector<int> indexxy(2);
									indexxy[0] = it_findn->second.frame_j;
									indexxy[1] = it_findn->second.ring_i;
									q.push(indexxy);
								}else if (angle > theta_thresh2) {
									vector<int> indexxy(2);
									indexxy[0] = it_findn->second.frame_j;
									indexxy[1] = it_findn->second.ring_i;
									q.push(indexxy);
								}
							}
						}
					}

					if (neighborlfid.size() > 0) {
						for (int in = 0; in < neighborlfid.size(); ++in) {
							unordered_map<int, Range>::iterator it_findn;
							it_findn = unordered_map_in.find(neighborlfid[in]);
							if (it_findn != unordered_map_in.end()) {
								float d1 = max(it_findo->second.range_zxy,
										it_findn->second.range_zxy);
								float d2 = min(it_findo->second.range_zxy,
										it_findn->second.range_zxy);

								float angle = fabs((float) atan2(d2* sin(horizon_angle_resolution),d1- d2* cos(horizon_angle_resolution)));
                                                                float dmax = (it_findo->second.range_zxy) * sin(360/ringnum*PI/180)/sin(30*PI/180 -360/ringnum*PI/180) + 3*0.2;
							        //if (fabs(it_findo->second.range_zxy-it_findn->second.range_zxy) < dmax) {
								if (angle > theta_thresh) {
									//cluster_indices[neighborlfid[in]] =
											//current_cluster;
									vector<int> indexxy(2);
									indexxy[0] = it_findn->second.frame_j;
									indexxy[1] = it_findn->second.ring_i;
                                                                        //cout<<"LLL2 "<<indexxy[0]<<" "<<indexxy[1]<<endl;
									q.push(indexxy);
								}
							}
						}
					}
					q.pop();
				}
				current_cluster++;
			}

	}

	return cluster_indices;
}

bool compare_cluster(pair<int, int> a, pair<int, int> b) {
	return a.second < b.second;
} //升序

bool most_frequent_value(vector<int> values, vector<int> &cluster_index) {
	unordered_map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		} else {
			histcounts[values[i]] += 1;
		}
	}

	int max = 0, maxi;
	vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
	sort(tr.begin(), tr.end(), compare_cluster);
	for (int i = 0; i < tr.size(); ++i) {
		if (tr[i].second > 10) {
			cluster_index.push_back(tr[i].first);
		}
	}

	return true;
}










int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr cloud_ng(
			new pcl::PointCloud<Velodyne_32_HDLE>);
	pcl::PointCloud<Velodyne_32_HDLE>::Ptr cloud_g(
			new pcl::PointCloud<Velodyne_32_HDLE>);

	pcl::PointCloud<Velodyne_32_HDLE>::Ptr cloud_new(
			new pcl::PointCloud<Velodyne_32_HDLE>);

	ifstream inputfile;
	inputfile.open(argv[1], ios::binary);
	if (!inputfile) {
		cerr << "Open input file error!" << endl;
		exit(-1);
	}

	inputfile.seekg(0, ios::beg);

	for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
		pcl::PointXYZI p;
		inputfile.read((char *) &p.x, 3 * sizeof(float));
		inputfile.read((char *) &p.intensity, sizeof(float));
		cloud->points.push_back(p);
	}

	cloud->height = 1;
	cloud->width = cloud->points.size();

        double ringnum=(cloud->points.size()/32)-1;
	unordered_map<int, Range> range_image;
	Transform2RangeImage(*cloud, *cloud_ng, *cloud_g, range_image);


	cloud_ng->height = 1;
	cloud_ng->width = cloud_ng->points.size();
	cloud_ng->is_dense = false;

	cloud_g->height = 1;
	cloud_g->width = cloud_ng->points.size();
	cloud_g->is_dense = false;
	vector<int> cluster_index = range_cluster(range_image,ringnum);
	cv::Mat bvimage = cv::Mat::zeros(32, total_frame, CV_8UC1);
	cv::Mat range_imagec = cv::Mat::zeros(32, total_frame, CV_8UC3);

	for (auto it = range_image.begin(); it != range_image.end(); ++it) {
		int index = it->second.count_num;
		//cout<<"final "<<index<<endl;
		cloud_new->points.push_back(cloud_ng->points[index]);
		float anglepix = Polar_angle_cal(cloud_ng->points[index].x,
				cloud_ng->points[index].y);
		bvimage.at<uchar>(it->second.ring_i, it->second.frame_j) =
				it->second.range_zxy / 30 * 256;

	}
	cv::resize(bvimage, bvimage, cv::Size(1000, 100));
        cv::imwrite("range_image2.png", bvimage);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("pcd")); //PCLVisualizer 可视化类
	viewer->setBackgroundColor(0.8, 0.8, 0.8);
	viewer->addCoordinateSystem(1);
	//pcl::visualization::PointCloudColorHandlerCustom <Velodyne_32_HDLE
			//> color(cloud_g, 0,0, 0);
	//viewer->addPointCloud(cloud_g, color, "cloud");
	vector<int> cluster_id;
	most_frequent_value(cluster_index, cluster_id);
        cv::RNG rng(12345);

	for (int k = 0; k < cluster_id.size(); ++k) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colorcloud2(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		vector<float> hsv(3);
		hsv[0] = float(k) / float(cluster_id.size());
		hsv[1] = 1;
		hsv[2] = 1;

                int r = rng.uniform(0, 255);
                int g = rng.uniform(0, 255);
                int b = rng.uniform(0, 255);
		vector<float> rgb = hsv2rgb(hsv);
		for (int i = 0; i < total_frame; ++i) {
			for (int j = 0; j < 32; ++j) {
				if (cluster_index[i * 32 + j] == cluster_id[k]
						&& cluster_id[k] != -1) {
					unordered_map<int, Range>::iterator it_find;
					it_find = range_image.find(i * 32 + j);
					if (it_find != range_image.end()) {
						pcl::PointXYZRGB p;
						p.x = cloud_ng->points[it_find->second.count_num].x;
						p.y = cloud_ng->points[it_find->second.count_num].y;
						p.z = cloud_ng->points[it_find->second.count_num].z;
						p.r = 255;
						p.g = 0;
						p.b = 0;
						Colorcloud2->points.push_back(p);
                                                range_imagec.at<cv::Vec3b>(it_find->second.ring_i, it_find->second.frame_j) = cv::Vec3b(r,g,b);
					}
				}
			}
		}

		if (Colorcloud2->points.size() > 5) {
			Colorcloud2->height = 1;
			Colorcloud2->width = Colorcloud2->points.size();
			float xmax1 = Colorcloud2->points[0].x;
			float xmin1 = Colorcloud2->points[0].x;
			float ymax1 = Colorcloud2->points[0].y;
			float ymin1 = Colorcloud2->points[0].y;
			float zmax1 = Colorcloud2->points[0].z;
			float zmin1 = Colorcloud2->points[0].z;
			//find the xmax, xmin, ymax, ymin
			for (int i = 0; i < Colorcloud2->points.size(); ++i) {
				if (Colorcloud2->points[i].z > zmax1) {
					zmax1 = Colorcloud2->points[i].z;
				}
				if (Colorcloud2->points[i].z < zmin1) {
					zmin1 = Colorcloud2->points[i].z;
				}
				if (Colorcloud2->points[i].x > xmax1) {
					xmax1 = Colorcloud2->points[i].x;
				}
				if (Colorcloud2->points[i].x < xmin1) {
					xmin1 = Colorcloud2->points[i].x;
				}
				if (Colorcloud2->points[i].y > ymax1) {
					ymax1 = Colorcloud2->points[i].y;
				}
				if (Colorcloud2->points[i].y < ymin1) {
					ymin1 = Colorcloud2->points[i].y;
				}

			}

			if (zmin1 < 0)
				zmin1 = 0; //make sure object is up the ground

			double depth = zmax1 - zmin1;

			Eigen::Vector3f eulerAngle(0.0, 0.0, 0.0);
			Eigen::AngleAxisf rollAngle(
					Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
			Eigen::AngleAxisf pitchAngle(
					Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
			Eigen::AngleAxisf yawAngle(
					Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));

			const Eigen::Quaternionf bboxQ1(yawAngle * pitchAngle * rollAngle);

			Eigen::Vector3f translation;
			translation(0) = (xmax1 - xmin1) / 2 + xmin1;
			translation(1) = (ymax1 - ymin1) / 2 + ymin1;
			translation(2) = (zmax1 - zmin1) / 2 + zmin1;
			double length = ymax1 - ymin1;
			double width = xmax1 - xmin1;

			pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZRGB
					> color2(Colorcloud2, (b), (g),
							(r));
			viewer->addPointCloud(Colorcloud2, color2, "cloud2" + toString(k));
		}
	}

	cout << range_image.size() << endl;

	int count_num = 0;
	for (auto it = range_image.begin(); it != range_image.end(); ++it) {
		count_num++;
	}


        //cv::imshow("bv", range_imagec);
	//cv::waitKey(0);
	cv::resize(range_imagec, range_imagec, cv::Size(1000, 100));
        cv::imwrite("range_image3.png", range_imagec);
	while (!viewer->wasStopped()) {
		viewer->spin();
	}


        return 0;
}



