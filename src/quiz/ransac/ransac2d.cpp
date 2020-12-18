/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <pcl/filters/random_sample.h>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	pcl::RandomSample<pcl::PointXYZ> sample;
	sample.setInputCloud(cloud);
	sample.setSample(2);
	int cloud_size = cloud->points.size();
	int max_inlier_count = 0;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		
		// Randomly sample subset and fit line
		//pcl::PointCloud<pcl::PointXYZ>::Ptr out_sample;
		//sample.apply_filter(out_sample);
		int p1_idx = rand() % cloud_size;

		int p2_idx = rand() % cloud_size;
		while (p2_idx == p1_idx) {
			p2_idx = rand() % cloud_size;
		}
		pcl::PointXYZ &p1 = cloud->points[p1_idx];
		pcl::PointXYZ &p2 = cloud->points[p2_idx];

		float A = p1.y - p2.y;
		float B = p2.x - p1.x;
		float C = p1.x*p2.y - p2.x*p1.y;
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		std::vector<int> inliers_list;
		for (int j = 0; j < cloud_size; j++) {
			if ((j == p1_idx) || (j == p2_idx)) {
				continue;
			}
			pcl::PointXYZ &p = cloud->points[j];
			float d = fabs(A*p.x + B*p.y + C) / sqrt(pow(A,2) + pow(B,2));
			if (d < distanceTol) {
				inlier_count += 1;
				inliers_list.push_back(j);
			}
		}
		
		if (inlier_count > max_inlier_count) {
			max_inlier_count = inlier_count;
			inliersResult.clear();
			for (int idx : inliers_list) {
				inliersResult.insert(idx);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
	}
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	pcl::RandomSample<pcl::PointXYZ> sample;
	sample.setInputCloud(cloud);
	sample.setSample(2);
	int cloud_size = cloud->points.size();
	int max_inlier_count = 0;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		
		// Randomly sample subset and fit line
		int p1_idx = rand() % cloud_size;

		int p2_idx = rand() % cloud_size;
		while (p2_idx == p1_idx) {
			p2_idx = rand() % cloud_size;
		}

		int p3_idx = rand() % cloud_size;
		while (p3_idx == p1_idx || p3_idx == p2_idx) {
			p3_idx = rand() % cloud_size;
		}
		pcl::PointXYZ &p1 = cloud->points[p1_idx];
		pcl::PointXYZ &p2 = cloud->points[p2_idx];
		pcl::PointXYZ &p3 = cloud->points[p3_idx];

		float A = (p2.y - p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
		float D = -(A*p1.x + B*p2.y + C*p1.z);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		std::vector<int> inliers_list;
		for (int j = 0; j < cloud_size; j++) {
			if ((j == p1_idx) || (j == p2_idx)) {
				continue;
			}
			pcl::PointXYZ &p = cloud->points[j];
			float d = fabs(A*p.x + B*p.y + C*p.z + D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
			if (d < distanceTol) {
				inlier_count += 1;
				inliers_list.push_back(j);
			}
		}
		
		if (inlier_count > max_inlier_count) {
			max_inlier_count = inlier_count;
			inliersResult.clear();
			for (int idx : inliers_list) {
				inliersResult.insert(idx);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
	}
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	pcl::RandomSample<pcl::PointXYZI> sample;
	sample.setInputCloud(cloud);
	sample.setSample(2);
	int cloud_size = cloud->points.size();
	int max_inlier_count = 0;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		
		// Randomly sample subset and fit line
		int p1_idx = rand() % cloud_size;

		int p2_idx = rand() % cloud_size;
		while (p2_idx == p1_idx) {
			p2_idx = rand() % cloud_size;
		}

		int p3_idx = rand() % cloud_size;
		while (p3_idx == p1_idx || p3_idx == p2_idx) {
			p3_idx = rand() % cloud_size;
		}
		pcl::PointXYZI &p1 = cloud->points[p1_idx];
		pcl::PointXYZI &p2 = cloud->points[p2_idx];
		pcl::PointXYZI &p3 = cloud->points[p3_idx];

		float A = (p2.y - p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
		float D = -(A*p1.x + B*p2.y + C*p1.z);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		std::vector<int> inliers_list;
		for (int j = 0; j < cloud_size; j++) {
			if ((j == p1_idx) || (j == p2_idx)) {
				continue;
			}
			pcl::PointXYZI &p = cloud->points[j];
			float d = fabs(A*p.x + B*p.y + C*p.z + D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
			if (d < distanceTol) {
				inlier_count += 1;
				inliers_list.push_back(j);
			}
		}
		
		if (inlier_count > max_inlier_count) {
			max_inlier_count = inlier_count;
			inliersResult.clear();
			for (int idx : inliers_list) {
				inliersResult.insert(idx);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
	}
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
