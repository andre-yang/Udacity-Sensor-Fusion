// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>



template <PointT>
class KdTree
{
	Node* root;

public:
	KdTree()
	: root(NULL)
	{}

	void insert(PointT point, int id)
	{
		if (this->root == NULL)
		{
			root = new Node(point, id);
		} else {
			Node *node (new Node(point, id));
			insertKd(root, node, 0);
		}
	}

	void insertKd(Node *&curr_node, Node *&node, int depth) {
		int axis_compare = depth % 3;

		if (node->point[axis_compare] < curr_node->point[axis_compare]) {
			if (curr_node->left == NULL) {
				curr_node->left = node;
			} else {
				insertKd(curr_node->left, node, depth + 1);
			}
		} else {
			if (curr_node->right == NULL) {
				curr_node->right = node;
			} else {
				insertKd(curr_node->right, node, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		search_recur(ids, root, target, distanceTol, 0);
		return ids;
	}
	
	void search_recur(std::vector<int> &ids, Node *&curr_node, std::vector<float> &target, float distanceTol, int depth) {
		
		if ( (curr_node->point[0] <= (target[0] + distanceTol) ) &&
			 (curr_node->point[0] >= (target[0] - distanceTol) ) &&
			 (curr_node->point[1] <= (target[1] + distanceTol) ) &&
			 (curr_node->point[1] >= (target[1] - distanceTol) ) && 
             (curr_node->point[2] <= (target[2] + distanceTol) ) &&
			 (curr_node->point[2] >= (target[2] - distanceTol) ))
		{
			if ( pow(pow(curr_node->point[0]-target[0],2) + 
                     pow(curr_node->point[1]-target[1],2) + 
                     pow(curr_node->point[2]-target[2],2), 0.5) <= distanceTol ) {
				ids.push_back(curr_node->id);
			}
		}

		// search left if the bounding box is within the left side of the splitting line, and vice versa
		int axis_compare = depth % 3;
		if ((target[axis_compare] - distanceTol) < curr_node->point[axis_compare]) {
			if (curr_node->left != NULL) {
				search_recur(ids, curr_node->left, target, distanceTol, depth+1);	
			}
		} 
		
		if ((target[axis_compare] + distanceTol) > curr_node->point[axis_compare]) {
			if (curr_node->right != NULL) {
				search_recur(ids, curr_node->right, target, distanceTol, depth+1);	
			}
		}
	}

};


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // voxelgrid filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    // cropbox filtering 
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_croppbed (new typename pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cb;
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud_filtered);
    cb.filter (*cloud_filtered_croppbed);

    // filtering out roof points found on the top of the ego vehicle
    std::vector<int> idxs;
    pcl::CropBox<PointT> roof_cb(true);
    roof_cb.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof_cb.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof_cb.setInputCloud(cloud_filtered_croppbed);
    roof_cb.filter (idxs);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : idxs) {
        inliers->indices.push_back(point);
    }

    // extract the roof points out of the filtered and cropped pcl
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered_croppbed);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered_croppbed);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered_croppbed;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road_cloud(new typename pcl::PointCloud<PointT>), 
                                        obstacle_cloud(new typename pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road_cloud, obstacle_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneMy(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndices: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (int index : getIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Proximity(point,cluster):
void proximity(const std::vector<std::vector<float>>& points, const std::vector<float> &point, std::vector<std::vector<int>> & clusters, std::vector<int> &new_cluster, std::set<std::vector<float>> & is_point_processed, KdTree* tree, float distanceTol)
{
    //mark point as processed
	auto it = find(points.begin(), points.end(), point);
	int idx = 0;
	if (it != points.end()) {
		idx = std::distance(points.begin(), it);
	}
	is_point_processed.insert(point);

    //add point to cluster
	new_cluster.push_back(idx);

    //nearby points = tree(point)
	std::vector<int> nearby_points_idxs = tree->search(point, distanceTol);

    //Iterate through each nearby point
	for (auto nearby_point_idx : nearby_points_idxs) {
        //If point has not been processed
		if (is_point_processed.find(points[nearby_point_idx]) == is_point_processed.end()) {
            //Proximity(cluster)
			proximity(points, points[nearby_point_idx], clusters, new_cluster, is_point_processed, tree, distanceTol);
		}
	}

}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::set<std::vector<float>> is_point_processed;

	for (auto point : points) {
		if (is_point_processed.find(point) == is_point_processed.end()) {
			std::vector<int> new_cluster;
			proximity(points, point, clusters, new_cluster, is_point_processed, tree, distanceTol);
			clusters.push_back(new_cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringMy(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    euclideanCluster();

    for (pcl::PointIndices getIndices: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (int index : getIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}