#include <iostream>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>

#include "pc_filter.h"

void pc_remove_outlier(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
	int iSizetolerance, float fMaxPercent)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud_in);
	vg.setLeafSize(10.0f, 10.0f, 10.0f);
	vg.filter(*cloud_c);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_c);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(iSizetolerance); // 2cm
	//ec.setMinClusterSize(100);
	//ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_c);
	ec.extract(cluster_indices);

	pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

	int iMax = cluster_indices[0].indices.size();
	for (int i = 1; i < cluster_indices.size(); i++)
	{
		if (iMax < cluster_indices[i].indices.size())
			iMax = cluster_indices[i].indices.size();
	}
	int iThresh = int(iMax * fMaxPercent);
	std::cout << "Thresh: " << iThresh;
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		std::cout << "\t cl " << i << "," << cluster_indices[i].indices.size();
		if (cluster_indices[i].indices.size() < iThresh)
		{

			outliers->indices.insert(outliers->indices.end(), cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
			std::cout << " del ";
		}
	}
	std::cout << std::endl;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_c);
	extract.setIndices(outliers);
	extract.setNegative(true);
	extract.filter(*cloud_out);

}
