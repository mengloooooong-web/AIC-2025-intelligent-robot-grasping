#ifndef _PC_FILTER_H_
#define _PC_FILTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void pc_remove_outlier(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
	                int iSizetolerance = 20, float fMaxPercent = 0.1f);


#endif
