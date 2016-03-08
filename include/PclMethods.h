#pragma once

#include <iostream>                  // for std::cout

#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>

#include "ofVec3f.h"

#define NUMBER_OF_THREADS 3

using namespace std;

pcl::PointIndices::Ptr filterDueCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr basis,  pcl::ModelCoefficients::Ptr coefficients);
pcl::PointIndices::Ptr filterAroundPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basis, pcl::PointCloud<pcl::PointXYZ>::Ptr point, double radius);
bool filterByProbability(pcl::PointCloud<pcl::PointXYZ>::Ptr basic, pcl::PointCloud<pcl::PointXYZ>::Ptr afterFilter, double threshold);
bool filterByVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr basic, pcl::PointCloud<pcl::PointXYZ>::Ptr afterFilter);
void segmentPlaneFromPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);
double  relateNearGrad(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius_distance, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
pcl::PointCloud<pcl::Normal>::Ptr pclNormalAndCurvature(float value, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures);

