#include "../include/PclMethods.h"

#define RANSAC_THRESHOLD 0.1

//filter around point in the given radius
pcl::PointIndices::Ptr filterDueCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr basis,  pcl::ModelCoefficients::Ptr coefficients)
{

		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	float a = coefficients->values[0], b = coefficients->values[1], c = coefficients->values[2], d = coefficients->values[3];
		  std::cerr << "Model coefficients filter: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

	for (int i = 0; i < basis->points.size(); i++) 
	{ 
			pcl::PointXYZ searchPoint = basis->points[i] ; 
			float res = abs(a*searchPoint.x + b*searchPoint.y + c*searchPoint.z + d);
		
			if (res < RANSAC_THRESHOLD)
				inliers->indices.push_back(i);
	}

	return inliers;
}

pcl::PointIndices::Ptr filterAroundPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basis, pcl::PointCloud<pcl::PointXYZ>::Ptr point, double radius)
{

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (basis); // 
	size_t removedIndices = 0;
	inliers->header = basis->header;
	inliers->indices.clear();
	for (int i = 0; i < point->points.size(); i++) 
	{ 
	pcl::PointXYZ searchPoint = point->points[i] ; 
	std::vector<int> pointIdxRadiusSearch; 
    std::vector<float> pointRadiusSquaredDistance; 

        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) 
        {       
                //set all found points to NaN 
                for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j) 
                { 
					inliers->indices.push_back(pointIdxRadiusSearch[j]);
					removedIndices++;
                } 
        }
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
	}

	
	cout << removedIndices << " indices were remove from the source cloud" << endl;
	return inliers;
}

//assuming after filter are empty
bool filterByProbability(pcl::PointCloud<pcl::PointXYZ>::Ptr basic, pcl::PointCloud<pcl::PointXYZ>::Ptr afterFilter, double threshold)
{
			
	cout << endl;
	cout << "Filtering by probability above " <<  threshold << endl;

	int precent = 0;
	size_t DivTen = basic->points.size() / 10;
	for (int i = 0; i < basic->points.size(); i++)
	{
		if ((rand() % 100 ) > threshold) 
			afterFilter->points.push_back(basic->points[i]);

		if ((DivTen != 0) && (i % DivTen) == 0)
		{
			printf("\r%d%%", (precent));
			precent += 10;
		}
	}
	cout << endl;
	std::cout << "Before: " << basic->points.size() << ", After: " << afterFilter->points.size () << std::endl; //*
	cout << endl;

	return true;
}

bool filterByVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr basic, pcl::PointCloud<pcl::PointXYZ>::Ptr afterFilter)
{
		
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (basic);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*afterFilter);
  std::cout << "Before: " << basic->points.size() << ", PointCloud after filtering has: " << afterFilter->points.size ()  << " data points." << std::endl; //*


	return true;
}


void segmentPlaneFromPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{

	pcl::SACSegmentation<pcl::PointXYZ> seg;
// Optional
	seg.setOptimizeCoefficients (true);
  // Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (RANSAC_THRESHOLD);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (basic_cloud_ptr);
    seg.segment (*inliers, *coefficients);

	cout << inliers->indices.size() << endl;
}


pcl::PointCloud<pcl::Normal>::Ptr pclNormalAndCurvature(float value, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures)
{


  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setKSearch(9);
	 ne.setNumberOfThreads(NUMBER_OF_THREADS);
  cout << "Starting to calculate normals" << endl;
  // Compute the features
  ne.compute (*cloud_normals);

//curveture
  cout << "Starting to calculate Curvature" << endl;

  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

  // Provide the original point cloud (without normals)
  principal_curvatures_estimation.setInputCloud(cloud);

  // Provide the point cloud with normals
  principal_curvatures_estimation.setInputNormals(cloud_normals);

  // Use the same KdTree from the normal estimation
  principal_curvatures_estimation.setSearchMethod(tree);
  principal_curvatures_estimation.setKSearch(20);

  // Actually compute the principal curvatures
  principal_curvatures_estimation.compute(*principal_curvatures);

  pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];

  return  cloud_normals;
}



double  relateNearGrad(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius_distance, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree)
{
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

	    pcl::PointXYZ searchPoint;
		searchPoint.x = cloud->points[i].x; searchPoint.x = cloud->points[i].y; searchPoint.x = cloud->points[i].z;
				
		ofVec3f pointNormal( normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
		ofVec3f sumVecs;
		if ( kdtree.radiusSearch (searchPoint, radius_distance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )		{

			for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			{
				sumVecs += ofVec3f(normals->points[pointIdxRadiusSearch[i]].normal_x, normals->points[pointIdxRadiusSearch[i]].normal_y, normals->points[pointIdxRadiusSearch[i]].normal_z); 

			}
	 }

		return sumVecs.normalize().dot(pointNormal);
}