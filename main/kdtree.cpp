#include "include/funcs.h"
void kdtree ()
{
 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("temp.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  searchPoint =  cloud->points[0];
  // K nearest neighbor search

  int K = 5;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);


  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "index :"<< pointIdxNKNSearch[i] << std::endl
                << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }
 
}
