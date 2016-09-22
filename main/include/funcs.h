#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <algorithm>
 

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "BALProblem.h"

using ceres::CostFunction;
using ceres::CauchyLoss;
using namespace std;

void writeplywj( char* outfile_name,BALProblem& bal_problem,float focal);
void kdtree();
