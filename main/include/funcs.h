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
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "BALProblem.h"

using ceres::CostFunction;
using ceres::CauchyLoss;
using namespace std;

struct pointinf
{
        int twoD[2];
        Eigen::Vector3d v;
        int vxvy[2];
        };


void writeplywj( char* outfile_name,BALProblem& bal_problem,float focal);

std::vector< std::vector<int> > kdtree (vector<pointinf> refmodel);

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a,
                            double epsilon = std::numeric_limits<double>::epsilon());

Eigen::MatrixXf pinvA(std::vector<pointinf> refmodel,std::vector<int> ring,int id);

Eigen::MatrixXd corres(vector<pointinf> refmodel,int image_height,int image_width);


std::vector<Eigen::Vector3d> all_thetas(std::vector<pointinf>* refmodel_p,
                                        std::vector< std::vector<int> >* rings);

Eigen::Vector3d thetai(std::vector<pointinf>* refmodel_p,std::vector<int> ringi);

Eigen::MatrixXd GV(std::vector<pointinf>* refmodel_p,std::vector<int> ringi);



