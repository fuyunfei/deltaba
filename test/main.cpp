#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

using namespace std;
struct pointinf
{ 
	int twoD[2];
	double v[3];
	int vxvy[2];
	};



std::vector< std::vector<int> > kdtree (vector<pointinf> refmodel){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width    = refmodel.size();
    cloud->height   = 1;
    cloud->points.resize (cloud->width * cloud->height);
    for(int i=0;i<refmodel.size();i++){
        cloud->points[i].x=refmodel[i].v[0];
        cloud->points[i].y=refmodel[i].v[1];
        cloud->points[i].z=refmodel[i].v[2];
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    int K = 5;
    int size = cloud->size();
    std::vector< std::vector<int> > onering(size,vector<int>(K));

    pcl::PointXYZ searchPoint;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i=0; i<cloud->size();i++){
        searchPoint =  cloud->points[i];
        kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        onering[i] = pointIdxNKNSearch ;
//        for (auto j: onering[i])
//          std::cout << j << ' ';
//          std::cout<<std::endl;
    }
    return onering;
}

// generate a matrix that store correspondence bettween x,y and index of 3d position
Eigen::MatrixXd corres(vector<pointinf> refmodel,int image_height,int image_width){
    Eigen::MatrixXd cor(image_height,image_width);
    for (int i; i<refmodel.size();i++){
        cor(refmodel[i].twoD[1],refmodel[i].twoD[0])=i;
    }
    return cor;
}


Eigen::MatrixXf invA(std::vector<pointinf> refmodel,std::vector<int> ring,int id){
    int neighbor_num=ring.size();

    Eigen::MatrixXf A(neighbor_num*3,7);

    for (int i=0; i<neighbor_num;i++){
        auto v= refmodel[ring[i]].v;
        auto x=v[0]; auto y=v[1]; auto z=v[2];
        A.block(i,0,1,7)<< x,0,z,-y,1,0,0;
        A.block(i+neighbor_num,0,1,7)<<y,-z,0,x,0,1,0;
        A.block(i+2*neighbor_num,0,1,7)<<z,y,-x,0,0,0,1;
    };
   // std::cout<<pseudoInverse(A)<<std::endl;
    std::cout<<pseudoInverse(A)*A<<std::endl;
};


int main () {
  ifstream myfile;
  myfile.open ("../../pre/MyFile.txt");
  std::vector<pointinf> refmodel;
 for(string line;  getline(myfile, line); )
 {
     pointinf onerow; 
     istringstream in(line);
     in>>onerow.twoD[0]>> onerow.twoD[1]
     >>onerow.v[0]>>onerow.v[1]>>onerow.v[2]
     >>onerow.vxvy[0]>>onerow.vxvy[1];
     refmodel.push_back(onerow);
 }

std::vector< std::vector<int> > onering;
onering=kdtree(refmodel);
Eigen::MatrixXd cor;
cor=corres(refmodel,960,540);
int id=0;
invA(refmodel,onering[id],id);
return 0; 
}
