#include "./include/funcs.h"

// write wj to ply file
void writeplywj( char* outfile_name,BALProblem& bal_problem,float focal){
    std::ofstream myfile;
    myfile.open (outfile_name);
    myfile<<"ply \nformat ascii 1.0 \nelement vertex "<<bal_problem.num_points()<<"\nproperty float x \n";
    myfile<<"property float y \nproperty float z\nproperty uchar red \nproperty uchar green \nproperty uchar blue \nend_header\n";

    for (int i = 0; i < bal_problem.num_points(); i++){

        double wj=(*(bal_problem.mutable_point_for_observation(0)+i));
        double tempx= bal_problem.camera_0_xy(2*i);
        double tempy= bal_problem.camera_0_xy(2*i+1);
        double xj= tempx/((-focal)*wj);
        double yj= tempy/((-focal)*wj);
        double zj= 1/wj;
        int red =*(bal_problem.feature_color(i));
        int green =*(bal_problem.feature_color(i)+1);
        int blue =*(bal_problem.feature_color(i)+2);
        if(zj<20&&zj>0){
            myfile <<xj<<" "<<yj<<" "<<zj<<" ";
            myfile <<red<<" "<<green<<" "<<blue<<"\n";
        }
    }

    myfile.close();
}


// kdtrees from refmodel, input refmodel , output neighbors of all vertices .
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
    std::vector< std::vector<int> > rings(size,vector<int>(K));

    pcl::PointXYZ searchPoint;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i=0; i<cloud->size();i++){
        searchPoint =  cloud->points[i];
        kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        rings[i] = pointIdxNKNSearch ;
        //        for (auto j: onering[i])
        //          std::cout << j << ' ';
        //          std::cout<<std::endl;
    }
    return rings;
}


// generate a matrix that store correspondence bettween x,y and index of 3d position
// cor(i,j)==> index of refmodel vertex  which  can be used for (i,j) + (vx,vy)
Eigen::MatrixXd corres(vector<pointinf> refmodel,int image_height,int image_width){
    Eigen::MatrixXd cor(image_height,image_width);
    for (int i; i<refmodel.size();i++){
        cor(refmodel[i].twoD[1],refmodel[i].twoD[0])=i;
    }
    return cor;
}

// pinv() for eigen , solve the pseudo inverse for matrix
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}



// Input refmodel,index
Eigen::MatrixXd pinvA(std::vector<pointinf>* refmodel_p,std::vector<int> ring){
    int neighbor_num=ring.size();

    Eigen::MatrixXd A(neighbor_num*3,7);

    for (int i=0; i<neighbor_num;i++){
        auto v= (*refmodel_p)[ring[i]].v;
        auto x=v[0]; auto y=v[1]; auto z=v[2];
        A.block(i,0,1,7)<< x,0,z,-y,1,0,0;
        A.block(i+neighbor_num,0,1,7)<<y,-z,0,x,0,1,0;
        A.block(i+2*neighbor_num,0,1,7)<<z,y,-x,0,0,0,1;
    };
    return  pseudoInverse(A);
};


// input ring of vertex_indexi, output the specific theta of the reference model
Eigen::Vector3d thetai(std::vector<pointinf>* refmodel_p,std::vector<int> ringi){
    vector<Eigen::Vector3d> ringpos;
    for (auto i :ringi){
        ringpos.push_back( (*refmodel_p)[i].v); //  pos of neighbors
    }

    auto pos0= (*refmodel_p)[ringi[0]].v;
    for (auto i=ringpos.begin()+1;i!=ringpos.end();i++){
        pos0=pos0-*i/ringpos.size();  // calculate theta of pos0
    }
    return pos0;
}

// input ring of vertex_indexi, output all thetas of the reference model
std::vector<Eigen::Vector3d> all_thetas(std::vector<pointinf>* refmodel_p,
                                        std::vector< std::vector<int> >* rings){

    std::vector<Eigen::Vector3d>  alltheta;
    for(auto ringi: *rings){
        vector<Eigen::Vector3d> ringpos;
        for (auto i :ringi){
            ringpos.push_back( (*refmodel_p)[i].v); //  pos of neighbors
        }

        auto pos0= (*refmodel_p)[ringi[0]].v;
        for (auto i=ringpos.begin()+1;i!=ringpos.end();i++){
            pos0=pos0-*i/ringpos.size();  // calculate theta of pos0
        }
        alltheta.push_back(pos0);
    }

    return alltheta;
    std::cout<<" generate all thetas "<<std::endl;
}




// output the matrix of G,G=(thetai_of_ref*T-L). G*(Vi') = error of theta .
// Vi' equals vertex i and its neighbors position in this iteration
Eigen::MatrixXd GV(std::vector<pointinf>* refmodel_p,std::vector<int> ringi){
    int neighbor_num=ringi.size();
    Eigen::MatrixXd invA=pinvA(refmodel_p,ringi);
    Eigen::Vector3d theta_i=thetai(refmodel_p,ringi);
    Eigen::MatrixXd deltaforT(3,3);
    deltaforT << theta_i(1),theta_i(1),theta_i(1),
            theta_i(1),theta_i(1),theta_i(1),
            theta_i(1),theta_i(1),theta_i(1);
    Eigen::MatrixXd deltaT= deltaforT*invA.block(0,0,3,neighbor_num*3);
    Eigen::MatrixXd L(3,neighbor_num*3);
    L=Eigen::MatrixXd::Zero(3,neighbor_num*3);
    L(0,0)= 1.0- neighbor_num;
    L(1,neighbor_num)= 1.0- neighbor_num ;
    L(2,neighbor_num*2)= 1.0- neighbor_num;
    L.block(0,1,1,neighbor_num-1).setOnes();
    L.block(1,neighbor_num,1,neighbor_num-1).setOnes();
    L.block(1,neighbor_num*2,1,neighbor_num-1).setOnes();
    L=-L;
    return L-deltaT;
}











































