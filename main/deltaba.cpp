#include "./include/funcs.h"
// Read a Bundle Adjustment in the Large dataset.
int fixedfocus;

struct WjError {
    WjError(double observed_x, double observed_y,double observed_img0_x,double observed_img0_y)
        : observed_x(observed_x), observed_y(observed_y),
          observed_img0_x(observed_img0_x), observed_img0_y(observed_img0_y)  {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    const T* const point1,
                    const T* const point2,
                    const T* const point3,
                    const T* const onlyfocal,
                    T* residuals) const {


        //const T& focal = T(onlyfocal[0]);
        const T& focal=T(fixedfocus);
        T xj= T(observed_img0_x)/(-focal);
        T yj= T(observed_img0_y)/(-focal);
        const T& wj = point[0];
        const T& wj0 = T(1)/point[0];
        const T& wj1 = T(1)/point1[0];
        const T& wj2 = T(1)/point2[0];
        const T& wj3 = T(1)/point3[0];



        // T xj= T(observed_img0_x)/(-focal); T yj = T(observed_img0_y)/(-focal);
        T pxi = T(observed_x);	 T pyi = T(observed_y);

        const T& theta_x =camera[0];
        const T& theta_y =camera[1];
        const T& theta_z =camera[2];
        const T& txi =camera[3];
        const T& tyi =camera[4];
        const T& tzi =camera[5];
        T axi = xj - theta_z*yj+ theta_y;
        T bxi = txi;
        T ayi = yj - theta_x+ theta_z*xj ;
        T byi = tyi;
        T ci  = -theta_y*xj+theta_x*yj+T(1);
        T di  = tzi;
        T exi = pxi*ci - axi*(-focal);    //focal_length
        T fxi = pxi*di - bxi*(-focal);
        T eyi = pyi*ci - ayi*(-focal);
        T fyi = pyi*di - byi*(-focal);

        residuals[0] = (exi+fxi*wj)/(ci+di*wj);
        residuals[1] = (eyi+fyi*wj)/(ci+di*wj);
        residuals[2] = (abs(wj0-wj1)+abs(wj0-wj2)+abs(wj0-wj3))*T(100);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,const double observed_y,
                                       const double observed_img0_x,const double observed_img0_y) {
        return (new ceres::AutoDiffCostFunction<WjError, 3, 9, 1, 1,1,1,1>(
                    new WjError(observed_x, observed_y,observed_img0_x,observed_img0_y)));
    }

    double observed_x;
    double observed_y;
    double observed_img0_x;
    double observed_img0_y;
};

std::vector< std::vector<int> > kdtree (BALProblem bal_problem){
    double *xy;
    int point_num=bal_problem.num_points();
    xy=bal_problem.camera_0_observation;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width    = point_num;
    cloud->height   = 1;
    cloud->points.resize (cloud->width * cloud->height);
    for(int i=0;i<point_num;i++){
        cloud->points[i].x=xy[2*i];
        cloud->points[i].y=xy[2*i+1];
        cloud->points[i].z=0;
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
    }

    //     pcl::io::savePLYFileASCII("w0.ply", *cloud);

    return onering;
}

int main(int argc, char** argv) {


    ifstream prefile;
    prefile.open ("../../pre/MyFile.txt");
    std::vector<pointinf> refmodel;
    for(string line;  getline(prefile, line); )
    {
        pointinf onerow;
        istringstream in(line);
        in>>onerow.twoD[0]>> onerow.twoD[1]
                >> onerow.v[0]>> onerow.v[1]>> onerow.v[2]
                >>onerow.vxvy[0]>>onerow.vxvy[1];
        refmodel.push_back(onerow);
    }
    //neighbors of veteces
    std::vector< std::vector<int> > rings= kdtree (refmodel);
    // all theta of the reference model
    std::vector<Eigen::Vector3d> alltheta= all_thetas(&refmodel,&rings);


    double *onlyfocal = new double [1];
    std::cout<<"initial onlyfocal"<<endl;  cin>>onlyfocal[0];
    fixedfocus=onlyfocal[0];

    BALProblem bal_problem1;
    bal_problem1.LoadFile(argv[1]);
    std::vector< std::vector<int> > onering=kdtree(bal_problem1);


    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1])) {
        std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
        return 1;
    }

    const double* observations = bal_problem.observations();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    int fuck=0;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {


        int j=bal_problem.point_index(i);
        double observations0x=  bal_problem.camera_0_xy(2*j);
        double observations0y=  bal_problem.camera_0_xy(2*j+1);
        //        float errorx = abs(observations0x-observations[2 * i + 0]);
        //        float errory = abs(observations0y-observations[2 * i + 1]);
        //        float dis = (errorx*errorx+errory*errory);
        //        if (dis>900){
        //            fuck++;
        //            //continue;
        //        }
        vector<int> ringi=onering[j];
        ringi.erase(std::remove(ringi.begin(),ringi.end(),j),ringi.end());
        auto ring_i=ringi.begin();
        ceres::CostFunction* cost_function =
                WjError::Create(observations[2 * i + 0], observations[2 * i + 1],observations0x,observations0y);
        std::cout<<*(ring_i)<<" "<<*(ring_i+1)<<" "<<*(ring_i+2)<<" "<<*(ring_i+3)<<std::endl;
        std::cout << *bal_problem.mutable_point_for_observation(i)<<" "
                  << *bal_problem.mutable_points(*(ring_i))<<" "
                  << *bal_problem.mutable_points(*(ring_i+1))<<" "
                  << *bal_problem.mutable_points(*(ring_i+2))<<" "
                  << *bal_problem.mutable_points(*(ring_i+3))<<std::endl;

        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i),
                                 bal_problem.mutable_points(*(ring_i)),
                                 bal_problem.mutable_points(*(ring_i+1)),
                                 bal_problem.mutable_points(*(ring_i+2)),
                                 onlyfocal
                                 );
    }
    std::cout<<"outlier_num="<<fuck<<std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations=150;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    writeplywj(argv[2],bal_problem,onlyfocal[0]);

    std::ofstream myfile;

    myfile.open ("ARTreport.txt");
    myfile<<onlyfocal[0]<<std::endl;
    double *campara=bal_problem.mutable_camera_for_observation(0);
    for (int i = 0; i < 200; i++){ myfile <<*(campara+i)<<"\n";}
    myfile.close();

    cout<<"onlyfocal: "<< onlyfocal[0]<<endl;
    return 0;
}
