#include "./include/funcs.h"
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
