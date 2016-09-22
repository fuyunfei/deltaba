
#include "include/funcs.h"
// Read a Bundle Adjustment in the Large dataset.

BALProblem:: ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
    delete[] camera_0_observation;
    //  delete[] feature_color_;
}

int BALProblem::num_observations()       const { return num_observations_;               }
int BALProblem::num_points()			   const { return num_points_;               }
const double*  BALProblem::  observations() const { return observations_;                   }
double* BALProblem:: mutable_cameras()          { return parameters_;                     }
double* BALProblem:: mutable_points()           { return parameters_  + 9 * num_cameras_; }
double  BALProblem::camera_0_xy(int i)          { return camera_0_observation[i];         }
int  BALProblem::point_index(int i)		     { return point_index_[i];  }
int* BALProblem:: feature_color(int i)          { return feature_color_+i*3;}

double* BALProblem:: mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
}
double* BALProblem:: mutable_point_for_observation(int i) {
    return (mutable_points() + point_index_[i]) ;  // 3-->1
}

double* BALProblem:: mutable_points(int i) {
    return (mutable_points() + i) ;  //
}

bool BALProblem:: LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
        return false;
    };
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    num_parameters_ = 9 * num_cameras_ + 1 * num_points_;  // 3-->1
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    parameters_ = new double[num_parameters_];
    camera_0_observation = new double[2 * num_points_];
    feature_color_=new int[3*num_points_];

    int countforcamera0=0;
    for (int i = 0; i < num_observations_; ++i) {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        for (int j = 0; j < 2; ++j) {
            FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
        }

        if (*(camera_index_+i)==0){
            camera_0_observation[countforcamera0]= *(observations_ + 2*i);
            camera_0_observation[countforcamera0+1]= *(observations_ + 2*i+1);
            countforcamera0=countforcamera0+2;
        }
    }
    for (int i = 0; i < num_parameters_; ++i) {
        FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    for (int i = 0; i < 3*num_points_; ++i) {
        FscanfOrDie(fptr, "%d", feature_color_ + i);
    }
    return true;
}

template<typename T>
void BALProblem:: FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
        LOG(FATAL) << "Invalid UW data file.";
    }
}


