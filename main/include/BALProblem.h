
class BALProblem {
public:
    ~BALProblem();

    int num_observations()       const ;
    int num_points()			   const ;
    const double* observations() const ;
    double* mutable_cameras()          ;
    double* mutable_points()           ;
    double* mutable_points(int i)      ;
    double camera_0_xy(int i)          ;
    int point_index(int i)		     ;
    int *feature_color(int i)          ;
    double* mutable_camera_for_observation(int i) ;
    double* mutable_point_for_observation(int i) ;
    bool LoadFile(const char* filename);
    double* camera_0_observation;

private:
    template<typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value);
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;
    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
    int* feature_color_;

} ;

