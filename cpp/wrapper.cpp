#include "reeds_shepp.h"
#include "wrapper.h"


void* create_rsspace(double r)
{
  auto ptr = static_cast<void*>(new ReedsSheppStateSpace(r));
  return ptr;
}

double compute_dist(void* ptr, double x1[3], double x2[3]){
  auto space = static_cast<ReedsSheppStateSpace*>(ptr);
  return space->distance(x1, x2);
}

int sample_callback(double x[3], void* user_data_){
  auto user_data = static_cast<double*>(user_data_);
  for(int i=0; i<3; i++){
    user_data[i] = x[i];
  }
}

void sample_points(void* ptr, double x1[3], double x2[3], ReedsSheppPathSamplingCallback f, double* arr){
  auto space = static_cast<ReedsSheppStateSpace*>(ptr);
  space->sample(x1, x2, 0.1, f, arr);
}

void* rspath_create(void* ptr, double q0[3], double q1[3]){
  auto space = static_cast<ReedsSheppStateSpace*>(ptr);
  ReedsSheppStateSpace::ReedsSheppPath* path_ptr = new ReedsSheppStateSpace::ReedsSheppPath(space->reedsShepp(q0, q1));
  return static_cast<void*>(path_ptr);
}

double rspath_distance(void* ptr){
  auto path = static_cast<ReedsSheppStateSpace::ReedsSheppPath*>(ptr);
  return path->length();
}
