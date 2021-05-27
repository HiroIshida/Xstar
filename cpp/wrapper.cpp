#include "reeds_shepp.h"
void* create_rsspace(double r)
{
  auto ptr = static_cast<void*>(new ReedsSheppStateSpace(r));
  return ptr;
}

double compute_cost(void* ptr, double x1[3], double x2[3]){
  auto space = static_cast<ReedsSheppStateSpace*>(ptr);
  return space->distance(x1, x2);
}

int sample_callback(double x[3], void* user_data_){
  auto user_data = static_cast<double*>(user_data_);
  for(int i=0; i<3; i++){
    user_data[i] = x[i];
  }
}

void sample_points(void* ptr, double x1[3], double x2[3], double* arr){
  ReedsSheppPathSamplingCallback f;
  f = &sample_callback;
  auto space = static_cast<ReedsSheppStateSpace*>(ptr);
  space->sample(x1, x2, 0.1, f, static_cast<void*>(arr));
}

