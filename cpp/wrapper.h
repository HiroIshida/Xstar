#ifdef __cplusplus
extern "C"{
#endif
  void* create_rsspace(double r);
  double compute_dist(void* ptr, double x1[3], double x2[3]);
  void sample_points(void* ptr, double x1[3], double x2[3], ReedsSheppPathSamplingCallback f, double* arr);

  void* rspath_create(void* ptr, double q0[3], double q1[3]);
  double rspath_distance(void* ptr);


#ifdef __cplusplus
}
#endif
