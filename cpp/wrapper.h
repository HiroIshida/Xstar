#ifdef __cplusplus
extern "C"{
#endif
  void* create_rsspace(double r);
  double compute_dist(void* ptr, double x1[3], double x2[3]);
  void sample_points(void* ptr, double x1[3], double x2[3], ReedsSheppPathSamplingCallback f);
#ifdef __cplusplus
}
#endif
