#ifdef __cplusplus
extern "C"{
#endif
  void* rsspace_create(double r);
  void rsspace_delete(void* ptr_space);
  double compute_dist(void* ptr_space, double x1[3], double x2[3]);
  void sample_points(void* ptr_space, double x1[3], double x2[3], ReedsSheppPathSamplingCallback f, double* arr);

  void* rspath_create(void* ptr_space, double q0[3], double q1[3]);
  double rspath_distance(void* ptr_path);
  double rspath_interpolate(void* ptr_path, void* ptr_space, double q0[3], double seg, double* q_out);
  void rspath_delete(void* ptr_path);


#ifdef __cplusplus
}
#endif
