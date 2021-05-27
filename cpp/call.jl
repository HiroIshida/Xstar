const mylib = joinpath(pwd(), "libc_reeds_shepp.so")
c_create_rsspace(r) = ccall((:create_rsspace, mylib), Ptr{Cvoid}, (Cdouble,), r)
c_compute_rsdist(ptr, p1, p2) = ccall((:compute_dist, mylib), Cdouble, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble},), ptr, p1, p2)

struct ReedsSheppMetric
    ptr::Ptr{Nothing}
end
ReedsSheppMetric(r::Float64) = ReedsSheppMetric(c_create_rsspace(r))
(metric::ReedsSheppMetric)(p1, p2) = c_compute_rsdist(metric.ptr, p1, p2)

metric = ReedsSheppMetric(1.0)
metric([0, 0, 0], [1., 1., 1.])
