using StaticArrays
const mylib = joinpath(pwd(), "libc_reeds_shepp.so")
c_create_rsspace(r) = ccall((:create_rsspace, mylib), Ptr{Cvoid}, (Cdouble,), r)
c_compute_rsdist(ptr, p1, p2) = ccall((:compute_dist, mylib), Cdouble, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble},), ptr, p1, p2)
c_sample_points(ptr, p1, p2, cb) = ccall((:sample_points, mylib), Cvoid, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cvoid},), ptr, p1, p2, cb)

struct ReedsSheppMetric
    ptr::Ptr{Nothing}
end
ReedsSheppMetric(r::Float64) = ReedsSheppMetric(c_create_rsspace(r))
(metric::ReedsSheppMetric)(p1, p2) = c_compute_rsdist(metric.ptr, p1, p2)

function inner(q_new, arr)
    println(arr)
    #push!(arr, q_new)
    nothing
end
function sample_points!(metric::ReedsSheppMetric, p1, p2, arr)
    inner_c = @cfunction(inner, Cvoid, (Ptr{Cdouble}, Ptr{Cvoid}))
    c_sample_points(metric.ptr, p1, p2, inner_c)
end

metric = ReedsSheppMetric(1.0)
metric([0, 0, 0], [1., 1., 1.])
sample_points!(metric, [0, 0, 0], [1., 1., 1.], [])
