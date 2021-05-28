import Base.finalizer
using StaticArrays
const mylib = joinpath(pwd(), "libc_reeds_shepp.so")
c_rspace_cleate(r) = ccall((:rsspace_create, mylib), Ptr{Cvoid}, (Cdouble,), r)
c_rsspace_delete(ptr) = ccall((:rsspace_delete, mylib), Cvoid, (Ptr{Cvoid},), ptr)

c_compute_rsdist(ptr, p1, p2) = ccall((:compute_dist, mylib), Cdouble, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble},), ptr, p1, p2)
c_sample_points(ptr, p1, p2, cb, arr) = ccall((:sample_points, mylib), Cvoid, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cvoid}, Ptr{Cdouble}), ptr, p1, p2, cb, arr)

c_rspath_create(ptr_space, q0, q1) = ccall((:rspath_create, mylib), 
                                         Ptr{Cvoid}, 
                                         (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble}),
                                         ptr_space, q0, q1)

c_rspath_distance(ptr_path) = ccall((:rspath_distance, mylib), Cdouble, (Ptr{Cvoid},), ptr_path)

c_rspath_interpolate(ptr_path, ptr_space, q0, seg, q_out) = ccall((:rspath_interpolate, mylib), 
                                                                  Cvoid,
                                                                  (Ptr{Cvoid}, Ptr{Cvoid}, Ptr{Cdouble}, Cdouble, Ptr{Cdouble}),
                                                                  ptr_path, ptr_space, q0, seg, q_out)
c_rspath_delete(ptr_path) = ccall((:rspath_delete, mylib), Cvoid, (Ptr{Cvoid},), ptr_path)

mutable struct ReedsSheppPath
    ptr::Ptr{Nothing}
    ptr_to_space::Ptr{Nothing}
    q0::Vector # TODO
    function ReedsSheppPath(ptr, ptr_to_space, q0)
        path = new(ptr, ptr_to_space, q0)
        delete(path_::ReedsSheppPath) = c_rspath_delete(path_.ptr)
        finalizer(delete, path)
    end
end
distance(path::ReedsSheppPath) = c_rspath_distance(path.ptr)
interpolate(path::ReedsSheppPath, seg::Float64, out::AbstractVector) = c_rspath_interpolate(path.ptr, path.ptr_to_space, path.q0, seg, out)

struct ReedsSheppMetric
    ptr::Ptr{Nothing}
    function ReedsSheppMetric(r::Float64)
        delete(metric::ReedsSheppMetric) = c_rsspace_delete(metric.ptr)
        ptr = c_rspace_cleate(r)
        new(ptr)
    end
end
(metric::ReedsSheppMetric)(p1, p2) = c_compute_rsdist(metric.ptr, p1, p2)
create_path(metric::ReedsSheppMetric, q0, q1) = ReedsSheppPath(c_rspath_create(metric.ptr, q0, q1), metric.ptr, q0)


function inner(idx, q_new, arr)
    for i in 1:3
        val = unsafe_load(q_new, i)
        unsafe_store!(arr, val, 3 * idx + i)
    end
    nothing
end
function sample_points!(metric::ReedsSheppMetric, p1, p2, arr)
    inner_c = @cfunction(inner, Cvoid, (Cint, Ptr{Cdouble}, Ptr{Cdouble}))
    c_sample_points(metric.ptr, p1, p2, inner_c, arr)
end

metric = ReedsSheppMetric(1.0)
metric([0, 0, 0], [1., 1., 1.])
arr = zeros(200)
sample_points!(metric, [0, 0, 0], [1., 1., 1.], arr)

path = create_path(metric, [0, 0, 0], [1., 1., 1.])
a = zeros(3)
interpolate(path, 0.1, a)

# compare 
using BenchmarkTools
@btime metric([0, 0, 0], [1., 1., 1.])
@btime create_path(metric, [0, 0, 0], [1., 1., 1.])
