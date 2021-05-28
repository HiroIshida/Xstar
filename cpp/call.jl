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

mutable struct CppReedsSheppMetric <: ReedsSheppMetric
    ptr::Ptr{Nothing}
    path_cache::Union{Nothing, ReedsSheppPath}
    function CppReedsSheppMetric(r::Float64)
        delete(metric::CppReedsSheppMetric) = c_rsspace_delete(metric.ptr)
        ptr = c_rspace_cleate(r)
        metric = new(ptr, nothing)
        finalizer(delete, metric)
        return metric
    end
end
function (metric::CppReedsSheppMetric)(q0, q1) 
    path = create_path(metric, q0, q1)
    metric.path_cache = path
    return distance(path)
end
create_path(metric::CppReedsSheppMetric, q0, q1) = ReedsSheppPath(c_rspath_create(metric.ptr, q0, q1), metric.ptr, q0)
function waypoints(metric::CppReedsSheppMetric, q0, q1, step)
    path = create_path(metric, q0, q1)
    dist = distance(path)
    num = Int(floor(dist/step) + 1)
    pts = SVector{3, Float64}[]
    for i in 1:num
        a = zeros(3)
        interpolate(path, (i-1) * step, a)
        push!(pts, a)
    end
    return pts
end

function test()
    metric = CppReedsSheppMetric(0.2)
    path = create_path(metric, [0, 0, 0], [1., 1., 1.])
    a = zeros(3)
    interpolate(path, 0.3, a)
    println(a)
end

