module Xstar

using PyCall
const __reeds_shepp__ = PyCall.PyNULL()

using Plots
using StaticArrays
using LinearAlgebra
export ConfigurationSpace, BoxSpace, uniform_sampling, visualize!
export Euclidean, DirectReedsSheppMetric, PythonReedsSheppMetric, CppReedsSheppMetric, waypoints
export RRTStar, extend, is_obstacle_free


function __init__()
    copy!(__reeds_shepp__, pyimport("reeds_shepp"))
end

include("configuration_space.jl")
include("metric.jl")
include("rrtstar.jl")
include("reeds_shepp_specific.jl")

end # module
