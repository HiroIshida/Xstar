abstract type Metric end
worse_calculating(metric::Metric, x1, x2, r) = true

struct Euclidean <:Metric end
(::Euclidean)(x1, x2) = norm(x1 - x2)

abstract type ReedsSheppMetric <: Metric end
heuristic(metric::ReedsSheppMetric, x1::AbstractVector, x2::AbstractVector) = norm(x1[1:2] - x2[1:2])
worse_calculating(metric::ReedsSheppMetric, x1, x2, r) = heuristic(metric, x1, x2) < r

include("../cpp/call.jl")

struct PythonReedsSheppMetric <: ReedsSheppMetric
    r::Float64
end
(metric::PythonReedsSheppMetric)(x1, x2) = __reeds_shepp__.path_length(x1, x2, metric.r)
function waypoints(metric::PythonReedsSheppMetric, x1, x2, step)
    waypoints = __reeds_shepp__.path_sample(x1, x2, metric.r, step)
end
