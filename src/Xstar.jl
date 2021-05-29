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

abstract type ConfigurationSpace{N} end
uniform_sampling(cspace::ConfigurationSpace) = error("please implement this")

struct BoxSpace{N} <: ConfigurationSpace{N}
    dim::Int
    lo::SVector{N, Float64}
    hi::SVector{N, Float64}
end
BoxSpace(dim::Int, lo, hi) = BoxSpace{dim}(dim, SVector{dim, Float64}(lo), SVector{dim, Float64}(hi))
function is_inside(space::BoxSpace, x)
    for i in 1:space.dim
        x[i] < space.lo[i] && (return false)
        x[i] > space.hi[i] && (return false)
    end
    return true
end

function uniform_sampling(box::BoxSpace)
    width = box.hi - box.lo
    r = box.lo + width .* rand(Float64, box.dim)
end

struct EllipticSE2 <: ConfigurationSpace{3}
    x_center::SVector{2, Float64} # TODO Euclid 2
    r1::Float64
    r2::Float64
    angle::Float64
end
function EllipticSE2(c_best, x_start, x_goal)
    c_min = norm(x_start[1:2] - x_goal[1:2])
    r1 = c_best * 0.5
    r2 = sqrt(c_best^2 - c_min^2) * 0.5
    angle = atan(x_goal[2] - x_start[2], x_goal[1] - x_start[1])
    center = 0.5 * (x_start[1:2] + x_goal[1:2])
    EllipticSE2(center, r1, r2, angle)
end

function uniform_sampling(cspace::EllipticSE2)
    # just rejection sampling is find because only 2dim 
    while true
        p = 2 * rand(Float64, 2) .* [cspace.r1, cspace.r2] .- [cspace.r1, cspace.r2]
        is_inside = (p[1]/cspace.r1)^2 + (p[2]/cspace.r2)^2 < 1.0
        if is_inside
            a = cspace.angle
            mat = [cos(a) -sin(a); sin(a) cos(a)]
            p_ = mat * p + cspace.x_center
            theta_ = rand() * 2 * π - π
            return SVector{3, Float64}([p_[1], p_[2], theta_])
        end
    end
end

mutable struct Node{N}
    x::SVector{N, Float64}
    cost::Float64
    cost_to_goal::Float64
    idx::Int
    parent_idx::Union{Int, Nothing}
end
Node(x, cost, idx) = Node(x, cost, Inf, idx, nothing)

function visualize!(box::BoxSpace, fig)
    v1 = [box.lo[1], box.lo[2]]
    v2 = [box.hi[1], box.lo[2]]
    v3 = [box.hi[1], box.hi[2]]
    v4 = [box.lo[1], box.hi[2]]
    V = [v1, v2, v3, v4]
    for i in 1:4
        if i==4
            pre, post = V[4], V[1]
        else
            pre, post = V[i], V[i+1]
        end
        xs = [pre[1], post[1]]
        ys = [pre[2], post[2]]
        plot!(fig, xs, ys, label="", line=(:red))
    end
end

abstract type Metric end
struct Euclidean <:Metric end
(::Euclidean)(x1, x2) = norm(x1 - x2)

abstract type ReedsSheppMetric <: Metric end

include("../cpp/call.jl")

struct PythonReedsSheppMetric <: ReedsSheppMetric
    r::Float64
end
(metric::PythonReedsSheppMetric)(x1, x2) = __reeds_shepp__.path_length(x1, x2, metric.r)
function waypoints(metric::PythonReedsSheppMetric, x1, x2, step)
    waypoints = __reeds_shepp__.path_sample(x1, x2, metric.r, step)
end


mutable struct RRTStar{N, MT}
    dim::Int
    cspace::ConfigurationSpace
    x_start::SVector{N}
    x_goal::SVector{N}
    sol_list::Vector{Node{N}}
    goal_node::Node{N}
    nodes::Vector{Node{N}}
    metric::MT
    mu::Float64
end
function RRTStar(cspace::ConfigurationSpace{N}, x_start, x_goal; mu=0.2, metric=nothing) where N
    if isnothing(metric)
        metric = Euclidean()
    end
    nodes = Vector{Node{N}}(undef, 0)
    goal_node = Node(x_goal, Inf, -1)
    push!(nodes, Node(x_start, 0.0, 1))
    sol_list = Vector{Node{N}}(undef, 0)
    RRTStar{N, typeof(metric)}(N, cspace, x_start, x_goal, sol_list, goal_node, nodes, metric, mu)
end
solution_found(rrtstar::RRTStar) = !isempty(rrtstar.sol_list)

function extend(rrtstar::RRTStar{N}) where N
    if solution_found(rrtstar)
        cost, idx_best = findmin([node.cost + node.cost_to_goal for node in rrtstar.sol_list])
        rrtstar.goal_node.cost = cost
        rrtstar.goal_node.parent_idx = rrtstar.sol_list[idx_best].idx
    end

    if solution_found(rrtstar)
        ellipse = EllipticSE2(rrtstar.goal_node.cost, rrtstar.x_start, rrtstar.x_goal)
        x_rand = uniform_sampling(ellipse)
    else
        x_rand = uniform_sampling(rrtstar.cspace)
    end
    node_nearest, x_new = _find_nearest_and_new(rrtstar, x_rand)
    is_obstacle_free(rrtstar, node_nearest, x_new) || (return false) 

    # find best node and cost
    node_min = node_nearest
    cost_min = node_nearest.cost + rrtstar.metric(node_nearest.x, x_new)

    node_nears = _find_nears(rrtstar, x_new)
    for node_near in node_nears
        # TODO cache metric computation
        cost_expect = node_near.cost + rrtstar.metric(node_near.x, x_new)
        if cost_expect < cost_min
            if is_obstacle_free(rrtstar, node_near, x_new)
                node_min = node_near
                cost_min = cost_expect
            end
        end
    end

    idx_new = length(rrtstar.nodes) + 1
    @assert is_obstacle_free(rrtstar, x_new)
    node_new = Node(x_new, cost_min, Inf, idx_new, node_min.idx)
    push!(rrtstar.nodes, node_new)

    # rewire
    for node_near in node_nears
        is_obstacle_free(rrtstar, node_new, node_near.x) || continue
        if node_new.cost + rrtstar.metric(node_near.x, x_new) < node_near.cost
            node_near.parent_idx = node_new.idx
        end
    end

    is_reached = is_obstacle_free(rrtstar, node_new, rrtstar.x_goal)
    if is_reached
        node_new.cost_to_goal = rrtstar.metric(node_new.x, rrtstar.x_goal)
        push!(rrtstar.sol_list, node_new)
    end
    return is_reached
end

is_obstacle_free(rrtstar::RRTStar, x) = error("please implement this")

function is_obstacle_free(rrtstar::RRTStar{<:Any, Euclidean}, node_start::Node, x_target)
    # assume x1 is known to be obstacle free
    n_wp = 5
    x_start = node_start.x
    step = (x_target - x_start)/n_wp
    for i in 1:n_wp
        is_obstacle_free(rrtstar, x_start + step * i) || (return false)
    end
    return true
end

function is_obstacle_free(rrtstar::RRTStar{3, <:ReedsSheppMetric}, node_start::Node, x_target)
    pts = waypoints(rrtstar.metric, node_start.x, x_target, 0.01)
    for pt in pts
        p = SVector{3, Float64}([pt[1], pt[2], pt[3]])
        is_obstacle_free(rrtstar, p) || (return false)
        is_inside(rrtstar.cspace, p) || (return false)
    end
    is_obstacle_free(rrtstar, x_target) || (return false)

    return true
end


function _find_nears(rrtstar::RRTStar{N}, x_center) where N
    gamma = 1.0
    card = length(rrtstar.nodes)
    r = min(gamma * (log(card)/card)^(1/N), rrtstar.mu)
    predicate = (node)->(rrtstar.metric(node.x, x_center) < r)
    nodes_nears = filter(predicate, rrtstar.nodes) # TODO doit lazy
end

function _find_nearest_and_new(rrtstar::RRTStar, x_rand)
    dist_min, idx_min = findmin((i)->rrtstar.metric(rrtstar.nodes[i].x, x_rand), 1:length(rrtstar.nodes))
    node_nearest = rrtstar.nodes[idx_min]
    x_nearest = node_nearest.x
    if dist_min < rrtstar.mu
        x_new = x_rand
    else
        x_new = x_nearest + normalize(x_rand - x_nearest) * rrtstar.mu
    end
    return node_nearest, x_new
end

function truncated_point(rrtstar::RRTStar, x_nearest, x_rand)
    x_new = x_nearest + normalize(x_rand - x_nearest) * rrtstar.mu
end

function truncated_point(rrtstar::RRTStar{3, <:ReedsSheppMetric}, x_nearest, x_rand)
    pts = waypoints(rrtstar.metric, x_nearest, x_rand, rrtstar.mu)
    pt = pts[2]
    return SVector{3, Float64}(pt[1], pt[2], pt[3])
end

function visualize_nodes!(rrtstar::RRTStar, nodes, fig)
    for node in nodes
        isnothing(node.parent_idx) && continue

        node_parent = rrtstar.nodes[node.parent_idx]
        x_child = node.x
        x_parent = node_parent.x
        xs, ys = [[x_child[i], x_parent[i]] for i in 1:2]
        plot!(fig, xs, ys, label="", line=(:black))
    end
end

function visualize_nodes!(rrtstar::RRTStar{3, <:ReedsSheppMetric}, nodes, fig; color=:black, width=0.5)
    for node in nodes
        isnothing(node.parent_idx) && continue

        node_parent = rrtstar.nodes[node.parent_idx]
        x_child = node.x
        x_parent = node_parent.x

        pts = waypoints(rrtstar.metric, x_child, x_parent, 0.005)
        xs = [p[1] for p in pts]
        ys = [p[2] for p in pts]
        plot!(fig, xs, ys, label="", linecolor=color, linewidth=width, alpha=0.5)
    end
end

function visualize!(rrtstar::RRTStar, fig; with_arrow=false, with_solution=true)
    visualize!(rrtstar.cspace, fig)
    xs, ys = [[n.x[i] for n in rrtstar.nodes] for i in 1:2]
    scatter!(fig, xs, ys, label="", markercolor=:green, markersize=5, markeralpha=0.5)
    #=
    u = cos.(zs) * rrtstar.mu * 0.5
    v = sin.(zs) * rrtstar.mu * 0.5
    with_arrow && quiver!(fig, xs, ys, quiver=(u, v))
    =#
    visualize_nodes!(rrtstar, rrtstar.nodes, fig)
    if with_solution && solution_found(rrtstar)
        nodes_path = back_trace(rrtstar, rrtstar.goal_node)
        xs, ys = [[n.x[i] for n in nodes_path] for i in 1:2]
        scatter!(fig, xs, ys, label="", markercolor=:blue, markersize=5, markeralpha=1.0)
        visualize_nodes!(rrtstar, nodes_path, fig; color=:blue, width=2.0)
    end
end
function back_trace(rrtstar::RRTStar, goal_node::Node)
    nodes = Node[]
    push!(nodes, goal_node)
    node = goal_node
    while node.idx!=1
        node = rrtstar.nodes[node.parent_idx]
        @assert is_obstacle_free(rrtstar, node.x)
        push!(nodes, node)
    end
    reverse!(nodes)
    return nodes
end




end # module
