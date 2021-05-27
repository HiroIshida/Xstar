module Xstar

using Plots
using StaticArrays
using LinearAlgebra
export ConfigurationSpace, BoxSpace, uniform_sampling, visualize!
export RRTStar, extend, is_obstacle_free

abstract type ConfigurationSpace{N} end
uniform_sampling(cspace::ConfigurationSpace) = error("please implement this")

struct BoxSpace{N} <: ConfigurationSpace{N}
    dim::Int
    lo::SVector{N, Float64}
    hi::SVector{N, Float64}
end
BoxSpace(dim::Int, lo, hi) = BoxSpace{dim}(dim, SVector{dim, Float64}(lo), SVector{dim, Float64}(hi))

function uniform_sampling(box::BoxSpace)
    width = box.hi - box.lo
    r = box.lo + width .* rand(Float64, box.dim)
end

mutable struct Node{N}
    x::SVector{N, Float64}
    cost::Float64
    idx::Int
    parent_idx::Union{Int, Nothing}
end
Node(x, cost, idx) = Node(x, cost, idx, nothing)

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

mutable struct RRTStar{N}
    dim::Int
    cspace::ConfigurationSpace
    x_start::SVector{N}
    x_goal::SVector{N}
    nodes::Vector{Node{N}}
    metric::Metric
    mu::Float64
end
function RRTStar(cspace::ConfigurationSpace{N}, x_start, x_goal; mu=0.1) where N
    nodes = Vector{Node{N}}(undef, 0)
    push!(nodes, Node(x_start, 0.0, 1))
    RRTStar{N}(N, cspace, x_start, x_goal, nodes, Euclidean(), mu)
end

function extend(rrtstar::RRTStar{N}) where N
    x_rand = uniform_sampling(rrtstar.cspace)
    node_nearest, x_new = _find_nearest_and_new(rrtstar, x_rand)
    if is_obstacle_free(rrtstar, node_nearest, x_new) # TODO path

        # find best node and cost
        node_min = node_nearest
        cost_min = node_nearest.cost + rrtstar.metric(node_nearest.x, x_new)

        node_nears = _find_nears(rrtstar, x_new)
        for node_near in node_nears
            # TODO cache metric computation
            cost_expect = node_near.cost + rrtstar.metric(node_near.x, x_new)
            if cost_expect < cost_min
                node_min = node_near
                cost_min = cost_expect
            end
        end

        idx_new = length(rrtstar.nodes) + 1
        node_new = Node(x_new, cost_min, idx_new, node_min.idx)
        push!(rrtstar.nodes, node_new)

        # rewire
        for node_near in node_nears
            is_obstacle_free(rrtstar, node_new, node_near.x) || continue
            if node_new.cost + rrtstar.metric(node_near.x, x_new) < node_near.cost
                node_near.parent_idx = node_new.idx
            end
        end
    end
end

is_obstacle_free(rrtstar::RRTStar, x) = error("please implement this")

function is_obstacle_free(rrtstar::RRTStar, node_start::Node, x_target)
    # assume x1 is known to be obstacle free
    n_wp = 5
    x_start = node_start.x
    step = (x_target - x_start)/n_wp
    for i in 1:n_wp
        is_obstacle_free(rrtstar, x_start + step * i) || (return false)
    end
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

function visualize!(rrtstar::RRTStar, fig)
    visualize!(rrtstar.cspace, fig)
    xs, ys = [[n.x[i] for n in rrtstar.nodes] for i in 1:2]
    scatter!(fig, xs, ys, label="", marker=(:green))

    for node in rrtstar.nodes
        isnothing(node.parent_idx) && continue

        node_parent = rrtstar.nodes[node.parent_idx]
        x_child = node.x
        x_parent = node_parent.x
        xs, ys = [[x_child[i], x_parent[i]] for i in 1:2]
        plot!(fig, xs, ys, label="", line=(:black))
    end
end




end # module
