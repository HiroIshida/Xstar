module Xstar

using Plots
using StaticArrays
export ConfigurationSpace, BoxSpace, uniform_sampling, visualize!
export RRTStar, extend

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

struct Node{N}
    x::SVector{N, Float64}
    cost::Float64
end

function visualize!(box::BoxSpace{2}, fig)
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

mutable struct RRTStar{N}
    dim::Int
    cspace::ConfigurationSpace
    x_start::SVector{N}
    x_goal::SVector{N}
    nodes::Vector{Node{N}}
end
function RRTStar(cspace::ConfigurationSpace{N}, x_start, x_goal) where N
    nodes = Vector{Node{N}}(undef, 0)
    RRTStar{N}(N, cspace, x_start, x_goal, nodes)
end

function extend(rrtstar::RRTStar{N}) where N
    x_new = uniform_sampling(rrtstar.cspace)
    node_new = Node{N}(x_new, 0.0)
    push!(rrtstar.nodes, node_new)
end

function visualize!(rrtstar::RRTStar{2}, fig)
    visualize!(rrtstar.cspace, fig)
    xs, ys = [[n.x[i] for n in rrtstar.nodes] for i in 1:2]
    scatter!(fig, xs, ys, label="", marker=(:green))
end




end # module
