using Revise
using Xstar
using Plots
using Random

using StaticArrays
using LinearAlgebra
gr()

x_start = [0., 0, 0]
x_goal = [0.2, 1, 1]

cspace = Xstar.EllipticSE2(2.0, x_start, x_goal)
xs = []
ys = []
for i in 1:1000
    pt = Xstar.uniform_sampling(cspace)
    push!(xs, pt[1])
    push!(ys, pt[2])
end

fig = plot(aspect_ratio=:equal)
scatter!(fig, xs, ys, label="", markercolor=:green)
scatter!(fig, [x_start[1], x_goal[1]], [x_start[2], x_goal[2]], label="", markercolor=:black, markersize=10)


