using Revise
using Xstar
using Plots

using StaticArrays
gr()

cspace = BoxSpace(2, [0.0, 0], [1., 1.])
x_start = SVector{2, Float64}([0.1, 0.1])
x_goal = SVector{2, Float64}([0.9, 0.9])
rrtstar = RRTStar(cspace, x_start, x_goal) 
for i in 1:2000
    extend(rrtstar)
end

fig = plot()
visualize!(rrtstar, fig)


