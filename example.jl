using Revise
using Xstar
using Plots

using StaticArrays
using LinearAlgebra
gr()

function Xstar.is_obstacle_free(rrtstar::RRTStar, x)
    # custom collision function
    return norm(x - [0.5, 0.5, 0.5]) > 0.5
end


cspace = BoxSpace(3, [0.0, 0, 0.0], [1., 1., 1.])
x_start = SVector{3, Float64}([0.1, 0.1, 0.1])
x_goal = SVector{3, Float64}([0.9, 0.9, 0.9])
rrtstar = RRTStar(cspace, x_start, x_goal) 
@time for i in 1:4000
    extend(rrtstar)
end

fig = plot()
visualize!(rrtstar, fig)

# plot circle
function circle(h, k, r)
    θ = LinRange(0, 2*π, 500)
    h .+ r*sin.(θ), k .+ r*cos.(θ)
end
plot!(fig, circle(0.5, 0.5, 0.3), label="", seriestype=[:shape],
      c=:blue, linecolor=:black, lw=0.5, fillalpha=0.2)



